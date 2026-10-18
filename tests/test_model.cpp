// Tests for the PS2 VIF/GIF helpers and the GTM1 car/wheel model decoders.

#include "test_framework.h"

#include "formats/model/car_model.h"
#include "formats/model/gtm1.h"
#include "formats/model/wheel_model.h"
#include "formats/ps2/vif.h"

#include <cmath>
#include <cstring>

using namespace gt3r;

namespace {

void put32(Bytes& b, u32 v) {
    for (int i = 0; i < 4; ++i) b.push_back(u8(v >> (8 * i)));
}
void putf(Bytes& b, float f) {
    u32 v;
    std::memcpy(&v, &f, 4);
    put32(b, v);
}
void pad4(Bytes& b) {
    while (b.size() % 4) b.push_back(0);
}

// One VIF packet: GIF tag, 4 positions in one strip, normals, UVs.
Bytes make_packet() {
    Bytes b;
    put32(b, 0x6801C0C0);                          // UNPACK V3-32 num=1 @0xC0 (GIF tag)
    put32(b, 0x00008004), put32(b, 0x303E4000), put32(b, 0x00001412); // NLOOP 4, EOP, PRE, PRIM 0x7C, 3 regs
    put32(b, 0x6804C000);                          // positions
    const float pos[4][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}};
    for (auto& p : pos) putf(b, p[0]), putf(b, p[1]), putf(b, p[2]);
    put32(b, 0x6202C040);                          // S-8 num=2 @0x40: 1 strip of (4-2)*3
    b.push_back(1), b.push_back(6);
    pad4(b);
    put32(b, 0x6804C080);                          // normals
    for (int i = 0; i < 4; ++i) putf(b, 0), putf(b, 0), putf(b, 1);
    put32(b, 0x7404C040);                          // masked V2-32 @0x40: UVs
    for (auto& p : pos) putf(b, p[0]), putf(b, p[1]);
    while (b.size() % 16) put32(b, 0);
    return b;
}

// Minimal GTM1: 1 node (draw chunk 0), 1 chunk with the packet above, no textures.
Bytes make_gtm1() {
    Bytes pkt = make_packet();
    Bytes g(0x40, 0);
    std::memcpy(g.data(), "GTM1", 4);
    g[0x10] = 1; // nodes
    g[0x12] = 1; // chunks
    auto set32 = [&](size_t o, u32 v) { for (int i = 0; i < 4; ++i) g[o + i] = u8(v >> (8 * i)); };
    set32(0x20, 0x40);
    set32(0x24, 0x44);
    g.resize(0x48, 0);
    set32(0x40, 0x50); // node 0 program
    set32(0x44, 0x60); // chunk 0
    g.resize(0x50, 0);
    // program: 04 00 (draw chunk 0), 00 (end)
    g.insert(g.end(), {0x04, 0x00, 0x00});
    g.resize(0x60, 0);
    // chunk header + one entry (offset relative to chunk), then the packet
    Bytes c(0x20, 0);
    c[8] = 1, c[10] = 1, c[12] = 4, c[14] = 2;
    c[16] = 0x20, c[20] = u8(pkt.size() / 16), c[22] = 0x00;
    g.insert(g.end(), c.begin(), c.end());
    g.insert(g.end(), pkt.begin(), pkt.end());
    return g;
}

} // namespace

TEST(vif_parse_and_unpack) {
    Bytes b = make_packet();
    std::vector<ps2::VifCommand> cmds = ps2::parse_vif(b);
    CHECK(cmds.size() >= 5);
    CHECK(cmds[0].is_unpack());
    ps2::VifUnpack u = cmds[1].unpack();
    CHECK(u.vn == 2 && u.vl == 0 && u.count == 4 && u.addr == 0 && u.usn && u.flg && !u.masked);
    auto f = ps2::unpack_floats(u);
    CHECK(f[3][0] == 1.0f && f[3][1] == 1.0f);
    CHECK(cmds[4].unpack().masked);
    ps2::GifTag t = ps2::GifTag::decode(cmds[0].payload);
    CHECK(t.nloop == 4 && t.eop && t.pre && t.nreg == 3);
    CHECK(t.reg(0) == ps2::kGifSt && t.reg(1) == ps2::kGifRgbaq && t.reg(2) == ps2::kGifXyzf2);
    ps2::Prim p = ps2::Prim::decode(t.prim);
    CHECK(p.type == 4 && p.iip && p.tme && p.fge && p.abe);
}

TEST(vif_signed_8bit) {
    Bytes b;
    put32(b, 0x6E02C000 & ~0x4000u); // V4-8 signed, num=2
    for (int v : {0x80, 0x7F, 0xFF, 0x00, 1, 2, 3, 4}) b.push_back(u8(v));
    auto cmds = ps2::parse_vif(b);
    auto w = ps2::unpack_words(cmds[0].unpack());
    CHECK(s32(w[0][0]) == -128 && w[0][1] == 127 && s32(w[0][2]) == -1 && w[1][3] == 4);
}

TEST(gtm1_synthetic_model) {
    Bytes g = make_gtm1();
    scene::Model m = gtm1::decode_gtm1(g);
    CHECK(m.meshes.size() == 1);
    if (m.meshes.empty()) return;
    const scene::Mesh& mesh = m.meshes[0];
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);
    CHECK(mesh.primitives.size() == 1 && mesh.primitives[0].texture == -1);
    // Counter-clockwise when seen from +Z, matching the +Z normals.
    const float* a = mesh.vertices[mesh.indices[0]].pos;
    const float* b = mesh.vertices[mesh.indices[1]].pos;
    const float* c = mesh.vertices[mesh.indices[2]].pos;
    float cross_z = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    CHECK(cross_z > 0);
    CHECK(m.bounds_max[0] == 1.0f && m.bounds_max[1] == 1.0f);
}

TEST(car_model_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Bytes data = disc->volume().read(*disc->volume().find("cars/day/am0006"));
    CHECK(CarFile::is_car(data));
    for (int lod = 0; lod < 3; ++lod) {
        CarModelOptions o;
        o.lod = lod;
        scene::Model m = load_car_model(data, o, "am0006");
        CHECK(!m.meshes.empty() && m.meshes[0].name == "body");
        CHECK(!m.textures.empty());
        // A car is roughly 4-5 m long, 1.2-1.4 m tall, 1.7-2.1 m wide.
        float len = m.bounds_max[2] - m.bounds_min[2];
        CHECK(len > 3.5f && len < 5.5f);
        CHECK(m.bounds_max[1] > 1.0f && m.bounds_max[1] < 1.6f);
        bool textured = false;
        for (const scene::Primitive& p : m.meshes[0].primitives) textured |= p.texture >= 0;
        CHECK(textured);
    }
    CarModelOptions o;
    o.wheels = false;
    scene::Model lod0 = load_car_model(data, o);
    size_t tris = lod0.meshes[0].indices.size() / 3;
    CHECK(tris > 2000 && tris < 6000);
}

TEST(wheel_model_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    size_t n = 0;
    for (const char* id : {"wheel/am0006", "wheel/enw001_s", "wheel/not_exist"}) {
        Bytes data = disc->volume().read(*disc->volume().find(id));
        CHECK(WheelFile::is_wheel(data));
        scene::Model m = load_wheel_model(data);
        CHECK(!m.meshes.empty());
        // Unit-scale rim: radius about 1.
        CHECK(m.bounds_max[1] > 0.8f && m.bounds_max[1] < 1.5f);
        ++n;
    }
    CHECK(n == 3);
}

TEST(car_paint_variants_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Bytes data = disc->volume().read(*disc->volume().find("cars/day/am0006"));
    CarFile f = CarFile::parse(data);
    gtm1::Gtm1 g = gtm1::Gtm1::parse(f.body);
    CHECK(g.texsets.size() == 4);
    ByteSpan set0 = f.body.subspan(g.texsets[0]);
    std::vector<gtm1::PaintVariant> v = gtm1::paint_variants(set0);
    CHECK(v.size() == 7); // seven colours in database/carcolor.db for am0006
    if (v.size() < 7) return;
    // Texture 19 (512x128 side panel) is painted: British Racing Green by default (TEX0.CSA honoured),
    // Braemar Blue for colour 3.
    auto mean = [](const scene::Texture& t) {
        double s[3] = {0, 0, 0};
        for (u32 p : t.rgba) for (int k = 0; k < 3; ++k) s[k] += (p >> (8 * k)) & 0xFF;
        for (double& x : s) x /= double(t.rgba.size());
        return std::array<double, 3>{s[0], s[1], s[2]};
    };
    std::vector<scene::Texture> t0, t3;
    gtm1::decode_texture_set(set0, t0, "", 0);
    gtm1::decode_texture_set(set0, t3, "", 3);
    CHECK(t0.size() == 43 && t3.size() == 43);
    auto g0 = mean(t0[19]), b3 = mean(t3[19]);
    CHECK(g0[1] > g0[0] + 5 && g0[1] > g0[2]); // green
    CHECK(b3[2] > b3[0] + 10);                   // blue
}

TEST(car_alpha_test_and_reflection_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Bytes data = disc->volume().read(*disc->volume().find("cars/day/ni0137")); // R34 GT-R V-spec
    CarModelOptions o;
    o.wheels = false;
    scene::Model m = load_car_model(data, o, "ni0137");
    CHECK(!m.meshes.empty());
    if (m.meshes.empty()) return;
    // The body's side panels are alpha-tested (wheel-arch cut-outs) and the reflection passes
    // become additive environment layers weighted by vertex alpha.
    size_t cut = 0, refl = 0, weighted = 0, per_pixel = 0;
    const scene::Mesh& body = m.meshes[0];
    for (const scene::Primitive& p : body.primitives) {
        if (p.flags & scene::kAlphaTest) {
            ++cut;
            CHECK(p.texture >= 0);
            if (p.texture >= 0)
                for (u32 c : m.textures[size_t(p.texture)].rgba) CHECK((c >> 24) == 0 || (c >> 24) == 255);
        }
        u32 want = scene::kAdditive | scene::kEnvironmentMap | scene::kUnlit | scene::kNoDepthWrite;
        // Reflection triangles over one textured pass carry that texture: its alpha is the reflectivity.
        if ((p.flags & want) == want && p.texture >= 0) ++per_pixel;
        if ((p.flags & want) == want && p.texture < 0) {
            ++refl;
            for (u32 i = 0; i < p.index_count; ++i)
                if ((body.vertices[body.indices[p.first_index + i]].color >> 24) > 0) { ++weighted; break; }
        }
    }
    CHECK(cut > 0);
    CHECK(refl > 0);
    CHECK(weighted > 0);
    CHECK(per_pixel > 0);
}
