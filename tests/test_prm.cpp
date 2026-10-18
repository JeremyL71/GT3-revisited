// Tests for the PRZ0/PRM0 prize scene decoder (medals, trophies, license cards).

#include "test_framework.h"

#include "formats/model/prm_model.h"

#include <cmath>
#include <cstring>

using namespace gt3r;

namespace {

void put32(Bytes& b, size_t o, u32 v) {
    for (int i = 0; i < 4; ++i) b[o + size_t(i)] = u8(v >> (8 * i));
}
void putf(Bytes& b, size_t o, float f) {
    u32 v;
    std::memcpy(&v, &f, 4);
    put32(b, o, v);
}
void put_matrix(Bytes& b, size_t o, const prm::Matrix& m) {
    for (int k = 0; k < 16; ++k) putf(b, o + 4 * size_t(k), m[size_t(k)]);
}

prm::Matrix rot_y(float a, float tz) {
    float c = std::cos(a), s = std::sin(a);
    return {c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, tz, 1};
}

bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

// PRZ0 header, 1 light, a static view matrix, 3 model keys, then a stub "GTM1".
Bytes make_prz0() {
    Bytes b(0x200, 0);
    std::memcpy(b.data(), "PRZ0", 4);
    putf(b, 0x14, 1.0f); // last frame
    putf(b, 0x18, 0.0f); // loop frame
    putf(b, 0x1C, 30.0f);
    b[0x20] = 1, b[0x22] = 3;
    put32(b, 0x24, 0x80);
    put32(b, 0x28, 0xC0);
    put32(b, 0x2C, 0x180);
    put32(b, 0x30, 0x40);
    b[0x34] = 1; // single view matrix
    putf(b, 0x40, 0.5f), putf(b, 0x44, 0.25f), putf(b, 0x48, 1.0f); // light direction
    putf(b, 0x50, 0.8f);                                              // diffuse r
    put_matrix(b, 0x80, {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -10, 1});
    put_matrix(b, 0xC0, rot_y(0, 0));
    put_matrix(b, 0x100, rot_y(1.0f, 2));
    put_matrix(b, 0x140, rot_y(2.0f, 4));
    std::memcpy(b.data() + 0x180, "GTM1", 4);
    return b;
}

} // namespace

TEST(prm_header_and_tracks) {
    Bytes b = make_prz0();
    CHECK(prm::PrmFile::is_prm(b));
    prm::PrmFile p = prm::PrmFile::parse(b);
    CHECK(!p.prm0);
    CHECK(p.view_keys.size() == 1 && p.model_keys.size() == 3);
    CHECK(p.lights.size() == 1 && near(p.lights[0].direction[1], 0.25f) && near(p.lights[0].diffuse[0], 0.8f));
    CHECK(p.gtm1.size() == 0x80);
    // 30 fps: half a frame per 60 Hz tick; past last_frame + 1 it wraps to loop_frame.
    CHECK(near(p.advance(0.0f), 0.5f));
    CHECK(near(p.advance(1.75f), 0.25f));
    // Interpolation: halfway between keys 0 and 1 is a 0.5 rad rotation, translation 1.
    prm::Matrix m = p.model_at(0.5f);
    CHECK(near(m[0], std::cos(0.5f), 1e-3f) && near(m[8], std::sin(0.5f), 1e-3f));
    CHECK(near(m[14], 1.0f));
    // The key after last_frame (1) is loop_frame (0).
    prm::Matrix w = p.model_at(1.5f);
    CHECK(near(w[0], std::cos(0.5f), 1e-3f) && near(w[14], 1.0f));
    CHECK(near(p.view_at(0.7f)[14], -10.0f));
    // Shadow: projects along the light onto y = -0.1.
    prm::Matrix s;
    CHECK(prm::shadow_matrix(p.lights[0], 0, s));
    float y = 1.0f; // point (0, 1, 0)
    CHECK(near(y * s[4], -2.0f) && near(y * s[5] + s[13], -0.1f) && near(y * s[6], -4.0f));
    // Truncated tracks are rejected.
    Bytes bad = b;
    bad[0x22] = 200;
    bool threw = false;
    try {
        prm::PrmFile::parse(bad);
    } catch (const FormatError&) {
        threw = true;
    }
    CHECK(threw);
}

TEST(prm_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    // Race medals/trophies: static camera, 322 model keys, 2 lights, GT logo trophy.
    for (const char* id : {"race/gold.prm", "race/silver.prm", "race/bronze.prm"}) {
        Bytes data = disc->volume().read(*disc->volume().find(id));
        prm::PrmFile p = prm::PrmFile::parse(data);
        CHECK(p.view_keys.size() == 1 && p.model_keys.size() == 322 && p.lights.size() == 2);
        CHECK(near(p.last_frame, 320) && near(p.loop_frame, 120) && near(p.fps, 60));
        // Flies in from z = -43.7 to 50.8 by the loop frame, in front of a camera at -75.2.
        CHECK(p.model_at(0)[14] < -40 && p.model_at(120)[14] > 50 && p.view_at(0)[14] < -75);
        scene::Model m = prm::load_prm_model(data, {}, id);
        CHECK(m.meshes.size() == 1 && !m.textures.empty());
        CHECK(m.bounds_max[0] > 5 && m.bounds_max[0] < 6 && m.bounds_max[1] > 3 && m.bounds_max[1] < 4.5f);
        size_t env = 0, additive = 0;
        for (const scene::Primitive& pr : m.meshes[0].primitives) {
            env += (pr.flags & scene::kEnvironmentMap) && pr.texture >= 0;
            additive += (pr.flags & scene::kAdditive) != 0;
        }
        CHECK(env > 10 && additive > 0);
        CHECK(prm::load_prm_shadow(data).meshes.empty());
        // Sphere map: a normal facing the camera samples the map centre.
        prm::apply_sphere_map(m, prm::multiply(p.model_at(120), p.view_at(120)));
    }
    // License card: animated card with a shadow node and the emblem drawn through its alpha mask.
    Bytes lic = disc->volume().read(*disc->volume().find("menu/lic_a.prm"));
    prm::PrmFile p = prm::PrmFile::parse(lic);
    CHECK(p.model_keys.size() == 322 && near(p.display[0], 16.0f));
    scene::Model card = prm::load_prm_model(lic, {}, "lic_a");
    CHECK(card.meshes.size() == 1);
    // About 8.0 x 5.9 units, flat in Y.
    float w = card.bounds_max[0] - card.bounds_min[0], d = card.bounds_max[2] - card.bounds_min[2];
    CHECK(w > 7.5f && w < 8.5f && d > 5.5f && d < 6.5f && card.bounds_max[1] < 0.5f);
    bool emblem = false;
    for (const scene::Primitive& pr : card.meshes[0].primitives)
        emblem |= pr.texture >= 0 && card.textures[size_t(pr.texture)].name.find("_mask") != std::string::npos;
    CHECK(emblem);
    CHECK(!prm::load_prm_shadow(lic).meshes.empty());
    // PRM0 (older layout): view track only, no lights.
    Bytes old = disc->volume().read(*disc->volume().find("race/prize.prm"));
    prm::PrmFile p0 = prm::PrmFile::parse(old);
    CHECK(p0.prm0 && p0.lights.empty() && p0.view_keys.size() == 121 && p0.model_keys.empty());
    CHECK(!prm::load_prm_model(old).meshes.empty());
}
