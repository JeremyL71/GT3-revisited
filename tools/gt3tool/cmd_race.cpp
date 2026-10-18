// Race workstream diagnostics.

#include "formats/course/course_file.h"
#include "formats/model/car_model.h"
#include "formats/model/gtm1.h"
#include "formats/gs_memory.h"
#include "formats/tex1.h"
#include "tool.h"

#include <cstdio>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <string>

namespace gt3r::tool {

namespace {

// Texture-LOD tables of a course's scenery GTM1: for every texture set slot,
// the Tex1 set each table points at and its texture sizes.
int race_texlod(Disc& disc, const Args& a) {
    Bytes data = disc.volume().read(need(disc.volume(), a[0]));
    course::CourseFile f = course::CourseFile::parse(data);
    int slot = a.size() > 1 ? std::stoi(a[1]) : course::kSlotScenery;
    ByteSpan s = f.section(data, slot);
    gtm1::Gtm1 g = gtm1::Gtm1::parse(s);
    std::printf("slot %d: %zu nodes, %zu chunks, %zu distinct sets, %zu texture-LOD tables\n", slot, g.nodes.size(),
                g.chunks.size(), g.texsets.size(), g.texset_tables.size());
    // How many draws use each set slot (op 0x2b/0x2c) is not tracked; list the tables.
    for (size_t i = 0; i < g.texset_tables[0].size(); ++i) {
        std::printf("set slot %2zu:", i);
        for (size_t t = 0; t < g.texset_tables.size(); ++t) {
            u32 idx = g.texset_tables[t][i];
            if (idx >= g.texsets.size()) {
                std::printf("  [T%zu none]", t);
                continue;
            }
            Tex1 tex = Tex1::parse(s.subspan(g.texsets[idx]));
            std::printf("  [T%zu set%u @%x:", t, idx, g.texsets[idx]);
            for (const Tex1::Texture& tx : tex.textures)
                std::printf(" %ux%u", tx.tex0.width(), tx.tex0.height());
            std::printf("]");
        }
        std::printf("\n");
    }
    return 0;
}

// Blended packets near a point: chunk flags, material word, PRIM, vertex alpha range
// and texture alpha range (to work out how road decals blend).
int race_blend(Disc& disc, const Args& a) {
    Bytes data = disc.volume().read(need(disc.volume(), a[0]));
    course::CourseFile f = course::CourseFile::parse(data);
    ByteSpan s = f.section(data, course::kSlotScenery);
    gtm1::Gtm1 g = gtm1::Gtm1::parse(s);
    float px = std::stof(a[1]), pz = std::stof(a[2]), rad = a.size() > 3 ? std::stof(a[3]) : 60.0f;
    bool all = a.size() > 4 && a[4] == "all";
    std::map<u32, std::vector<scene::Texture>> sets;
    for (u32 node = 0; node < g.nodes.size(); ++node) {
        for (const gtm1::Draw& d : g.run(node)) {
            const gtm1::Chunk& c = g.chunks[d.chunk];
            for (const gtm1::Packet& pk : c.packets) {
                gtm1::DecodedPacket dp = g.decode_packet(pk);
                bool near = false;
                for (const scene::Vertex& v : dp.vertices)
                    if (std::hypot(v.pos[0] - px, v.pos[2] - pz) < rad) near = true;
                if (!near || (!dp.prim.abe && !all)) continue;
                float u0 = 1e9f, u1 = -1e9f, v0 = 1e9f, v1 = -1e9f;
                for (const scene::Vertex& v : dp.vertices)
                    u0 = std::min(u0, v.uv[0]), u1 = std::max(u1, v.uv[0]), v0 = std::min(v0, v.uv[1]), v1 = std::max(v1, v.uv[1]);
                u32 amin = 255, amax = 0;
                for (const scene::Vertex& v : dp.vertices) amin = std::min(amin, v.color >> 24), amax = std::max(amax, v.color >> 24);
                u32 tex = pk.mat & 0x1FF;
                std::string tinfo = "-";
                if (tex && tex != 0x1FF && d.texset < g.texsets.size()) {
                    auto it = sets.find(d.texset);
                    if (it == sets.end()) {
                        std::vector<scene::Texture> v;
                        gtm1::decode_texture_set(s.subspan(g.texsets[d.texset]), v, "t");
                        it = sets.emplace(d.texset, std::move(v)).first;
                    }
                    if (tex - 1 < it->second.size()) {
                        const scene::Texture& t = it->second[tex - 1];
                        u32 tmin = 255, tmax = 0;
                        for (u32 c2 : t.rgba) tmin = std::min(tmin, c2 >> 24), tmax = std::max(tmax, c2 >> 24);
                        Tex1 tx = Tex1::parse(s.subspan(g.texsets[d.texset]));
                        const Tex0& t0 = tx.textures[tex - 1].tex0;
                        char b[160];
                        const auto& cl = tx.textures[tex - 1].clamp;
                        std::snprintf(b, sizeof(b), "set%u/%u %ux%u psm=%u tcc=%u tfx=%u texA %u..%u wms=%u wmt=%u uv %.2f..%.2f %.2f..%.2f",
                                      d.texset, tex - 1, t.width, t.height, t0.psm, t0.tcc, t0.tfx, tmin, tmax,
                                      u32(cl.wms), u32(cl.wmt), u0, u1, v0, v1);
                        tinfo = b;
                    }
                }
                if (all && a.size() > 5 && a[5] == "red") {
                    // Only packets whose texture x vertex colour comes out red-dominant.
                    double acc[3] = {};
                    if (tex && tex != 0x1FF && sets.count(d.texset) && tex - 1 < sets[d.texset].size()) {
                        const scene::Texture& t = sets[d.texset][tex - 1];
                        for (u32 c2 : t.rgba)
                            for (int k = 0; k < 3; ++k) acc[k] += (c2 >> (8 * k)) & 0xFF;
                        for (double& x : acc) x /= double(t.rgba.size());
                    } else acc[0] = acc[1] = acc[2] = 255;
                    double vc[3] = {};
                    for (const scene::Vertex& v : dp.vertices)
                        for (int k = 0; k < 3; ++k) vc[k] += (v.color >> (8 * k)) & 0xFF;
                    for (int k = 0; k < 3; ++k) acc[k] *= vc[k] / dp.vertices.size() / 255.0;
                    if (!(acc[0] > 1.8 * acc[1] && acc[0] > 60)) continue;
                    std::printf("RED %.0f %.0f %.0f  ", acc[0], acc[1], acc[2]);
                }
                std::printf("node %u chunk %u flags %04x mat %04x prim %02x nv %zu vA %u..%u %s\n", node, d.chunk,
                            c.flags, pk.mat, u32(dp.tag.prim & 0x7FF), dp.vertices.size(), amin, amax, tinfo.c_str());
            }
        }
    }
    return 0;
}

// For every chunk-flag-4 (second road pass) packet: how many of its vertices
// coincide with a vertex of the node's base pass, and that vertex's alpha.
int race_sheen(Disc& disc, const Args& a) {
    Bytes data = disc.volume().read(need(disc.volume(), a[0]));
    course::CourseFile f = course::CourseFile::parse(data);
    ByteSpan s = f.section(data, course::kSlotScenery);
    gtm1::Gtm1 g = gtm1::Gtm1::parse(s);
    size_t total = 0, matched = 0, packets = 0, lit = 0, uv = 0, envn = 0;
    std::map<u32, size_t> alpha_hist;
    std::map<u32, size_t> mats;
    for (u32 node = 0; node < g.nodes.size(); ++node) {
        std::map<std::array<float, 3>, u32> base;
        std::vector<gtm1::Draw> draws = g.run(node);
        for (const gtm1::Draw& d : draws) {
            if (g.chunks[d.chunk].flags & 4) continue;
            for (const gtm1::Packet& pk : g.chunks[d.chunk].packets)
                for (const scene::Vertex& v : g.decode_packet(pk).vertices) base[{v.pos[0], v.pos[1], v.pos[2]}] = v.color >> 24;
        }
        for (const gtm1::Draw& d : draws) {
            if (!(g.chunks[d.chunk].flags & 4)) continue;
            for (const gtm1::Packet& pk : g.chunks[d.chunk].packets) {
                gtm1::DecodedPacket dp = g.decode_packet(pk);
                ++packets;
                lit += dp.has_normals, uv += dp.has_uv, envn += dp.env_normals;
                ++mats[pk.mat];
                for (const scene::Vertex& v : dp.vertices) {
                    ++total;
                    auto it = base.find({v.pos[0], v.pos[1], v.pos[2]});
                    if (it != base.end()) ++matched, ++alpha_hist[it->second / 32];
                }
            }
        }
    }
    std::printf("%zu flag-4 packets (normals %zu, uv %zu, env-normals %zu), %zu vertices, %zu match a base vertex\n",
                packets, lit, uv, envn, total, matched);
    for (auto [k, n] : alpha_hist) std::printf("  base alpha %3u..%3u: %zu\n", k * 32, k * 32 + 31, n);
    for (auto [k, n] : mats) std::printf("  mat %04x: %zu packets\n", k, n);
    return 0;
}

// Body packets with most vertices inside a sphere (car frame): what is drawn there.
int race_carprims(Disc& disc, const Args& a) {
    Bytes data = disc.volume().read(need(disc.volume(), "cars/day/" + a[0]));
    CarFile cf = CarFile::parse(data);
    gtm1::Gtm1 g = gtm1::Gtm1::parse(cf.body);
    float c[3] = {std::stof(a[1]), std::stof(a[2]), std::stof(a[3])}, r = std::stof(a[4]);
    for (const gtm1::Draw& d : g.run(0, {0, 0, 0})) {
        const gtm1::Chunk& ch = g.chunks[d.chunk];
        for (const gtm1::Packet& pk : ch.packets) {
            gtm1::DecodedPacket dp = g.decode_packet(pk);
            size_t in = 0;
            u32 cmin = 0xFFFFFFFF, cmax = 0;
            for (const scene::Vertex& v : dp.vertices) {
                float dx = v.pos[0] - c[0], dy = v.pos[1] - c[1], dz = v.pos[2] - c[2];
                if (dx * dx + dy * dy + dz * dz < r * r) ++in;
                cmin = std::min(cmin, v.color), cmax = std::max(cmax, v.color);
            }
            if (in < 3) continue;
            std::printf("chunk %u flags %04x mat %04x prim %03x nv %zu in %zu normals %d colors %d uv %d env %d col %08x..%08x\n",
                        d.chunk, ch.flags, pk.mat, u32(dp.tag.prim & 0x7FF), dp.vertices.size(), in, dp.has_normals,
                        dp.has_colors, dp.has_uv, dp.env_normals, cmin, cmax);
        }
    }
    return 0;
}

} // namespace

static CommandRegistrar race_carprims_cmd{
    {"race-carprims", "<model> <x> <y> <z> <r>", "car body packets inside a sphere (car frame)", 5, race_carprims}};
static CommandRegistrar race_sheen_cmd{
    {"race-sheen", "<vol-path>", "second road pass (chunk flag 4) vs base pass vertices", 1, race_sheen}};
static CommandRegistrar race_blend_cmd{
    {"race-blend", "<vol-path> <x> <z> [radius] [all]", "blended scenery packets near a point", 3, race_blend}};
static CommandRegistrar race_texlod_cmd{
    {"race-texlod", "<vol-path> [slot]", "course scenery texture-LOD tables (set sizes per table)", 1, race_texlod}};

} // namespace gt3r::tool
