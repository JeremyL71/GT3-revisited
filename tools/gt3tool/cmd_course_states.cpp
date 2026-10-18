// course-states <vol-path> [slot]: GS render state survey of a course GTM1 block.
//
// For every packet drawn by the slot's node programs (all nodes, LOD 0) prints a
// histogram of (PRIM, ALPHA, TEST ATE/ATST/AREF/AFAIL, ZMSK, depth test, FBMSK),
// then the block's textures (size, PSM, CLAMP, TEX1 filter fields, alpha histogram)
// and the vertex colour / UV ranges per texture. Used to match the renderer's
// blend / alpha-test / depth-write state to the game's.

#include "tool.h"

#include "formats/course/course_file.h"
#include "formats/model/gtm1.h"
#include "formats/tex1.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace gt3r::tool {

namespace {

int cmd_course_states(Disc& disc, const Args& a) {
    Bytes data = disc.volume().read(need(disc.volume(), a[0]));
    int slot = a.size() > 1 ? std::atoi(a[1].c_str()) : 1;
    course::CourseFile file = course::CourseFile::parse(data);
    ByteSpan s = file.section(data, slot);
    if (s.empty() || !gtm1::Gtm1::is_gtm1(s)) {
        std::printf("slot %d: no GTM1\n", slot);
        return 1;
    }
    gtm1::Gtm1 g = gtm1::Gtm1::parse(s);
    if (a.size() > 5 && a[2] == "cover") {
        // ASCII XZ coverage map of the slot's triangles around (x, z): '#' covered by an opaque
        // packet, '+' only by blended ones, '.' nothing. Rows = +z down, columns = +x right.
        float cx = std::stof(a[3]), cz = std::stof(a[4]), r = std::stof(a[5]);
        float ymax = a.size() > 6 ? std::stof(a[6]) : 1e9f;
        const int n = 80;
        float cell = 2 * r / n;
        std::vector<int> cov(size_t(n) * n, 0);
        for (u32 node = 0; node < g.nodes.size(); ++node)
            for (const gtm1::Draw& d : g.run(node))
                for (const gtm1::Packet& pk : g.chunks[d.chunk].packets) {
                    gtm1::DecodedPacket dp = g.decode_packet(pk);
                    if ((d.fb_mask & 0x00FFFFFFu) == 0x00FFFFFFu) continue;
                    size_t k = 0;
                    for (u32 len : dp.strips) {
                        for (u32 i = 0; i + 2 < len; ++i) {
                            const float* p[3] = {dp.vertices[k + i].pos, dp.vertices[k + i + 1].pos, dp.vertices[k + i + 2].pos};
                            if (d.has_matrix || std::max({p[0][1], p[1][1], p[2][1]}) > ymax) continue;
                            for (int y = 0; y < n; ++y)
                                for (int x = 0; x < n; ++x) {
                                    float qx = cx - r + (x + 0.5f) * cell, qz = cz - r + (y + 0.5f) * cell;
                                    auto e = [&](const float* A, const float* B) {
                                        return (B[0] - A[0]) * (qz - A[2]) - (B[2] - A[2]) * (qx - A[0]);
                                    };
                                    float e0 = e(p[0], p[1]), e1 = e(p[1], p[2]), e2 = e(p[2], p[0]);
                                    if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0)) {
                                        int& c = cov[size_t(y) * n + x];
                                        c = std::max(c, dp.prim.abe ? 1 : 2);
                                        if (r < 1 && x == n / 2 && y == n / 2) {
                                            u32 amin = 255, amax = 0;
                                            for (const scene::Vertex& v : dp.vertices)
                                                amin = std::min(amin, v.color >> 24), amax = std::max(amax, v.color >> 24);
                                            std::printf("node %u chunk %u flags %04x mat %04x prim %02x y %.1f %.1f %.1f "
                                                        "vA %u..%u ALPHA %02x ATE %d %d/%02x AFAIL %d Z %d FBMSK %08x cull %d\n",
                                                        node, d.chunk, g.chunks[d.chunk].flags, pk.mat, dp.tag.prim & 0x7FF,
                                                        p[0][1], p[1][1], p[2][1], amin, amax, d.alpha_mode, d.alpha_test,
                                                        d.alpha_test_method, d.alpha_test_ref, d.alpha_fail, d.depth_write,
                                                        d.fb_mask, d.cull);
                                        }
                                    }
                                }
                        }
                        k += len;
                    }
                }
        for (int y = 0; y < n && r >= 1; ++y) {
            for (int x = 0; x < n; ++x) std::putchar(".+#"[cov[size_t(y) * n + x]]);
            std::putchar('\n');
        }
        return 0;
    }
    using Key = std::tuple<u32, u32, int, int, int, int, int, int, u32, int>;
    std::map<Key, size_t> hist;
    struct Range {
        u32 cmin[4] = {255, 255, 255, 255}, cmax[4] = {0, 0, 0, 0};
        float umin = 1e9f, umax = -1e9f, vmin = 1e9f, vmax = -1e9f;
        float ymin = 1e9f, ymax = -1e9f, rmax = 0;
        size_t verts = 0;
        u32 raw_max = 0;
    };
    std::map<std::pair<u32, u32>, Range> ranges; // (texset, tex) -> ranges
    for (u32 n = 0; n < g.nodes.size(); ++n) {
        if (!g.nodes[n]) continue;
        for (const gtm1::Draw& d : g.run(n)) {
            const gtm1::Chunk& c = g.chunks[d.chunk];
            for (const gtm1::Packet& pk : c.packets) {
                gtm1::DecodedPacket dp = g.decode_packet(pk);
                Key k{dp.tag.prim, d.alpha_mode, d.alpha_test, d.alpha_test_method, d.alpha_test_ref, d.alpha_fail,
                      d.depth_write, d.depth_test, d.fb_mask, d.cull};
                hist[k]++;
                Range& r = ranges[{d.texset, u32(pk.mat & 0x1FF)}];
                r.raw_max = std::max(r.raw_max, dp.raw_rgb_max);
                for (const scene::Vertex& v : dp.vertices) {
                    for (int i = 0; i < 4; ++i) {
                        u32 ch = (v.color >> (8 * i)) & 0xFF;
                        r.cmin[i] = std::min(r.cmin[i], ch), r.cmax[i] = std::max(r.cmax[i], ch);
                    }
                    r.umin = std::min(r.umin, v.uv[0]), r.umax = std::max(r.umax, v.uv[0]);
                    r.vmin = std::min(r.vmin, v.uv[1]), r.vmax = std::max(r.vmax, v.uv[1]);
                    r.ymin = std::min(r.ymin, v.pos[1]), r.ymax = std::max(r.ymax, v.pos[1]);
                    r.rmax = std::max(r.rmax, std::sqrt(v.pos[0] * v.pos[0] + v.pos[2] * v.pos[2]));
                    ++r.verts;
                }
            }
        }
    }
    std::printf("slot %d: %zu nodes, %zu chunks, %zu texture sets\n", slot, g.nodes.size(), g.chunks.size(),
                g.texsets.size());
    std::printf("  count  PRIM  ALPHA ATE ATST AREF AFAIL ZWRITE ZTEST FBMSK    CULL\n");
    for (const auto& [k, n] : hist)
        std::printf("  %5zu  0x%02x  0x%02x  %d   %d    0x%02x %d     %d      %d     %08x %d\n", n, std::get<0>(k),
                    std::get<1>(k), std::get<2>(k), std::get<3>(k), std::get<4>(k), std::get<5>(k), std::get<6>(k),
                    std::get<7>(k), std::get<8>(k), std::get<9>(k));
    bool detail = a.size() > 2 && a[2] == "tex";
    if (!detail) return 0;
    for (size_t ts = 0; ts < g.texsets.size(); ++ts) {
        ByteSpan sub = g.data.subspan(g.texsets[ts]);
        Tex1 t = Tex1::parse(sub);
        GsMemory gs;
        t.upload(sub, gs);
        for (size_t i = 0; i < t.textures.size(); ++i) {
            const Tex1::Texture& tx = t.textures[i];
            Tex1::Image im = t.decode(gs, i);
            size_t ah[5] = {};
            for (u32 c : im.rgba) {
                u32 al = c >> 24;
                ah[al == 0 ? 0 : al < 128 ? 1 : al < 254 ? 2 : al < 255 ? 3 : 4]++;
            }
            u64 t1 = tx.tex1;
            std::printf("  set %zu tex %zu: %ux%u psm %u cpsm %u tcc %d tfx %u clamp %u/%u u[%u..%u] v[%u..%u] "
                        "TEX1 lcm %u mxl %u mmag %u mmin %u l %u k %d | alpha 0:%zu <half:%zu <full:%zu ~full:%zu full:%zu\n",
                        ts, i, tx.tex0.width(), tx.tex0.height(), tx.tex0.psm, tx.tex0.cpsm, tx.tex0.tcc, tx.tex0.tfx,
                        tx.clamp.wms, tx.clamp.wmt, tx.clamp.minu, tx.clamp.maxu, tx.clamp.minv, tx.clamp.maxv,
                        u32(t1 & 1), u32((t1 >> 2) & 7), u32((t1 >> 5) & 1), u32((t1 >> 6) & 7), u32((t1 >> 19) & 3),
                        int(s32(u32(t1 >> 32) << 20) >> 20), ah[0], ah[1], ah[2], ah[3], ah[4]);
        }
    }
    for (const auto& [k, r] : ranges)
        std::printf("  set %u tex %u: %zu verts, rgba min %u %u %u %u max %u %u %u %u, u [%.3f, %.3f] v [%.3f, %.3f], "
                    "y [%.1f, %.1f], r %.1f, raw rgb max 0x%02x\n",
                    k.first, k.second, r.verts, r.cmin[0], r.cmin[1], r.cmin[2], r.cmin[3], r.cmax[0], r.cmax[1],
                    r.cmax[2], r.cmax[3], r.umin, r.umax, r.vmin, r.vmax, r.ymin, r.ymax, r.rmax, r.raw_max);
    return 0;
}

} // namespace

GT3TOOL_COMMAND(course_states, "course-states", "<vol-path> [slot] [tex | cover x z r]",
                "GS render state / texture survey of a course GTM1 slot", 1, cmd_course_states);

} // namespace gt3r::tool
