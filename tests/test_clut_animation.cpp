// Tex1 CLUT animations: parsing, the player's timing and colour mixing, recoloured textures.
// Disc-free (synthetic data).

#include "test_framework.h"

#include "formats/clut_animation.h"
#include "formats/tex1.h"

#include <cstring>

using namespace gt3r;

namespace {

Tex1::ClutAnimation anim(u16 mode, std::vector<std::pair<float, u32>> frames, u16 group = 0) {
    Tex1::ClutAnimation a;
    a.group = group, a.mode = mode, a.words = 1;
    for (auto& [s, c] : frames) {
        a.frames.push_back({s, 0});
        a.colours.push_back(c);
    }
    return a;
}

u32 colour(const ClutPlayer& p, size_t i = 0) {
    u32 c = 0;
    p.colours(i, &c);
    return c;
}

// The game's mix of two colours: weight w (x 4096, truncated) of `a`, the rest of `b`.
u32 mix(u32 a, u32 b, float weight) {
    s32 w = s32(weight * 4096.0f);
    u32 v = 0;
    for (int s = 0; s < 32; s += 8)
        v |= ((u32(s32((a >> s) & 0xFF) * w + s32((b >> s) & 0xFF) * (4096 - w)) >> 12) & 0xFF) << s;
    return v;
}

constexpr u32 kA = 0x80000010u, kB = 0x80202020u, kC = 0x00303030u;

void put32(Bytes& d, size_t o, u32 v) { std::memcpy(&d[o], &v, 4); }
void put16(Bytes& d, size_t o, u16 v) { std::memcpy(&d[o], &v, 2); }
void put64(Bytes& d, size_t o, u64 v) { std::memcpy(&d[o], &v, 8); }

// A Tex1 set: one 4x4 T4 texture (texel index (x + y) & 3) at block 0, its 16-colour CT32 CSM1
// CLUT (an 8x2 upload) at block 1, and one animation of CLUT entries 1 and 2 over three frames.
Bytes synthetic_set() {
    Bytes d(0x200, 0);
    std::memcpy(&d[0], "Tex1", 4);
    put32(d, 0x0C, u32(d.size()));
    put16(d, 0x14, 1), put16(d, 0x16, 2);
    put32(d, 0x18, 0x30), put32(d, 0x1C, 0x60), put32(d, 0x24, 0x80);
    // TEX0: tbp 0, tbw 1, T4, 4x4, TCC, CBP 1, CT32, CSM1, CSA 0, CLD 1.
    u64 tex0 = 0 | (1ull << 14) | (0x14ull << 20) | (2ull << 26) | (2ull << 30) | (1ull << 34) | (1ull << 37) |
               (1ull << 61);
    put64(d, 0x30, tex0);
    // Buffers: texels (T4, 4x4, at 0xC0), CLUT (CT32 8x2 at 0x100).
    put32(d, 0x60, 0xC0), put16(d, 0x64, 0), d[0x66] = 1, d[0x67] = 0x14, put16(d, 0x68, 4), put16(d, 0x6A, 4);
    put32(d, 0x6C, 0x100), put16(d, 0x70, 1), d[0x72] = 1, d[0x73] = 0, put16(d, 0x74, 8), put16(d, 0x76, 2);
    for (u32 y = 0; y < 4; ++y)
        for (u32 x = 0; x < 4; x += 2) d[0xC0 + y * 2 + x / 2] = u8(((x + y) & 3) | (((x + 1 + y) & 3) << 4));
    for (u32 e = 0; e < 16; ++e) put32(d, 0x100 + 4 * e, 0x80000000u | (e * 0x10101u)); // stored colours
    // Animation table: 1 entry at 0xA0; frames at 0x140; their colours from 0x160.
    put32(d, 0x80, 1), put32(d, 0x84, 0xA0);
    put16(d, 0xA0, 0), put16(d, 0xA2, 0), put16(d, 0xA4, 2), put16(d, 0xA6, 3);
    put32(d, 0xA8, 0x104), put32(d, 0xAC, 0x140);
    const float secs[3] = {0.5f, 1.0f, 0.25f};
    const u32 cols[3][2] = {{0x00102030u, 0x00405060u}, {0x80A0B0C0u, 0x80D0E0F0u}, {0x40112233u, 0x40445566u}};
    for (u32 f = 0; f < 3; ++f) {
        u32 bits;
        std::memcpy(&bits, &secs[f], 4);
        put32(d, 0x140 + 8 * f, bits), put32(d, 0x144 + 8 * f, 0x160 + 8 * f);
        put32(d, 0x160 + 8 * f, cols[f][0]), put32(d, 0x164 + 8 * f, cols[f][1]);
    }
    return d;
}

} // namespace

TEST(clut_player_loop_crossfade) {
    ClutPlayer p({anim(0, {{0.5f, kA}, {1.0f, kB}, {0.25f, kC}})});
    CHECK(colour(p) == kA); // reset: frame 0, weight 1
    p.update(0.25f);
    CHECK(colour(p) == mix(kA, kB, 0.5f)); // half way: fading to frame 1
    p.update(0.25f);
    CHECK(colour(p) == kB); // frame 1
    p.update(1.35f);         // past frames 1 and 2: 0.1 s into frame 0 again
    CHECK(colour(p) == mix(kA, kB, 1.0f - 0.1f / 0.5f));
}

TEST(clut_player_hold_release_once) {
    ClutPlayer p({anim(2, {{0.5f, kA}, {-0.25f, kB}, {0.5f, kC}})});
    p.update(1.0f);
    CHECK(colour(p) == kB); // held on the hold frame (time 0)
    p.update(5.0f);
    CHECK(colour(p) == kB);
    p.release(-1);
    p.update(0.1f);
    CHECK(colour(p) == mix(kB, kC, 1.0f - 0.1f / 0.25f));
    p.update(0.2f);
    CHECK(colour(p) == kC); // mode 2: stays on the last frame
    p.update(3.0f);
    CHECK(colour(p) == kC);
    // A hold on the first frame holds from the start.
    ClutPlayer q({anim(0, {{-1.0f, kA}, {1.0f, kB}})});
    q.update(2.0f);
    CHECK(colour(q) == kA);
}

TEST(clut_player_ping_pong) {
    ClutPlayer p({anim(1, {{1.0f, kA}, {1.0f, kB}, {1.0f, kC}})});
    p.update(2.0f); // frame 2, fading back to frame 1
    CHECK(colour(p) == kC);
    p.update(0.5f);
    CHECK(colour(p) == mix(kC, kB, 0.5f));
    p.update(1.5f); // frame 0 again after frame 1
    CHECK(colour(p) == kA);
}

TEST(clut_player_start_signal) {
    // A start light as Route 11's: off, on for 4.9 s, a hold frame at its end, off again.
    const u32 off = 0x00000000u, on = 0x80FFFFFFu, last = 0x80FF0000u;
    std::vector<std::pair<float, u32>> f = {{0.95f, off}, {0.05f, off}, {0.05f, on}, {4.9f, on},
                                            {0.05f, on},  {-0.2f, last}, {0.3f, off}, {0.5f, off}};
    ClutPlayer p({anim(2, f, 1), anim(0, {{1.0f, kA}, {1.0f, kB}})});
    p.update(20.0f); // before the start sequence: it runs to its hold and stays
    CHECK(colour(p, 0) == last);
    p.reset();
    p.start_signal(4.0f); // a 4 s countdown: 0.95 s into the lit frame, released past the hold
    CHECK(colour(p, 0) == mix(on, on, 1.0f - 0.95f / 4.9f));
    CHECK(colour(p, 1) == kA); // other groups untouched
    p.update(3.9f);
    CHECK(colour(p, 0) == on);
    p.update(0.15f); // through the hold frame
    CHECK(colour(p, 0) != on);
    p.update(1.0f);
    CHECK(colour(p, 0) == off); // the last frame
    // No countdown (rolling starts): straight past the hold.
    p.reset();
    p.start_signal(0.0f);
    CHECK(colour(p, 0) == last);
    p.update(0.25f);
    p.update(1.0f);
    CHECK(colour(p, 0) == off);
}

TEST(clut_textures_synthetic) {
    Bytes d = synthetic_set();
    Tex1 t = Tex1::parse(d);
    CHECK(t.clut_animations.size() == 1);
    if (t.clut_animations.size() != 1) return;
    const Tex1::ClutAnimation& a = t.clut_animations[0];
    CHECK(a.group == 0 && a.mode == 0 && a.words == 2 && a.target == 0x104 && a.frames.size() == 3);
    CHECK(a.frames[1].seconds == 1.0f && a.frames[1].source == 0x168 && a.colours[3] == 0x80D0E0F0u);

    ClutTextures ct(d);
    CHECK(ct.records() == std::vector<u32>{0});
    const std::vector<u32>* px = ct.rgba(0);
    CHECK(px && px->size() == 16);
    if (!px || px->size() != 16) return;
    // Index 0: the stored colour; 1 and 2: frame 0's colours (alpha 0); 3: stored.
    CHECK((*px)[0] == clut_rgba(0x80000000u, 0));
    CHECK((*px)[1] == clut_rgba(0x00102030u, 0)); // texel (1, 0)
    CHECK((*px)[2] == clut_rgba(0x00405060u, 0));
    CHECK((*px)[3] == clut_rgba(0x80030303u, 0));
    CHECK(ct.min_alpha(0) == 0);
    CHECK(ct.refresh().empty()); // nothing changed
    ct.player().update(0.5f);
    CHECK(ct.refresh() == std::vector<u32>{0});
    CHECK((*ct.rgba(0))[1] == clut_rgba(0x80A0B0C0u, 0));
    CHECK((*ct.rgba(0))[0] == clut_rgba(0x80000000u, 0));
}

TEST(clut_animation_bad_table_ignored) {
    Bytes d = synthetic_set();
    put32(d, 0xAC, 0x1F8); // frame table runs past the set
    CHECK(Tex1::parse(d).clut_animations.empty());
    put32(d, 0xAC, 0x140);
    put32(d, 0x24, 0);
    CHECK(Tex1::parse(d).clut_animations.empty());
}
