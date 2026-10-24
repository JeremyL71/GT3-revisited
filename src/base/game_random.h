#pragma once

// GT3's random number generator. One algorithm,
// several independent states:
//
//   step:  x = 17 x + 17 (mod 2^32); output x ^ rotl(x, 16)
//   unit:  the low 23 output bits as the float 1.m, minus 1: [0, 1) in steps of 2^-23
//
// The states are the game's own: the shared library state (`shared_random`),
// the race module's (`race_info_random`: opponent pool draws, paint colours),
// the GT-mode state saved with the game (GameState::rng), plus per-object
// ones (cameras, the replay director) that start at kBootSeed.

#include "base/bytes.h"

namespace gt3r {

struct GameRandom {
    static constexpr u32 kBootSeed = 0x87654321u;
    u32 x = kBootSeed;

    // On an external state word.
    static u32 next(u32& x);
    static float rand01(u32& x);
    static void skip(u32& x, int n) {
        for (int i = 0; i < n; ++i) next(x);
    }
    // The GT-mode "stir" (gt_mode §20.2): (hardware + shared.int31()) mod 13 steps, at least 1.
    static void stir(u32& x, u32 hardware, GameRandom& shared);

    u32 next() { return next(x); }
    float rand01() { return rand01(x); }
    void skip(int n) { skip(x, n); }
    // The shared state's integer draw: int(unit x 2^31), 0 .. 2^31 - 256 (exact).
    int int31() { return int(rand01() * 2147483648.0f); }
};

// The library's global state (seeded kBootSeed at start, never reseeded).
GameRandom& shared_random();
// The race module's state: every opponent pool draw and paint-colour
// assignment of the session consumes it (seeded kBootSeed at start).
GameRandom& race_info_random();

// The PS2 vector unit's random register R, a second generator the race draws from for car
// effects (the flap pendulums' kicks):
//
//   init:  R = 1.m with the seed's mantissa
//   next:  R = 1.m with m = (m << 1 | (bit 4 ^ bit 22)), 23 bits; output 1.m - 1, [0, 1)
//
// One register (`vu0_random`), seeded with 0.1 at every run start.
struct Vu0Random {
    static constexpr float kRunSeed = 0.1f;
    u32 r = 0x3F800000u;

    void init(float seed);
    float next();
};
Vu0Random& vu0_random();

// Where the game wants unpredictable input it uses a hardware value in 0..47
// that changes every cycle. Ours derives the same range
// from the high-resolution clock; tests can install a fixed source.
u32 hardware_random();
void set_hardware_random_source(u32 (*source)()); // nullptr = the clock

} // namespace gt3r
