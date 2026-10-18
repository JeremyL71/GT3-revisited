#include "base/game_random.h"

#include <chrono>
#include <cstring>

namespace gt3r {

u32 GameRandom::next(u32& x) {
    x = x * 17u + 17u;
    return x ^ ((x << 16) | (x >> 16));
}

float GameRandom::rand01(u32& x) {
    u32 bits = (next(x) & 0x7FFFFFu) | 0x3F800000u; // 1.m
    float f;
    std::memcpy(&f, &bits, 4);
    return f - 1.0f; // exact, so no rounding mode matters
}

void GameRandom::stir(u32& x, u32 hardware, GameRandom& shared) {
    int n = (int(hardware) + shared.int31()) % 13;
    skip(x, n > 0 ? n : 1);
}

GameRandom& shared_random() {
    static GameRandom r;
    return r;
}

GameRandom& race_info_random() {
    static GameRandom r;
    return r;
}

namespace {
u32 (*g_hardware_source)() = nullptr;
}

u32 hardware_random() {
    if (g_hardware_source) return g_hardware_source();
    auto t = std::chrono::high_resolution_clock::now().time_since_epoch();
    return u32(u64(std::chrono::duration_cast<std::chrono::nanoseconds>(t).count()) % 48u);
}

void set_hardware_random_source(u32 (*source)()) { g_hardware_source = source; }

} // namespace gt3r
