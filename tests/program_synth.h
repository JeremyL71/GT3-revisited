#pragma once

// A synthetic game program for disc-free tests of the exact core's builders: random tables (a fixed seed) in
// ProgramData::synthetic, the static block's rolling-resistance curve with its 8 points. INVENTED: nothing comes from a disc.

#include "sim/exact/program_data.h"

#include <random>
#include <vector>

namespace gt3r::test {

inline sim::exact::ProgramData synthetic_program() {
    namespace ex = sim::exact;
    using T = ex::ProgramData::Table;
    ex::ProgramData d = ex::ProgramData::synthetic();
    std::mt19937 rng(5);
    for (u32 i = 0; i < ex::ProgramData::size(T::Static); ++i) d.poke(T::Static, i, u8(rng()));
    d.poke(T::Static, 0x0B, 8);
    for (u32 i = 0; i < ex::ProgramData::size(T::LsdTypes); ++i) d.poke(T::LsdTypes, i, u8(rng() % 8));
    return d;
}

} // namespace gt3r::test
