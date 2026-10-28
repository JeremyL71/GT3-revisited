// GT3's program tables as values (src/sim/exact/program_tables.h), disc-free: each packs to its table's size, curve B is
// the spec curves' second half, PAL's HUD messages differ from NTSC's only in their display times (120 steps at 60 a
// second, 100 at 50), and a hash of every table's bytes is pinned so a change to any value fails. The disc tests
// (test_program_locate.cpp) check the values against each disc's program.

#include "sim/exact/program_data.h"
#include "sim/exact/program_tables.h"
#include "test_framework.h"

using namespace gt3r;
namespace ex = gt3r::sim::exact;
using T = ex::ProgramData::Table;

TEST(program_tables_values) {
    for (int i = 0; i < ex::ProgramData::kTables; ++i) CHECK(ex::gt3_table(T(i)).size() == ex::ProgramData::size(T(i)));
    const ByteSpan curves = ex::gt3_table(T::SpecCurves), b = ex::gt3_table(T::SpecCurveB);
    for (u32 k = 0; k < b.size(); ++k) CHECK(b[k] == curves[0x10 + k]);
    const ByteSpan ntsc = ex::gt3_table(T::Messages), pal = ex::gt3_table(T::Messages, ex::Program::Pal);
    int changed = 0;
    for (u32 m = 0; m < 16; ++m) {
        const u32 n = rd32(ntsc, 8 * m + 4), p = rd32(pal, 8 * m + 4);
        CHECK(rd32(ntsc, 8 * m) == m && rd32(pal, 8 * m) == m);
        CHECK(n == p || (n == 120 && p == 100));
        changed += n != p;
    }
    CHECK(changed > 0);
    for (int i = 0; i < ex::ProgramData::kTables; ++i) // NTSC-J's and PAL's are NTSC-U's but for PAL's messages
        CHECK(ex::gt3_table_differs(T(i), ex::Program::Pal) == (T(i) == T::Messages) && !ex::gt3_table_differs(T(i), ex::Program::NtscJ));
    std::printf("  tables hash %016llX\n", (unsigned long long)ex::gt3_tables_hash());
    CHECK(ex::gt3_tables_hash() == 0x2F3756C147813DA3ull);
}

// ProgramData serves the values: a program it cannot check (here nothing of GT3's) reads GT3's NTSC-U tables.
TEST(program_tables_served) {
    const std::vector<u8> img(0x200000, 0);
    const ex::ProgramData d = ex::ProgramData::load(ByteSpan(img.data(), img.size()), 0x00100000);
    for (int i = 0; i < ex::ProgramData::kTables; ++i) {
        const ByteSpan g = ex::gt3_table(T(i));
        for (u32 k = 0; k < g.size(); ++k) CHECK(d.b(T(i), k) == g[k]);
    }
}
