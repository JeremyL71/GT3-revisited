#pragma once

// GT3's program tables the exact core reads, written out as values (program_tables.cpp: the static physics block, the tyre
// effect block, the controller settings, the AI driver sets and line lists, the car spec's defaults, the HUD messages, the
// machine test's sectors, the stick curve, the tyre effect kinds, scales and parameters). They are the same in GT3's three
// releases but for the HUD messages, whose display times PAL counts in its 50 steps a second. ProgramData serves these
// bytes and checks the disc's own tables against them.

#include "sim/exact/program_data.h"

namespace gt3r::sim::exact {

// Table t's bytes as release p's program holds them.
ByteSpan gt3_table(ProgramData::Table t, Program p = Program::NtscU);
// The releases whose table t differs from NTSC-U's (PAL's HUD messages).
bool gt3_table_differs(ProgramData::Table t, Program p);
// A hash of every table's bytes, each release's variants included (the disc-free test pins it).
u64 gt3_tables_hash();

} // namespace gt3r::sim::exact
