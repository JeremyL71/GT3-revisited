#pragma once

// "CRSS" course ambience emitters (course file slot 35). See
// docs/formats/sound.md "Course ambience (CRSS)".
//
//   0x00 "CRSS", 0x04 u32 0, 0x08 u32 0 (relocation)
//   0x0C u32 size of the used data, 0x10 u32 1
//   0x14 u32 list_offset[5]   one list per ambience channel
//   list:    u32 count; u32 emitter_offset[count]
//   emitter: 0x00 u32 point count
//            0x04..0x0F 0 (0x0C holds the emitter's gain at run time)
//            0x10 vec4 gate A, 0x20 vec4 gate B (x, y, z; w = 0 in the
//                 file, the gate's track distance at run time)
//            0x30 point[count]: x, y, z, weight (floats)
// All offsets are relative to the section start; positions are course
// coordinates. An emitter whose gates coincide (all zero in practice) is
// always active; otherwise it sounds while a car is on the stretch of
// track between the gates.

#include "base/bytes.h"

#include <array>
#include <vector>

namespace gt3r::sound {

struct CrssEmitter {
    struct Point {
        float x = 0, y = 0, z = 0, weight = 0;
    };
    std::array<float, 3> gate_a{}, gate_b{};
    std::vector<Point> points;
    bool gated() const { return gate_a != gate_b; }
};

struct CrssData {
    static constexpr int kChannels = 5;
    std::array<std::vector<CrssEmitter>, kChannels> channels;

    static CrssData parse(ByteSpan section);
};

} // namespace gt3r::sound
