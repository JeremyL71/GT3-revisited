#pragma once

// "MADS" streamed-music index (bgm/ads.inf) with its groups, as the game
// reads it (docs/formats/sound.md):
//
//   0x00 "MADS", 0x04 u32 1, 0x08 u32 0 (relocation base), 0x0C u32 record count
//   0x10 { u32 record_offset; u32 record_count }[17]   groups
//   records, 20 bytes: u32 name, file, title, artist (string offsets),
//                      u32 info-block offset
//
// The game's BGM ids 0x12..0x22 select group id - 0x12.

#include "base/bytes.h"

#include <string>
#include <vector>

namespace gt3r::sound {

struct MadsTrack {
    std::string name, file, title, artist;
};

struct MadsIndex {
    static constexpr int kGroups = 17;
    std::vector<std::vector<MadsTrack>> groups;

    static MadsIndex parse(ByteSpan ads_inf);
};

} // namespace gt3r::sound
