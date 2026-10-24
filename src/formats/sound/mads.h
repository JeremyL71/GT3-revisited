#pragma once

// "MADS" streamed-music index (bgm/ads.inf) with its groups, as the game
// reads it (docs/formats/sound.md):
//
//   0x00 "MADS", 0x04 u32 1, 0x08 u32 0 (relocation base), 0x0C u32 record count
//   0x10 { u32 record_offset; u32 record_count }[17]   groups
//   records, 20 bytes: u32 name, file, title, artist (string offsets),
//                      u32 info-block offset
//   info block: +0x00 u32 relocation base (0 in the file), +0x04 u32 sample rate,
//               +0x08 u32 length in samples (per channel), +0x0C u32 count[32],
//               +0x8C u32 offset[32] (of each track's marker array, from the block),
//               the arrays: u32 sample positions, ascending
//
// The game's BGM ids 0x12..0x22 select group id - 0x12. The info block's marker
// tracks time the replay cameras' cuts (music §8): on the disc only the race songs and
// demo_a01 have any (tracks 1 beats, 2 phrases, 3 and 4 the cut points).

#include "base/bytes.h"

#include <array>
#include <string>
#include <vector>

namespace gt3r::sound {

struct MadsMarkers {
    static constexpr int kTracks = 32;
    u32 rate = 0;   // samples per second (44100)
    u32 length = 0; // samples per channel
    std::array<std::vector<u32>, kTracks> tracks; // sample positions, ascending

    bool any() const {
        for (const auto& t : tracks)
            if (!t.empty()) return true;
        return false;
    }
    // Track k's marker i in seconds.
    double seconds(int k, size_t i) const { return rate ? double(tracks[size_t(k)][i]) / rate : 0.0; }
};

struct MadsTrack {
    std::string name, file, title, artist;
    MadsMarkers markers; // empty when the record has no info block
};

struct MadsIndex {
    static constexpr int kGroups = 17;
    std::vector<std::vector<MadsTrack>> groups;

    static MadsIndex parse(ByteSpan ads_inf);
    // A record's info block at `offset` (0: none, all tracks empty). Throws FormatError.
    static MadsMarkers parse_markers(ByteSpan ads_inf, u32 offset);
};

} // namespace gt3r::sound
