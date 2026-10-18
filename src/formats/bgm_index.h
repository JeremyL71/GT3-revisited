#pragma once

#include "base/bytes.h"

#include <string>
#include <vector>

namespace gt3r {

// bgm/ads.inf ("MADS"): the music track index.
//
// The full structure (playlists/categories) is not yet decoded. Track
// records are 5 u32s: name, file name, title, artist (all offsets of
// NUL-terminated strings within the file) and an offset into a second
// table. They are located by scanning for records whose file name string
// ends in ".ads", which is robust across regions.
struct BgmTrack {
    std::string name;   // e.g. "race_a01"
    std::string file;   // e.g. "race_a01.ads" (relative to bgm/)
    std::string title;
    std::string artist;
};

std::vector<BgmTrack> parse_bgm_index(ByteSpan ads_inf);

} // namespace gt3r
