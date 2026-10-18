#include "formats/bgm_index.h"

#include <set>

namespace gt3r {

namespace {

// Returns the string at `off` if it is a plausible NUL-terminated string.
bool string_at(ByteSpan d, u32 off, std::string& out) {
    if (off >= d.size()) return false;
    out.clear();
    for (size_t i = off; i < d.size() && out.size() < 256; ++i) {
        if (d[i] == 0) return true;
        out.push_back(static_cast<char>(d[i]));
    }
    return false;
}

bool ends_with(const std::string& s, const char* suffix) {
    size_t n = std::strlen(suffix);
    return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

} // namespace

std::vector<BgmTrack> parse_bgm_index(ByteSpan d) {
    if (!has_magic(d, "MADS")) throw FormatError("not a MADS index");
    std::vector<BgmTrack> tracks;
    std::set<std::string> seen;
    for (size_t o = 8; o + 20 <= d.size(); o += 4) {
        BgmTrack t;
        if (!string_at(d, rd32(d, o + 4), t.file) || !ends_with(t.file, ".ads")) continue;
        if (!string_at(d, rd32(d, o), t.name) || t.file.compare(0, t.name.size(), t.name) != 0) continue;
        string_at(d, rd32(d, o + 8), t.title);
        string_at(d, rd32(d, o + 12), t.artist);
        if (seen.insert(t.file).second) tracks.push_back(std::move(t));
        o += 16;
    }
    return tracks;
}

} // namespace gt3r
