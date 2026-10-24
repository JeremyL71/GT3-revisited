#include "formats/sound/mads.h"

namespace gt3r::sound {

namespace {

std::string cstr(ByteSpan d, u32 off) {
    if (off >= d.size()) throw FormatError("MADS: string offset out of range");
    std::string s;
    for (size_t p = off; p < d.size() && d[p]; ++p) s.push_back(char(d[p]));
    return s;
}

} // namespace

MadsMarkers MadsIndex::parse_markers(ByteSpan d, u32 info) {
    MadsMarkers m;
    if (info == 0) return m;
    if (u64(info) + 0x10C > d.size()) throw FormatError("MADS: info block out of range");
    m.rate = rd32(d, info + 4);
    m.length = rd32(d, info + 8);
    for (int k = 0; k < MadsMarkers::kTracks; ++k) {
        const u32 n = rd32(d, info + 0x0C + 4 * size_t(k)), off = rd32(d, info + 0x8C + 4 * size_t(k));
        if (n == 0) continue;
        if (n > 0x10000 || u64(info) + off + 4ull * n > d.size()) throw FormatError("MADS: marker track out of range");
        std::vector<u32>& t = m.tracks[size_t(k)];
        t.reserve(n);
        for (u32 i = 0; i < n; ++i) t.push_back(rd32(d, info + off + 4 * size_t(i)));
    }
    return m;
}

MadsIndex MadsIndex::parse(ByteSpan d) {
    if (!has_magic(d, "MADS") || d.size() < 0x10 + 8 * kGroups) throw FormatError("not a MADS index");
    MadsIndex m;
    for (int g = 0; g < kGroups; ++g) {
        u32 rec = rd32(d, 0x10 + 8 * g), n = rd32(d, 0x14 + 8 * g);
        if (n > 256 || u64(rec) + 20ull * n > d.size()) throw FormatError("MADS: bad group");
        std::vector<MadsTrack> list;
        for (u32 i = 0; i < n; ++i) {
            size_t o = rec + 20 * i;
            list.push_back({cstr(d, rd32(d, o)), cstr(d, rd32(d, o + 4)), cstr(d, rd32(d, o + 8)), cstr(d, rd32(d, o + 12)),
                            parse_markers(d, rd32(d, o + 16))});
        }
        m.groups.push_back(std::move(list));
    }
    return m;
}

} // namespace gt3r::sound
