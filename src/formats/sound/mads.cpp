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

MadsIndex MadsIndex::parse(ByteSpan d) {
    if (!has_magic(d, "MADS") || d.size() < 0x10 + 8 * kGroups) throw FormatError("not a MADS index");
    MadsIndex m;
    for (int g = 0; g < kGroups; ++g) {
        u32 rec = rd32(d, 0x10 + 8 * g), n = rd32(d, 0x14 + 8 * g);
        if (n > 256 || u64(rec) + 20ull * n > d.size()) throw FormatError("MADS: bad group");
        std::vector<MadsTrack> list;
        for (u32 i = 0; i < n; ++i) {
            size_t o = rec + 20 * i;
            list.push_back({cstr(d, rd32(d, o)), cstr(d, rd32(d, o + 4)), cstr(d, rd32(d, o + 8)), cstr(d, rd32(d, o + 12))});
        }
        m.groups.push_back(std::move(list));
    }
    return m;
}

} // namespace gt3r::sound
