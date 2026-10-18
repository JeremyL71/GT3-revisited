#include "formats/sound/crss.h"

#include <cstring>

namespace gt3r::sound {

namespace {

float rdf(ByteSpan d, size_t o) {
    u32 v = rd32(d, o);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}

} // namespace

CrssData CrssData::parse(ByteSpan d) {
    if (!has_magic(d, "CRSS") || d.size() < 0x28) throw FormatError("not a CRSS section");
    CrssData c;
    for (int ch = 0; ch < kChannels; ++ch) {
        u32 lo = rd32(d, 0x14 + 4 * ch);
        if (u64(lo) + 4 > d.size()) throw FormatError("CRSS: list out of range");
        u32 n = rd32(d, lo);
        if (n > 256 || u64(lo) + 4 + 4ull * n > d.size()) throw FormatError("CRSS: bad list count");
        for (u32 k = 0; k < n; ++k) {
            u32 eo = rd32(d, lo + 4 + 4 * k);
            if (u64(eo) + 0x30 > d.size()) throw FormatError("CRSS: emitter out of range");
            u32 np = rd32(d, eo);
            if (np > 1024 || u64(eo) + 0x30 + 16ull * np > d.size()) throw FormatError("CRSS: bad point count");
            CrssEmitter e;
            for (int i = 0; i < 3; ++i) {
                e.gate_a[size_t(i)] = rdf(d, eo + 0x10 + 4 * i);
                e.gate_b[size_t(i)] = rdf(d, eo + 0x20 + 4 * i);
            }
            for (u32 p = 0; p < np; ++p) {
                size_t o = eo + 0x30 + 16 * p;
                e.points.push_back({rdf(d, o), rdf(d, o + 4), rdf(d, o + 8), rdf(d, o + 12)});
            }
            c.channels[size_t(ch)].push_back(std::move(e));
        }
    }
    return c;
}

} // namespace gt3r::sound
