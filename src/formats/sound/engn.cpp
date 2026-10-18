#include "formats/sound/engn.h"

namespace gt3r::sound {

EngineBank EngineBank::parse(ByteSpan d) {
    if (!has_magic(d, "ENGN") || d.size() < 0x30) throw FormatError("not an ENGN bank");
    u32 data_size = rd32(d, 0x10);
    u32 data_off = rd32(d, 0x14);
    u32 count = rd32(d, 0x18);
    u32 table = rd32(d, 0x1C);
    if (u64(data_off) + data_size > d.size()) throw FormatError("ENGN: data out of range");
    if (count == 0 || count > 64 || u64(table) + 16ull * count > d.size()) throw FormatError("ENGN: bad layer table");

    EngineBank b;
    b.unknown_28 = rd32(d, 0x28);
    b.unknown_2c = rd32(d, 0x2C);
    b.adpcm = std::make_shared<const Bytes>(d.begin() + data_off, d.begin() + data_off + data_size);
    for (u32 i = 0; i < count; ++i) {
        size_t o = table + 16 * i;
        EngineLayer l;
        l.base_rpm = rd16(d, o);
        l.lo_rpm = s16(rd16(d, o + 2));
        l.hi_rpm = s16(rd16(d, o + 4));
        l.volume = s16(rd16(d, o + 6));
        l.pitch = rd32(d, o + 8);
        l.offset = rd32(d, o + 12);
        if (l.offset >= data_size || l.offset % 16) throw FormatError("ENGN: bad layer offset");
        b.layers.push_back(l);
    }
    for (size_t i = 0; i < b.layers.size(); ++i) {
        u32 end = i + 1 < b.layers.size() ? b.layers[i + 1].offset : data_size;
        if (end <= b.layers[i].offset) throw FormatError("ENGN: layers out of order");
        b.layers[i].size = end - b.layers[i].offset;
    }
    return b;
}

} // namespace gt3r::sound
