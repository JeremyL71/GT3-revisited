#include "formats/menu/imgs.h"

#include <algorithm>

namespace gt3r::menu {

bool Imgs::is_imgs(ByteSpan data) { return has_magic(data, "IMGS") && data.size() >= 0x10; }

Imgs Imgs::parse(ByteSpan data) {
    if (!is_imgs(data)) throw FormatError("not an IMGS image set");
    u32 count = rd32(data, 0x08);
    size_t first = rd32(data, 0x0C);
    if (count == 0 || count > 0x10000 || first > data.size() || first < 0x10 || (first - 0x10) % count)
        throw FormatError("IMGS header out of range");
    // Entries are 0x20 bytes in most files and 0x40 in some (longer names).
    size_t stride = (first - 0x10) / count;
    if (stride < 8 || stride % 4) throw FormatError("IMGS: bad entry size");
    Imgs out;
    // Walk the Tex1 sets first: they are stored back to back, 16-byte aligned.
    std::vector<std::pair<size_t, size_t>> sets;
    size_t o = first;
    while (sets.size() < count) {
        if (o + 0x20 > data.size() || !Tex1::is_tex1(data.subspan(o))) throw FormatError("IMGS: missing Tex1 set");
        size_t sz = rd32(data, o + 0x0C);
        if (sz < 0x20 || sz > data.size() - o) throw FormatError("IMGS: bad Tex1 size");
        sets.emplace_back(o, sz);
        o = (o + sz + 15) & ~size_t(15);
    }
    for (u32 i = 0; i < count; ++i) {
        size_t e = 0x10 + size_t(i) * stride;
        Entry en;
        const char* s = reinterpret_cast<const char*>(&data[e]);
        en.name.assign(s, strnlen(s, stride - 4));
        en.index = rd32(data, e + stride - 4);
        if (en.index >= count) throw FormatError("IMGS: entry index out of range");
        en.offset = sets[en.index].first;
        en.size = sets[en.index].second;
        out.entries.push_back(std::move(en));
    }
    return out;
}

int Imgs::find(std::string_view name) const {
    for (size_t i = 0; i < entries.size(); ++i)
        if (entries[i].name == name) return int(i);
    return -1;
}

Tex1::Image Imgs::decode(ByteSpan data, size_t i) const {
    const Entry& e = entries.at(i);
    ByteSpan sub = data.subspan(e.offset, e.size);
    Tex1 set = Tex1::parse(sub);
    if (set.textures.empty()) return {};
    GsMemory gs;
    set.upload(sub, gs);
    return set.decode(gs, 0);
}

} // namespace gt3r::menu
