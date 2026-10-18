#include "formats/menu/mbl.h"

namespace gt3r::menu {

namespace {

constexpr u32 kMaxCount = 100000;

struct Layout {
    u32 page_header;
    u32 item_size;
};

class Reader {
public:
    Reader(ByteSpan d, Layout l) : d_(d), l_(l) {}

    u32 ptr(size_t o) const {
        u32 p = rd32(d_, o);
        if (p >= d_.size()) throw FormatError("mbl: pointer out of range");
        return p;
    }
    std::string str(size_t o, size_t max) const {
        if (o + max > d_.size()) throw FormatError("mbl: string out of range");
        const char* s = reinterpret_cast<const char*>(&d_[o]);
        return std::string(s, strnlen(s, max));
    }
    template <typename Fn>
    void each(size_t count_field, Fn fn) {
        u32 count = rd32(d_, count_field);
        if (count == 0) return;
        if (count > kMaxCount) throw FormatError("mbl: bad count");
        u32 table = ptr(count_field + 4);
        for (u32 i = 0; i < count; ++i) fn(ptr(table + size_t(i) * 4));
    }

    MblItem item(size_t o, int depth) {
        if (depth > 8) throw FormatError("mbl: nesting too deep");
        MblItem it;
        it.name = str(o, 64);
        size_t tail;
        if (l_.item_size == 0x14C) {
            it.str40 = str(o + 0x40, 64);
            it.str80 = str(o + 0x80, 64);
            it.action = str(o + 0xC0, 64);
            it.link = str(o + 0x100, 64);
            tail = o + 0x140;
        } else {
            it.str40 = str(o + 0x40, 32);
            it.link = str(o + 0x60, 64);
            tail = o + 0xA0;
        }
        it.unk140 = s16(rd16(d_, tail));
        it.unk142 = rd16(d_, tail + 2);
        each(tail + 4, [&](u32 p) { it.children.push_back(item(p, depth + 1)); });
        return it;
    }

    MblPage page(size_t o) {
        MblPage pg;
        pg.name = str(o, 64);
        pg.unk40 = rd16(d_, o + 0x40);
        pg.flags42 = rd16(d_, o + 0x42);
        u32 extra = l_.page_header - 0x44;
        if (extra == 0xC0) {
            pg.str44 = str(o + 0x44, 64);
            pg.back = str(o + 0x84, 64);
            pg.back_action = str(o + 0xC4, 64);
        } else {
            pg.str44 = str(o + 0x44, extra);
        }
        each(o + l_.page_header, [&](u32 p) { pg.items.push_back(item(p, 0)); });
        return pg;
    }

private:
    ByteSpan d_;
    Layout l_;
};

} // namespace

bool MblFile::is_mbl(ByteSpan data) {
    return data.size() >= 12 && rd32(data, 4) == 8 && rd32(data, 0) < kMaxCount && rd32(data, 0) > 0 &&
           rd32(data, data.size() - 4) == data.size();
}

MblFile MblFile::parse(ByteSpan data) {
    if (!is_mbl(data)) throw FormatError("not an mbl menu flow file");
    u32 npages = rd32(data, 0);
    u32 first = rd32(data, 8);
    // The first page's item table pointer immediately follows its header,
    // which identifies the layout.
    MblFile out;
    for (u32 h : {0x104u, 0x84u, 0x64u}) {
        if (first + h + 8 <= data.size() && rd32(data, first + h + 4) == first + h + 8) {
            out.page_header = h;
            break;
        }
    }
    if (!out.page_header) throw FormatError("mbl: unknown page layout");
    out.legacy = out.page_header != 0x104;
    Reader r(data, {out.page_header, out.legacy ? 0xACu : 0x14Cu});
    for (u32 i = 0; i < npages; ++i) out.pages.push_back(r.page(r.ptr(8 + size_t(i) * 4)));
    return out;
}

int MblFile::find_page(std::string_view name) const {
    for (size_t i = 0; i < pages.size(); ++i)
        if (pages[i].name == name) return int(i);
    return -1;
}

} // namespace gt3r::menu
