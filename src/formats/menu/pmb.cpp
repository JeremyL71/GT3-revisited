#include "formats/menu/pmb.h"

#include "base/gzip.h"

#include <charconv>

namespace gt3r::menu {

namespace {

constexpr u32 kMaxCount = 100000;

class Reader {
public:
    Reader(ByteSpan d, u32 name_size) : d_(d), n_(name_size) {}

    u32 ptr(size_t o) const {
        u32 p = rd32(d_, o);
        if (p >= d_.size()) throw FormatError("pmb: pointer out of range");
        return p;
    }
    std::string str(size_t o, size_t max) const {
        if (o >= d_.size()) throw FormatError("pmb: string out of range");
        max = std::min(max, d_.size() - o);
        const char* s = reinterpret_cast<const char*>(&d_[o]);
        return std::string(s, strnlen(s, max));
    }
    Rgba rgba(size_t o) const {
        if (o + 4 > d_.size()) throw FormatError("pmb: color out of range");
        return {d_[o], d_[o + 1], d_[o + 2], d_[o + 3]};
    }

    // Reads `count` pointers at `table`, calling fn(target) for each.
    template <typename Fn>
    void each(u32 count, size_t table_field, Fn fn) {
        if (count == 0) return;
        if (count > kMaxCount) throw FormatError("pmb: bad count");
        u32 table = ptr(table_field);
        for (u32 i = 0; i < count; ++i) fn(ptr(table + size_t(i) * 4));
    }

    Element element(size_t o, int depth) {
        if (depth > 8) throw FormatError("pmb: element nesting too deep");
        Element e;
        e.offset = o;
        e.name = str(o, n_);
        size_t b = o + n_;
        e.type = rd16(d_, b);
        e.flags = rd16(d_, b + 2);
        float f[5];
        for (int i = 0; i < 5; ++i) {
            u32 v = rd32(d_, b + 4 + i * 4);
            std::memcpy(&f[i], &v, 4);
        }
        e.x = f[0], e.y = f[1], e.z = f[2], e.w = f[3], e.h = f[4];
        size_t slots = b + 0x18;
        if (rd32(d_, slots) != 0) e.widget_class = str(ptr(ptr(slots)), 256);
        if (rd32(d_, slots + 4) != 0) e.alt.push_back(element(ptr(slots + 4), depth + 1));
        if (rd32(d_, slots + 8) != 0) {
            u32 p = ptr(slots + 8);
            ImageRef r;
            r.image = rd32(d_, p);
            for (int i = 0; i < 8; ++i) {
                u32 v = rd32(d_, p + 4 + i * 4);
                std::memcpy(&r.uv[i], &v, 4);
            }
            e.image = r;
        }
        if (rd32(d_, slots + 12) != 0) {
            u32 p = ptr(slots + 12);
            std::array<Rgba, 4> c;
            for (int i = 0; i < 4; ++i) c[i] = rgba(p + i * 4);
            e.colors = c;
        }
        if (rd32(d_, slots + 16) != 0) {
            u32 p = ptr(slots + 16);
            TextStyle t;
            t.unk0 = rd16(d_, p);
            t.align = rd16(d_, p + 2);
            t.color = rgba(p + 4);
            if (rd32(d_, p + 8)) t.text = str(ptr(p + 8), 4096);
            if (rd32(d_, p + 12)) t.font = str(ptr(p + 12), 256);
            e.text = std::move(t);
        }
        return e;
    }

    Layer layer(size_t o) {
        Layer l;
        l.name = str(o, n_);
        size_t b = o + n_;
        each(rd32(d_, b), b + 4, [&](u32 p) { l.elements.push_back(element(p, 0)); });
        each(rd32(d_, b + 8), b + 12, [&](u32 p) { l.widgets.push_back(element(p, 0)); });
        return l;
    }

    Page page(size_t o) {
        Page pg;
        pg.name = str(o, n_);
        size_t b = o + n_;
        each(rd32(d_, b), b + 4, [&](u32 p) { pg.items.push_back(element(p, 0)); });
        each(rd32(d_, b + 8), b + 12, [&](u32 p) { pg.layers.push_back(layer(p)); });
        return pg;
    }

private:
    ByteSpan d_;
    u32 n_;
};

bool names_plausible(const PmbFile& f) {
    auto ok = [](const std::string& s) {
        for (unsigned char c : s)
            if (c < 0x20) return false;
        return true;
    };
    for (const Page& p : f.pages) {
        if (!ok(p.name)) return false;
        for (const Layer& l : p.layers)
            if (!ok(l.name)) return false;
    }
    return true;
}

} // namespace

bool PmbFile::is_pmb(ByteSpan data) {
    return data.size() >= 0x10 && rd32(data, 4) == 0x10 && rd32(data, 0) < kMaxCount && rd32(data, 0x0C) <= data.size();
}

PmbFile PmbFile::parse(ByteSpan data) {
    if (!is_pmb(data)) throw FormatError("not a pmb menu layout");
    u32 npages = rd32(data, 0), table = rd32(data, 4);
    u32 nimages = rd32(data, 8), itab = rd32(data, 12);
    PmbFile out;
    std::string last_error;
    // Shipped files use 64-byte names; a few old test files use 32.
    for (u32 n : {64u, 32u}) {
        try {
            PmbFile f;
            f.name_size = n;
            Reader r(data, n);
            for (u32 i = 0; i < npages; ++i) f.pages.push_back(r.page(r.ptr(table + size_t(i) * 4)));
            if (!names_plausible(f)) throw FormatError("pmb: implausible names");
            out = std::move(f);
            last_error.clear();
            break;
        } catch (const FormatError& e) {
            last_error = e.what();
        }
    }
    if (!last_error.empty()) throw FormatError(last_error);
    if (nimages > kMaxCount || itab + size_t(nimages) * 8 > data.size()) throw FormatError("pmb: bad image table");
    for (u32 i = 0; i < nimages; ++i) {
        PmbImage im;
        im.size = rd32(data, itab + i * 8);
        im.offset = rd32(data, itab + i * 8 + 4);
        if (im.offset + 4 > data.size()) throw FormatError("pmb: image offset out of range");
        im.gzip = data[im.offset] == 0x1F && data[im.offset + 1] == 0x8B;
        if (!im.gzip && !has_magic(data.subspan(im.offset), "Tex1")) throw FormatError("pmb: unknown image data");
        out.images.push_back(im);
    }
    return out;
}

Bytes PmbFile::image_data(ByteSpan file, size_t i) const {
    const PmbImage& im = images.at(i);
    ByteSpan rest = file.subspan(im.offset);
    if (im.gzip) return inflate_gzip(rest, im.size);
    if (rest.size() < im.size) throw FormatError("pmb: image truncated");
    return Bytes(rest.begin(), rest.begin() + im.size);
}

int PmbFile::find_page(std::string_view name) const {
    for (size_t i = 0; i < pages.size(); ++i)
        if (pages[i].name == name) return int(i);
    return -1;
}

FontSpec FontSpec::parse(std::string_view spec) {
    FontSpec f;
    // Windows-style specs in some Japanese files: "<face>|h-24|c128|...".
    if (size_t bar = spec.find('|'); bar != std::string_view::npos) {
        f.name = std::string(spec.substr(0, bar));
        size_t h = spec.find("|h", bar);
        if (h != std::string_view::npos) {
            int v = 0;
            const char* p = spec.data() + h + 2;
            if (*p == '-') ++p;
            std::from_chars(p, spec.data() + spec.size(), v);
            if (v > 0) f.size = float(v);
        }
        return f;
    }
    size_t star = spec.find('*');
    std::string_view head = spec.substr(0, star);
    if (star != std::string_view::npos) {
        std::string_view sc = spec.substr(star + 1);
        int a = 100, b = -1;
        auto r = std::from_chars(sc.data(), sc.data() + sc.size(), a);
        if (r.ptr < sc.data() + sc.size() && *r.ptr == ',') std::from_chars(r.ptr + 1, sc.data() + sc.size(), b);
        f.sx = a / 100.0f;
        f.sy = b < 0 ? f.sx : b / 100.0f; // a single value scales both axes (confirmed in game code)
    }
    size_t hash = head.find('#');
    f.name = std::string(head.substr(0, hash));
    if (hash != std::string_view::npos) {
        std::string_view v = head.substr(hash + 1);
        // The game reads one character: anything below '0' (e.g. "#!1") selects look 0.
        if (!v.empty() && v[0] < '0') f.variant = 0;
        else std::from_chars(v.data(), v.data() + v.size(), f.variant);
    }
    size_t us = f.name.rfind('_');
    if (us != std::string::npos) {
        int sz = 0;
        auto r = std::from_chars(f.name.data() + us + 1, f.name.data() + f.name.size(), sz);
        if (r.ec == std::errc() && sz > 0) f.size = float(sz);
    }
    return f;
}

} // namespace gt3r::menu
