#include "game/gcf.h"

#include <algorithm>

namespace gt3r::game {

std::string Gcf::Entry::str_at(size_t off) const {
    std::string s;
    for (size_t i = off; i < bytes.size() && bytes[i]; ++i) s.push_back(char(bytes[i]));
    return s;
}

const Gcf::Entry* Gcf::Section::find(const std::string& name) const {
    for (const Entry& e : entries)
        if (e.name == name) return &e;
    return nullptr;
}

Gcf Gcf::parse(Bytes data) {
    Gcf g;
    g.data_ = std::move(data);
    ByteSpan d(g.data_);
    u32 n = rd32(d, 0), table = rd32(d, 4);
    if (n > 256 || size_t(table) + 8ull * n > d.size()) throw FormatError("gcf: bad section table");
    std::vector<std::pair<u32, u32>> secs;
    for (u32 i = 0; i < n; ++i) secs.push_back({rd32(d, table + 8 * i), rd32(d, table + 8 * i + 4)});
    g.offsets_ = secs;
    std::vector<u32> starts;
    for (auto& s : secs) starts.push_back(s.second);
    starts.push_back(u32(d.size()));
    std::sort(starts.begin(), starts.end());
    for (auto [tag, off] : secs) {
        Section s;
        s.tag = tag;
        u32 end = *std::upper_bound(starts.begin(), starts.end(), off);
        u32 count = rd32(d, off), et = rd32(d, off + 4);
        // Some sections (tag 0) are not entry lists; keep them empty.
        if (count == 0 || et != 8 || size_t(off) + 8 + 4ull * count > end) {
            g.sections_.push_back(std::move(s));
            continue;
        }
        std::vector<u32> eo;
        for (u32 i = 0; i < count; ++i) eo.push_back(off + rd32(d, off + 8 + 4 * i));
        for (u32 i = 0; i < count; ++i) {
            u32 a = eo[i];
            u32 b = i + 1 < count && eo[i + 1] > a ? eo[i + 1] : end;
            if (a >= end || b > d.size()) throw FormatError("gcf: entry out of range");
            Entry e;
            e.bytes = d.subspan(a, b - a);
            e.name = e.str_at(rd32(d, a));
            s.entries.push_back(std::move(e));
        }
        g.sections_.push_back(std::move(s));
    }
    return g;
}

const Gcf::Section* Gcf::section(u32 tag) const {
    for (const Section& s : sections_)
        if (s.tag == tag) return &s;
    return nullptr;
}

std::optional<u32> Gcf::section_u32(u32 tag, size_t off) const {
    for (auto [t, o] : offsets_)
        if (t == tag && size_t(o) + off + 4 <= data_.size()) return rd32(ByteSpan(data_), o + off);
    return std::nullopt;
}

std::optional<u32> Gcf::value(const std::string& name) const {
    const Section* s = section(10);
    const Entry* e = s ? s->find(name) : nullptr;
    if (!e) return std::nullopt;
    return e->u32_at(4);
}

} // namespace gt3r::game
