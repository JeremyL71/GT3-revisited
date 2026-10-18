#include "formats/text/mltx.h"

#include <algorithm>

namespace gt3r::text {

namespace {

constexpr const char* kCodes[kLanguageCount] = {"JP", "US", "BR", "FR", "GR", "IT", "SP"};
constexpr const char* kNames[kLanguageCount] = {"Japanese", "English (US)", "English (UK)", "French",
                                                "German",   "Italian",      "Spanish"};

std::string cstring_at(ByteSpan d, size_t off, size_t end) {
    if (off >= end) throw FormatError("MLTX string offset out of range");
    size_t e = off;
    while (e < end && d[e]) ++e;
    return std::string(reinterpret_cast<const char*>(d.data() + off), e - off);
}

} // namespace

const char* language_code(Language l) { return kCodes[size_t(l) % kLanguageCount]; }
const char* language_name(Language l) { return kNames[size_t(l) % kLanguageCount]; }

std::optional<Language> language_from_code(std::string_view code) {
    for (int i = 0; i < kLanguageCount; ++i)
        if (code == kCodes[i]) return Language(i);
    return std::nullopt;
}

StringTable StringTable::parse(ByteSpan d) {
    if (!is_mltx(d)) throw FormatError("not an MLTX string table");
    if (rd32(d, 4) != 2) throw FormatError("unsupported MLTX version");
    u32 pool = rd32(d, 8), count = rd32(d, 12), pool_size = rd32(d, 16);
    if (pool > d.size() || 0x14 + u64(count) * 8 > pool) throw FormatError("MLTX header out of range");
    StringTable t;
    // The file holds as many complete pools as fit (7 on retail discs).
    int langs = pool_size ? int(std::min<u64>(kLanguageCount, (d.size() - pool) / pool_size)) : 0;
    if (langs == 0) throw FormatError("MLTX has no language pools");
    t.values_.resize(langs);
    for (u32 i = 0; i < count; ++i) {
        u32 key = rd32(d, 0x14 + i * 8), value = rd32(d, 0x18 + i * 8);
        t.keys_.push_back(cstring_at(d, key, pool));
        for (int l = 0; l < langs; ++l) {
            size_t base = pool + size_t(l) * pool_size;
            t.values_[l].push_back(cstring_at(d, base + value, base + pool_size));
        }
    }
    return t;
}

size_t StringTable::filled(Language l) const {
    if (size_t(l) >= values_.size()) return 0;
    const auto& v = values_[size_t(l)];
    return size_t(std::count_if(v.begin(), v.end(), [](const std::string& s) { return !s.empty(); }));
}

int StringTable::find(std::string_view key) const {
    for (size_t i = 0; i < keys_.size(); ++i)
        if (keys_[i] == key) return int(i);
    return -1;
}

} // namespace gt3r::text
