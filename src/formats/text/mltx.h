#pragma once

// "MLTX" multi-language string tables (text/realtime.dat).
//
//   0x00 "MLTX"
//   0x04 u32 version (2)
//   0x08 u32 pool offset          -> language 0's string pool
//   0x0C u32 entry count
//   0x10 u32 pool size            -> language L's pool is at pool_offset + L * pool_size
//   0x14 entries[count]           {u32 key offset (absolute), u32 value offset (in pool)}
//   ...  keys (NUL-terminated ASCII, sorted)
//   ...  7 pools of identical layout (each value is padded so all languages
//        share the same offsets)
//
// Languages are indexed in the game's order: JP, US, BR (British English),
// FR, GR (German), IT, SP. An empty value means "keep the default text built
// into the program" for that key; on PBPX-955.03 only JP and US are complete,
// BR is partial and FR..SP are blank. Values use the game encoding
// (see charset.h).

#include "base/bytes.h"
#include "formats/text/charset.h"

#include <optional>
#include <string>
#include <vector>

namespace gt3r::text {

enum class Language : u8 { JP, US, BR, FR, GR, IT, SP };
constexpr int kLanguageCount = 7;
// "JP", "US", "BR", ... (also the menu/<LANG>/ folder names).
const char* language_code(Language l);
// "Japanese", "English (US)", ...
const char* language_name(Language l);
std::optional<Language> language_from_code(std::string_view code);

class StringTable {
public:
    static bool is_mltx(ByteSpan data) { return has_magic(data, "MLTX"); }
    static StringTable parse(ByteSpan data);

    size_t size() const { return keys_.size(); }
    int language_count() const { return int(values_.size()); }
    const std::string& key(size_t i) const { return keys_.at(i); }
    // Raw game-encoded value ("" = use the built-in default).
    const std::string& raw(size_t i, Language l) const { return values_.at(size_t(l)).at(i); }
    std::string utf8(size_t i, Language l, const JisTable* jis) const { return game_text_to_utf8(raw(i, l), jis); }
    // Number of non-empty values for a language.
    size_t filled(Language l) const;
    // Entry index for a key, or -1.
    int find(std::string_view key) const;

private:
    std::vector<std::string> keys_;
    std::vector<std::vector<std::string>> values_; // [language][entry]
};

} // namespace gt3r::text
