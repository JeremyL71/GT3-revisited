#include "base/config.h"

#include "base/file.h"

#include <cctype>

namespace gt3r {

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

std::vector<std::string_view> split(std::string_view s, char sep) {
    std::vector<std::string_view> out;
    size_t start = 0;
    for (size_t i = 0; i <= s.size(); ++i) {
        if (i == s.size() || s[i] == sep) {
            out.push_back(s.substr(start, i - start));
            start = i + 1;
        }
    }
    return out;
}

std::string to_lower(std::string_view s) {
    std::string out(s);
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

IniDocument IniDocument::parse(std::string_view text) {
    IniDocument doc;
    Section* cur = &doc.get_or_add("");
    for (std::string_view line : split(text, '\n')) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        if (line.front() == '[') { // "[name]", optionally followed by a comment: "[paints]  ; in menu order"
            size_t close = line.find(']');
            std::string_view rest = close == std::string_view::npos ? line : trim(line.substr(close + 1));
            if (close != std::string_view::npos && (rest.empty() || rest[0] == ';' || rest[0] == '#')) {
                cur = &doc.get_or_add(trim(line.substr(1, close - 1)));
                continue;
            }
        }
        size_t eq = line.find('=');
        if (eq == std::string_view::npos) continue;
        cur->entries.emplace_back(std::string(trim(line.substr(0, eq))),
                                  std::string(trim(line.substr(eq + 1))));
    }
    return doc;
}

std::optional<IniDocument> IniDocument::load(const std::string& path) {
    auto bytes = read_host_file(path);
    if (!bytes) return std::nullopt;
    return parse(std::string_view(reinterpret_cast<const char*>(bytes->data()), bytes->size()));
}

std::string IniDocument::serialize() const {
    std::string out;
    for (const Section& s : sections_) {
        if (s.name.empty() && s.entries.empty()) continue;
        if (!s.name.empty()) out += "[" + s.name + "]\n";
        for (const auto& [k, v] : s.entries) out += k + " = " + v + "\n";
        out += "\n";
    }
    return out;
}

bool IniDocument::save(const std::string& path) const {
    std::string text = serialize();
    return write_host_file(path, ByteSpan(reinterpret_cast<const u8*>(text.data()), text.size()));
}

const IniDocument::Section* IniDocument::find(std::string_view section) const {
    for (const Section& s : sections_)
        if (s.name == section) return &s;
    return nullptr;
}

IniDocument::Section& IniDocument::get_or_add(std::string_view section) {
    for (Section& s : sections_)
        if (s.name == section) return s;
    sections_.push_back(Section{std::string(section), {}});
    return sections_.back();
}

std::optional<std::string> IniDocument::get(std::string_view section, std::string_view key) const {
    if (const Section* s = find(section))
        for (const auto& [k, v] : s->entries)
            if (k == key) return v;
    return std::nullopt;
}

void IniDocument::set(std::string_view section, std::string_view key, std::string_view value) {
    Section& s = get_or_add(section);
    for (auto& [k, v] : s.entries)
        if (k == key) {
            v = value;
            return;
        }
    s.entries.emplace_back(std::string(key), std::string(value));
}

} // namespace gt3r
