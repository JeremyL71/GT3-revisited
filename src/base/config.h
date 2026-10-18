#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace gt3r {

// Minimal INI document: `[section]` headers, `key = value` lines, `#` or `;`
// comments. Order and duplicate keys are preserved, which the input binding
// format relies on. Hand-editable by users and modders.
class IniDocument {
public:
    struct Section {
        std::string name;
        std::vector<std::pair<std::string, std::string>> entries;
    };

    static IniDocument parse(std::string_view text);
    static std::optional<IniDocument> load(const std::string& path);
    std::string serialize() const;
    bool save(const std::string& path) const;

    const Section* find(std::string_view section) const;
    Section& get_or_add(std::string_view section);
    std::optional<std::string> get(std::string_view section, std::string_view key) const;
    void set(std::string_view section, std::string_view key, std::string_view value);

    std::vector<Section>& sections() { return sections_; }
    const std::vector<Section>& sections() const { return sections_; }

private:
    std::vector<Section> sections_;
};

std::string_view trim(std::string_view s);
std::vector<std::string_view> split(std::string_view s, char sep);
std::string to_lower(std::string_view s);

} // namespace gt3r
