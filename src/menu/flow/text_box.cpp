#include "menu/flow/text_box.h"

#include <algorithm>

namespace gt3r::menu::flow {

void TextBox::reset(std::string t, size_t max_chars) {
    limit = max_chars;
    text = std::move(t);
    if (text.size() > limit) text.resize(limit);
    cursor = text.size();
}

bool TextBox::insert(const std::string& key) {
    if (text.size() >= limit) return false;
    const std::string c = key.empty() ? std::string(" ") : key.substr(0, 1);
    cursor = std::min(cursor, text.size());
    text.insert(cursor, c);
    cursor += c.size();
    return true;
}

bool TextBox::back_space() {
    const bool had_text = !text.empty();
    cursor = std::min(cursor, text.size());
    if (cursor > 0) --cursor;
    if (cursor < text.size()) text.erase(cursor, 1);
    return had_text;
}

bool TextBox::move(int dir) {
    const size_t before = cursor = std::min(cursor, text.size());
    if (dir < 0 && cursor > 0) --cursor;
    else if (dir > 0 && cursor < text.size()) ++cursor;
    return cursor != before;
}

} // namespace gt3r::menu::flow
