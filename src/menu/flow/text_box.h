#pragma once

// The name entry's text box (menu/name_entry.mbl, the memory card's name_entry page): the
// text being entered and its cursor, edited as the original's text box does (menus §12.2).
// A key inserts its character at the cursor while the text is shorter than the limit; Back
// Space steps the cursor left and deletes the character there; L1 / R1 move the cursor over
// the text. The cursor starts at the end of the starting text.

#include <cstddef>
#include <string>

namespace gt3r::menu::flow {

struct TextBox {
    std::string text;
    size_t cursor = 0;   // insertion point, 0 .. text.size()
    size_t limit = 15;   // characters at most

    // A new entry starting with `t` (cut to the limit), the cursor after it.
    void reset(std::string t, size_t max_chars);
    // A key: its character at the cursor (a key without text types a space), the cursor
    // after it. False when the text is full (nothing happens).
    bool insert(const std::string& key);
    // Back Space: the cursor one to the left (it stays at the start) and the character at it
    // deleted, so at the start the first character goes. True when the text was not empty
    // (the original's cursor sound).
    bool back_space();
    // L1 (-1) / R1 (+1): the cursor one character over the text; true when it moved.
    bool move(int dir);
};

} // namespace gt3r::menu::flow
