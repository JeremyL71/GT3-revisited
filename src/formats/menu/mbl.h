#pragma once

#include "base/bytes.h"

#include <string>
#include <vector>

namespace gt3r::menu {

// .mbl menu flow ("menu button links"), menu/<screen>.mbl. Describes, per
// pmb page, what each selectable item does: the action string handed to game
// code and/or the page to switch to. Same relocatable-pointer style as .pmb:
//
//   0x00 u32 page count, 0x04 u32 offset of page pointer table (8)
//   ...  last u32 of the file = file size
//
// Current layout (every file the game loads):
//   Page (0x10C): char name[64]      "pmbfile/page", e.g. "ghost/ghost"
//                 u16 unk40 (1); u16 flags42
//                 char str44[64]
//                 char back[64]      page to return to on cancel
//                 char back_action[64]
//                 u32 n_items; u32 items -> Item*[]
//   Item (0x14C): char name[64]      element name in the pmb page
//                 char str40[64]; char str80[64]   pmb layer names (likely shown on focus)
//                 char action[64]    command for game code
//                 char link[64]      page to switch to
//                 s16 unk140; u16 unk142; u32 n_children; u32 children -> Item*[]
//
// A few old test files use a smaller layout (page header 0x84 or 0x64 bytes,
// items 0xAC bytes with only name/str40[32]/link); they are parsed too and
// flagged with `legacy`.
// Field meanings: str40 = "group" layer shown (sliding)
// while the item is focused, str80 = layer shown while focused, action = game
// command run on confirm, link = target page; unk140 = an id read only by
// control code; unk142 = kind: low nibble 0 go to link, 1 open link as popup,
// 2 finish (control code takes over), 3 back, 4 multi container (bit 4:
// children laid out horizontally, bit 5: choice wraps).
struct MblItem {
    std::string name, str40, str80, action, link;
    s16 unk140 = 0;
    u16 unk142 = 0;
    std::vector<MblItem> children;
};

// unk40 = pmb number N (layout in menu/<LANG>/<set>N.pmb), str44 = import
// page drawn with this one (e.g. "00cmn/status"), back = cancel target
// ("return" = the page we came from), back_action = command run on cancel.
// flags42: 0x1 horizontal list, 0x2 wrap, 0x4 fade from black on build,
// 0x8 fade to black on leave, 0x30 frame-buffer clear mode, 0xC0 confirm
// feedback (0 / 0x40 glow / 0x80 flicker / 0xC0 blink), 0x100 cancel allowed.
struct MblPage {
    std::string name;
    u16 unk40 = 0, flags42 = 0;
    std::string str44, back, back_action;
    std::vector<MblItem> items;
};

struct MblFile {
    bool legacy = false;
    u32 page_header = 0; // 0x104, 0x84 or 0x64
    std::vector<MblPage> pages;

    static bool is_mbl(ByteSpan data);
    static MblFile parse(ByteSpan data);
    int find_page(std::string_view name) const;
};

} // namespace gt3r::menu
