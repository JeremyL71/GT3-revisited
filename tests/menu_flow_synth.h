#pragma once

// Writers for small synthetic .mbl/.pmb files used by the menu-flow tests
// (no game data). Layouts follow docs/formats/menu.md.

#include "base/bytes.h"

#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace gt3r::test::menusynth {

class Buf {
public:
    size_t alloc(size_t n) {
        size_t o = (b.size() + 3) & ~size_t(3);
        b.resize(o + n, 0);
        return o;
    }
    void u32_(size_t o, u32 v) { std::memcpy(&b[o], &v, 4); }
    void u16_(size_t o, u16 v) { std::memcpy(&b[o], &v, 2); }
    void f32_(size_t o, float f) { std::memcpy(&b[o], &f, 4); }
    void str(size_t o, const std::string& s) { std::memcpy(&b[o], s.data(), s.size()); }
    size_t cstr(const std::string& s) {
        size_t o = alloc(s.size() + 1);
        str(o, s);
        return o;
    }
    Bytes b;
};

struct SynElement {
    std::string name;
    float x = 0, y = 0, w = 100, h = 20, z = 1;
    u32 rgba = 0xFFFFFFFF; // r in the low byte
    std::string text;
    bool has_alt = false;
    u32 alt_rgba = 0xFF00FFFF;
    // Where the alt sits in the layout, relative to the element (the game's files often place
    // it elsewhere: shifted, parked off-screen, at 0, 0), and its z relative to the element's.
    float alt_dx = 0, alt_dy = 0, alt_dz = 1;
    u16 focus_style = 0; // element type bits 12-15: 0x2000 pulsing highlight, 0x1000 static
    u16 flags = 0x0C;
    std::string widget_class; // bound to game code by class name ("" = none)
    bool widget = false;      // in a layer: written to the layer's widget table, not its elements
    bool image = false;       // an image part (image 0; the file has no images, so nothing decodes)
    u16 align = 4;            // text anchor (3x3, 0 = top-left)
};

struct SynLayer {
    std::string name;
    std::vector<SynElement> elements;
};

struct SynPage {
    std::string name;
    std::vector<SynElement> items;
    std::vector<SynLayer> layers;
};

inline size_t write_element(Buf& w, const SynElement& e, bool alt) {
    size_t o = w.alloc(0x6C);
    w.str(o, e.name);
    u16 type = u16(2 | (e.text.empty() ? 0 : 4) | (e.has_alt && !alt ? 8 : 0) | (e.image ? 1 : 0) |
                   (alt ? 0 : e.focus_style));
    w.u16_(o + 0x40, type);
    w.u16_(o + 0x42, alt ? 0x0F : e.flags);
    w.f32_(o + 0x44, alt ? e.x + e.alt_dx : e.x);
    w.f32_(o + 0x48, alt ? e.y + e.alt_dy : e.y);
    w.f32_(o + 0x4C, alt ? e.z + e.alt_dz : e.z);
    w.f32_(o + 0x50, e.w);
    w.f32_(o + 0x54, e.h);
    size_t c = w.alloc(16);
    for (int i = 0; i < 4; ++i) w.u32_(c + i * 4, alt ? e.alt_rgba : e.rgba);
    w.u32_(o + 0x64, u32(c));
    if (!e.widget_class.empty()) {
        size_t p = w.alloc(4);
        w.u32_(p, u32(w.cstr(e.widget_class)));
        w.u32_(o + 0x58, u32(p));
    }
    if (e.image) w.u32_(o + 0x60, u32(w.alloc(36))); // image 0, zero UVs
    if (!e.text.empty()) {
        size_t t = w.alloc(16);
        w.u16_(t, 1);
        w.u16_(t + 2, e.align);
        w.u32_(t + 4, 0x80FFFFFF);
        w.u32_(t + 8, u32(w.cstr(e.text)));
        w.u32_(t + 12, u32(w.cstr("sw_16")));
        w.u32_(o + 0x68, u32(t));
    }
    if (e.has_alt && !alt) w.u32_(o + 0x5C, u32(write_element(w, e, true)));
    return o;
}

inline void write_list(Buf& w, size_t field, const std::vector<size_t>& ptrs) {
    size_t t = w.alloc(4 * std::max<size_t>(1, ptrs.size()));
    for (size_t i = 0; i < ptrs.size(); ++i) w.u32_(t + i * 4, u32(ptrs[i]));
    w.u32_(field, u32(ptrs.size()));
    w.u32_(field + 4, u32(t));
}

// Pages must be given sorted by name, as in the game's files.
inline Bytes write_pmb(const std::vector<SynPage>& pages) {
    Buf w;
    w.alloc(0x10);
    size_t table = w.alloc(4 * pages.size());
    w.u32_(0, u32(pages.size()));
    w.u32_(4, u32(table));
    for (size_t p = 0; p < pages.size(); ++p) {
        const SynPage& pg = pages[p];
        size_t po = w.alloc(0x50);
        w.str(po, pg.name);
        w.u32_(table + p * 4, u32(po));
        std::vector<size_t> items, layers;
        for (const SynElement& e : pg.items) items.push_back(write_element(w, e, false));
        for (const SynLayer& l : pg.layers) {
            size_t lo = w.alloc(0x50);
            w.str(lo, l.name);
            std::vector<size_t> els, widgets;
            for (const SynElement& e : l.elements)
                (e.widget ? widgets : els).push_back(write_element(w, e, false));
            write_list(w, lo + 0x40, els);
            write_list(w, lo + 0x48, widgets);
            layers.push_back(lo);
        }
        write_list(w, po + 0x40, items);
        write_list(w, po + 0x48, layers);
    }
    w.alloc(4);
    w.u32_(8, 0);
    w.u32_(0xC, u32(w.b.size()));
    return w.b;
}

struct SynItem {
    std::string name, str40, str80, action, link;
    s16 id = -1;
    u16 kind = 0;
    std::vector<SynItem> children;
};

struct SynFlowPage {
    std::string name;
    int pmb = 1;
    u16 flags = 0x1E0;
    std::string import, back, back_action;
    std::vector<SynItem> items;
};

inline size_t write_item(Buf& w, const SynItem& it) {
    size_t o = w.alloc(0x14C);
    w.str(o, it.name);
    w.str(o + 0x40, it.str40);
    w.str(o + 0x80, it.str80);
    w.str(o + 0xC0, it.action);
    w.str(o + 0x100, it.link);
    w.u16_(o + 0x140, u16(it.id));
    w.u16_(o + 0x142, it.kind);
    std::vector<size_t> ch;
    for (const SynItem& c : it.children) ch.push_back(write_item(w, c));
    write_list(w, o + 0x144, ch);
    return o;
}

// Pages must be given sorted by name.
inline Bytes write_mbl(const std::vector<SynFlowPage>& pages) {
    Buf w;
    w.alloc(8);
    size_t table = w.alloc(4 * pages.size());
    w.u32_(0, u32(pages.size()));
    w.u32_(4, u32(table));
    for (size_t p = 0; p < pages.size(); ++p) {
        const SynFlowPage& pg = pages[p];
        size_t o = w.alloc(0x10C);
        w.u32_(table + p * 4, u32(o));
        w.str(o, pg.name);
        w.u16_(o + 0x40, u16(pg.pmb));
        w.u16_(o + 0x42, pg.flags);
        w.str(o + 0x44, pg.import);
        w.str(o + 0x84, pg.back);
        w.str(o + 0xC4, pg.back_action);
        std::vector<size_t> items;
        // The item table must follow the page header directly (layout detection).
        size_t t = w.alloc(4 * std::max<size_t>(1, pg.items.size()));
        w.u32_(o + 0x104, u32(pg.items.size()));
        w.u32_(o + 0x108, u32(t));
        for (size_t i = 0; i < pg.items.size(); ++i) w.u32_(t + i * 4, u32(write_item(w, pg.items[i])));
    }
    size_t end = w.alloc(4);
    w.u32_(end, u32(end + 4));
    return w.b;
}

} // namespace gt3r::test::menusynth
