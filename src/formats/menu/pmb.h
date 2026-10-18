#pragma once

#include "base/bytes.h"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace gt3r::menu {

// .pmb menu layout ("page" file), menu/<LANG>/<screen><n>.pmb. See
// docs/formats/menu.md for the full description.
//
// The file is a serialized object tree in which every pointer is a file
// offset (the game relocates them in place after loading):
//
//   0x00 u32 page count        0x04 u32 offset of page pointer table (0x10)
//   0x08 u32 image count       0x0C u32 offset of image table
//
//   Page  = Group:  char name[N]; u32 n_items; u32 items; u32 n_layers; u32 layers
//   Layer = Group:  char name[N]; u32 n_elems; u32 elems; u32 n_widgets; u32 widgets
//   Element:        char name[N]; u16 type; u16 flags;
//                   f32 x, y, z, w, h;
//                   u32 widget_class -> u32 -> char[]   (type-independent)
//                   u32 alt          -> Element          (type & 8)
//                   u32 image        -> ImageRef         (type & 1)
//                   u32 colors       -> u32 rgba[4]      (type & 2)
//                   u32 text         -> TextStyle        (type & 4)
//   Image table:    { u32 unpacked_size; u32 offset } per image; the data is a
//                   Tex1 set, either raw or as a gzip member.
//
// N (the name field size) is 64 in all shipped files and 32 in a few old
// Japanese test files; it is detected automatically.
//
// Coordinates are in a 640x448 space with the origin at the bottom left
// (y grows upwards); (x, y) is the bottom-left corner of the element box.
// z orders drawing (larger z is in front).

struct Rgba {
    u8 r = 0, g = 0, b = 0, a = 0; // a: 0x80 = opaque for text; rect colors use 0xFF
};

struct ImageRef {
    u32 image = 0;               // index into PmbFile::images
    std::array<float, 8> uv{};   // (u,v) for the 4 corners, same order as colors
};

struct TextStyle {
    u16 unk0 = 0;   // always 1
    u16 align = 0;  // low nibble: 3x3 anchor, 0 = top-left .. 8 = bottom-right; 0x10/0x20 unknown flags
    Rgba color;
    std::string text;
    std::string font; // e.g. "sw_16#1*90": font name, "#variant", "*xscale[,yscale]" percent
};

struct Element {
    // Bits of `type` that select the optional parts.
    static constexpr u16 kImage = 1, kColor = 2, kText = 4, kAlt = 8;

    std::string name;
    // `type` and `flags` are the low and high halves of one u32 flag word:
    // type: low nibble parts present; bits 4-7 blend mode; bits 8-11 edge
    //   decoration; bits 12-15 focus style (0x2000 pulsing highlight, 0x1000 static).
    // flags: bits 0-1 name payload (widget class), bit 2 texture is gzip,
    //   bit 3 ?, bits 4-7 fade in on appear (24 frames), bits 8-11 fade out on
    //   dismiss (9 frames).
    u16 type = 0;
    u16 flags = 0;
    float x = 0, y = 0, z = 0, w = 0, h = 0;
    std::optional<std::string> widget_class; // set for layer widgets (bound to game code by class name)
    std::optional<ImageRef> image;
    std::optional<std::array<Rgba, 4>> colors; // corners: top-left, top-right, bottom-right, bottom-left
    std::optional<TextStyle> text;
    std::vector<Element> alt; // 0 or 1 alternative state (focused/selected look)
    size_t offset = 0;        // file offset, for debugging
};

struct Layer {
    std::string name;
    std::vector<Element> elements;
    std::vector<Element> widgets; // elements with a widget_class, drawn by game code
};

struct Page {
    std::string name;
    std::vector<Element> items; // selectable page items (buttons)
    std::vector<Layer> layers;
};

struct PmbImage {
    u32 size = 0;   // unpacked size
    u32 offset = 0;
    bool gzip = false;
};

struct PmbFile {
    u32 name_size = 64;
    std::vector<Page> pages;
    std::vector<PmbImage> images;

    static bool is_pmb(ByteSpan data);
    static PmbFile parse(ByteSpan data);

    // Returns the Tex1 data of image `i` (decompressed if needed).
    Bytes image_data(ByteSpan file, size_t i) const;
    int find_page(std::string_view name) const;
};

// Splits a font spec such as "sw_16#1*90,100" into name "sw_16", pixel size
// 16, x/y scale. Unknown specs give size 16 and scale 1.
struct FontSpec {
    std::string name;
    int variant = 0;
    float size = 16;
    float sx = 1, sy = 1;
    static FontSpec parse(std::string_view spec);
};

} // namespace gt3r::menu
