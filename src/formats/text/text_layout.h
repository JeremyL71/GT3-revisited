#pragma once

// Text layout exactly as GT3's renderer does it (no SDL): turns a string into
// positioned glyph quads for a LatinFont plus an optional KanjiFont.
//
// Per Latin glyph g following glyph p on the same line (pen = current x):
//   x0   = pen + (g.left - kerning(p, g)) * sx      y0 = baseline + g.top * sy
//   size = (g.width * sx, g.height * sy)
//   pen  = x0 + g.advance * sx + spacing
// Per kanji: x0 = pen + kanji_dx * sx, size = kanji_w/h * s, y0 = baseline +
// kanji_dy * sy, pen = x0 + kanji_w * sx + spacing; kerning restarts after it.
// '\n' returns to the start x and moves down line_height * sy + line_spacing.
// '\r' returns to the start x on the same line.
//
// Alignment works on the whole block like the game: the block is shifted by
// its widest line (not per line). Vertically, Top/Middle/Bottom place the
// first baseline at y + ascent * sy, minus the block height (or half of it)
// for Bottom (Middle).

#include "formats/text/charset.h"
#include "formats/text/fnt.h"

#include <string_view>
#include <vector>

namespace gt3r::text {

enum class HAlign : u8 { Left, Center, Right };
enum class VAlign : u8 { Baseline, Top, Middle, Bottom };

struct LayoutOptions {
    float scale_x = 1.0f, scale_y = 1.0f;
    float spacing = 0.0f;      // extra pixels after every glyph
    float line_spacing = 0.0f; // extra pixels between lines
    HAlign align = HAlign::Left;
    VAlign valign = VAlign::Top;
};

// One character to lay out: a Latin font code or a JIS code for the kanji font.
struct GlyphRef {
    bool kanji = false;
    u16 code = 0;
};

struct GlyphQuad {
    bool kanji = false;
    u16 code = 0;               // font code or JIS code
    float x = 0, y = 0, w = 0, h = 0;  // destination
    u16 sx = 0, sy = 0, sw = 0, sh = 0; // source rect (Latin atlas, or the kanji cell)
};

struct TextLayout {
    std::vector<GlyphQuad> quads;
    float x = 0, y = 0;          // top-left of the block (top = first baseline - ascent)
    float width = 0, height = 0; // widest line; line_height * lines (+ line spacing)
    int lines = 1;
};

// Maps Unicode text to glyph references: Latin font first, then the kanji
// font via `jis`, else '?'. '\n' and '\r' pass through as Latin codes.
std::vector<GlyphRef> map_unicode(std::u32string_view text, const LatinFont& font, const KanjiFont* kanji,
                                  const JisTable* jis);
// Maps a game-encoded string exactly like the game: ASCII and {0x80+N, c}
// escapes to the Latin font, EUC-JP pairs to the kanji font.
std::vector<GlyphRef> map_game_text(ByteSpan text);

// Lays out at (x, y). `kanji` may be null (kanji are then skipped).
TextLayout layout_text(const std::vector<GlyphRef>& glyphs, const LatinFont& font, const KanjiFont* kanji, float x,
                       float y, const LayoutOptions& opt);

} // namespace gt3r::text
