#include "formats/text/text_layout.h"

#include <algorithm>

namespace gt3r::text {

std::vector<GlyphRef> map_unicode(std::u32string_view text, const LatinFont& font, const KanjiFont* kanji,
                                  const JisTable* jis) {
    std::vector<GlyphRef> out;
    out.reserve(text.size());
    for (char32_t c : text) {
        if (c == '\n' || c == '\r') {
            out.push_back({false, u16(c)});
            continue;
        }
        u16 code = unicode_to_font_code(c);
        if (code != kNoCode && font.has(code)) {
            out.push_back({false, code});
            continue;
        }
        if (kanji && jis) {
            if (u16 j = jis->from_unicode(c); j && kanji->has(j)) {
                out.push_back({true, j});
                continue;
            }
        }
        out.push_back({false, u16('?')});
    }
    return out;
}

std::vector<GlyphRef> map_game_text(ByteSpan s) {
    std::vector<GlyphRef> out;
    for (size_t i = 0; i < s.size() && s[i]; ++i) {
        u8 b = s[i];
        if (b < 0x80) {
            out.push_back({false, b});
            continue;
        }
        if (i + 1 >= s.size() || !s[i + 1]) break;
        u8 c = s[++i];
        if (b >= 0xA1)
            out.push_back({true, u16((b & 0x7F) << 8 | (c & 0x7F))});
        else if (b <= 0x8F)
            out.push_back({false, iso8859_to_font_code(b - 0x80, c)}); // kNoCode is skipped like the game
    }
    return out;
}

TextLayout layout_text(const std::vector<GlyphRef>& glyphs, const LatinFont& font, const KanjiFont* kanji, float x,
                       float y, const LayoutOptions& opt) {
    TextLayout lay;
    const float sx = opt.scale_x, sy = opt.scale_y;
    const float line_advance = font.line_height * sy + opt.line_spacing;
    float pen = 0, base = 0, widest = 0;
    const FontGlyph* prev = nullptr;
    for (const GlyphRef& g : glyphs) {
        if (!g.kanji && (g.code == '\n' || g.code == '\r')) {
            widest = std::max(widest, pen);
            pen = 0;
            prev = nullptr;
            if (g.code == '\n') {
                base += line_advance;
                ++lay.lines;
            }
            continue;
        }
        if (g.kanji) {
            prev = nullptr;
            if (!kanji) continue;
            GlyphQuad q;
            q.kanji = true;
            q.code = g.code;
            q.x = pen + font.kanji_dx * sx;
            q.y = base + font.kanji_dy * sy;
            q.w = font.kanji_width * sx;
            q.h = font.kanji_height * sy;
            q.sw = u16(kanji->cell_width());
            q.sh = u16(kanji->cell_height());
            pen = q.x + q.w + opt.spacing;
            if (kanji->drawable(g.code)) lay.quads.push_back(q);
            continue;
        }
        const FontGlyph* fg = font.glyph(g.code);
        if (!fg) { // out of the font's range: skipped, kerning restarts
            prev = nullptr;
            continue;
        }
        int kern = prev ? font.kerning(*prev, *fg) : 0;
        GlyphQuad q;
        q.code = g.code;
        q.x = pen + float(fg->left - kern) * sx;
        q.y = base + fg->top * sy;
        q.w = fg->width * sx;
        q.h = fg->height * sy;
        q.sx = fg->atlas_x;
        q.sy = fg->atlas_y;
        q.sw = fg->width;
        q.sh = fg->height;
        pen = q.x + fg->advance * sx + opt.spacing;
        prev = fg;
        if (fg->width && fg->height) lay.quads.push_back(q);
    }
    widest = std::max(widest, pen);
    lay.width = widest;
    lay.height = font.line_height * sy + float(lay.lines - 1) * line_advance;

    float ox = x;
    if (opt.align == HAlign::Right) ox -= lay.width;
    else if (opt.align == HAlign::Center) ox -= lay.width * 0.5f;
    float baseline = y;
    const float ascent = font.ascent * sy;
    switch (opt.valign) {
    case VAlign::Baseline: break;
    case VAlign::Top: baseline = y + ascent; break;
    case VAlign::Middle: baseline = y + ascent - lay.height * 0.5f; break;
    case VAlign::Bottom: baseline = y + ascent - lay.height; break;
    }
    for (GlyphQuad& q : lay.quads) {
        q.x += ox;
        q.y += baseline;
    }
    lay.x = ox;
    lay.y = baseline - ascent;
    return lay;
}

} // namespace gt3r::text
