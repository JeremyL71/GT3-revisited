// gt3tool text commands: string tables, font atlases, text rendering.

#include "tool.h"

#include "formats/png_writer.h"
#include "formats/text/charset.h"
#include "formats/text/fnt.h"
#include "formats/text/mltx.h"
#include "formats/text/text_layout.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

namespace gt3r::tool {

namespace {

using namespace gt3r::text;

struct FontData {
    std::vector<std::pair<std::string, FontFile>> files;
    JisTable jis;
    std::unique_ptr<KanjiFont> kanji;

    explicit FontData(Volume& vol) {
        for (const char* p : {"font/system.fnt", "font/menu.fnt"})
            files.emplace_back(p, FontFile::parse(vol.read(need(vol, p))));
        jis = JisTable::parse(vol.read(need(vol, "font/jis2uni.dat")));
        kanji = std::make_unique<KanjiFont>(KanjiFont::parse(vol.read(need(vol, "font/system.kf"))));
    }
    const LatinFont* find(const std::string& name) const {
        for (auto& f : files)
            for (auto& lf : f.second.fonts)
                if (lf.name == name) return &lf;
        return nullptr;
    }
};

// Source-over blend of a straight-alpha RGBA pixel, tinted.
void blend(u32& dst, u32 src, u32 tint) {
    auto ch = [](u32 c, int s) { return (c >> s) & 0xFF; };
    u32 a = ch(src, 24) * ch(tint, 24) / 255;
    if (!a) return;
    u32 out = 0;
    for (int s = 0; s < 24; s += 8) {
        u32 sc = ch(src, s) * ch(tint, s) / 255, dc = ch(dst, s);
        out |= ((sc * a + dc * (255 - a)) / 255) << s;
    }
    u32 da = ch(dst, 24);
    out |= (a + da * (255 - a) / 255) << 24;
    dst = out;
}

// Nearest-neighbour CPU renderer for a layout (good enough for checking).
void draw_layout(std::vector<u32>& img, u32 w, u32 h, const TextLayout& lay, const LatinFont& font,
                 const KanjiFont* kanji, FontVariant v, u32 tint) {
    const Tex1::Image& atlas = font.atlas[size_t(v)];
    std::vector<u32> cell;
    for (const GlyphQuad& q : lay.quads) {
        const u32* src = nullptr;
        u32 src_w = 0;
        if (q.kanji) {
            cell.assign(size_t(q.sw) * q.sh, 0);
            if (!kanji || !kanji->render(q.code, v, cell.data())) continue;
            src = cell.data();
            src_w = q.sw;
        } else {
            src = atlas.rgba.data() + size_t(q.sy) * atlas.width + q.sx;
            src_w = atlas.width;
        }
        int x0 = int(std::lround(q.x)), y0 = int(std::lround(q.y));
        int dw = int(std::lround(q.w)), dh = int(std::lround(q.h));
        for (int y = 0; y < dh; ++y)
            for (int x = 0; x < dw; ++x) {
                int px = x0 + x, py = y0 + y;
                if (px < 0 || py < 0 || u32(px) >= w || u32(py) >= h) continue;
                u32 sx = u32(x * q.sw / std::max(dw, 1)), sy = u32(y * q.sh / std::max(dh, 1));
                blend(img[size_t(py) * w + px], src[size_t(sy) * src_w + sx], tint);
            }
    }
}

std::string sanitize(std::string s) {
    for (char& c : s)
        if (c == '\n') c = '|';
    return s;
}

int cmd_text(Disc& disc, const Args& a) {
    Volume& vol = disc.volume();
    const std::string& out = a[0];
    FontData fd(vol);

    // String table, one UTF-8 file per language.
    StringTable st = StringTable::parse(vol.read(need(vol, "text/realtime.dat")));
    std::printf("text/realtime.dat: %zu keys, %d languages\n", st.size(), st.language_count());
    for (int l = 0; l < st.language_count(); ++l) {
        Language lang = Language(l);
        std::string body;
        size_t bad = 0;
        for (size_t i = 0; i < st.size(); ++i) {
            std::string v = st.utf8(i, lang, &fd.jis);
            if (v.find("\xEF\xBF\xBD") != std::string::npos) ++bad;
            body += st.key(i) + "\t" + sanitize(v) + "\n";
        }
        std::printf("  %s %-13s %3zu/%zu filled, %zu with unmapped characters\n", language_code(lang),
                    language_name(lang), st.filled(lang), st.size(), bad);
        if (!save(out + "/strings/realtime_" + language_code(lang) + ".txt",
                  ByteSpan(reinterpret_cast<const u8*>(body.data()), body.size())))
            return 1;
    }

    // Latin font atlases and metrics.
    std::string metrics;
    for (auto& [path, file] : fd.files) {
        std::string base = path.substr(path.find('/') + 1);
        base = base.substr(0, base.find('.'));
        for (const LatinFont& f : file.fonts) {
            size_t defined = std::count_if(f.glyphs.begin(), f.glyphs.end(), [](auto& g) { return g.defined(); });
            char line[256];
            std::snprintf(line, sizeof(line),
                          "%s/%s: codes 0x%X-0x%X (%zu defined), size %u, line %u, ascent %d, profile rows %u, "
                          "kanji %ux%u at (%d,%d), atlas %ux%u\n",
                          path.c_str(), f.name.c_str(), f.first, f.last, defined, f.size, f.line_height, f.ascent,
                          f.profile_rows, f.kanji_width, f.kanji_height, f.kanji_dx, f.kanji_dy, f.atlas[0].width,
                          f.atlas[0].height);
            std::fputs(line, stdout);
            metrics += line;
            for (int v = 0; v < kFontVariants; ++v) {
                const Tex1::Image& img = f.atlas[v];
                if (!save(out + "/fonts/" + base + "_" + f.name + "_v" + std::to_string(v) + ".png",
                          encode_png(img.width, img.height, img.rgba)))
                    return 1;
            }
        }
    }
    if (!save(out + "/fonts/metrics.txt", ByteSpan(reinterpret_cast<const u8*>(metrics.data()), metrics.size())))
        return 1;

    // Kanji fonts as grids of cells, 64 per row.
    for (const char* p : {"font/system.kf", "font/kanji22.kf"}) {
        KanjiFont k = KanjiFont::parse(vol.read(need(vol, p)));
        std::vector<u16> codes = k.codes();
        u32 cw = k.cell_width(), ch = k.cell_height(), cols = 64, rows = u32((codes.size() + cols - 1) / cols);
        std::vector<u32> img(size_t(cols * cw) * rows * ch, 0xFF402020u);
        std::vector<u32> cell(size_t(cw) * ch);
        for (size_t i = 0; i < codes.size(); ++i) {
            k.render(codes[i], FontVariant::Outlined, cell.data());
            u32 ox = u32(i % cols) * cw, oy = u32(i / cols) * ch;
            for (u32 y = 0; y < ch; ++y)
                for (u32 x = 0; x < cw; ++x) blend(img[size_t(oy + y) * cols * cw + ox + x], cell[y * cw + x], ~0u);
        }
        std::printf("%s: %s, %ux%u cells, %zu glyphs, %zu mapped codes\n", p, k.name().c_str(), cw, ch,
                    k.glyph_count(), codes.size());
        std::string name = std::string(p).substr(5);
        name[name.find('.')] = '_';
        if (!save(out + "/fonts/" + name + ".png", encode_png(cols * cw, rows * ch, img))) return 1;
    }
    std::printf("jis2uni.dat: %zu JIS codes\n", fd.jis.size());
    return 0;
}

// text-render <font> <utf8 text> <out.png> [scale]
int cmd_text_render(Disc& disc, const Args& a) {
    FontData fd(disc.volume());
    const LatinFont* font = fd.find(a[0]);
    if (!font) {
        std::fprintf(stderr, "unknown font %s\n", a[0].c_str());
        return 1;
    }
    std::string text;
    for (size_t i = 0; i < a[1].size(); ++i) // allow "\n" escapes on the command line
        if (a[1][i] == '\\' && i + 1 < a[1].size() && a[1][i + 1] == 'n') {
            text += '\n';
            ++i;
        } else {
            text += a[1][i];
        }
    LayoutOptions opt;
    opt.scale_x = opt.scale_y = a.size() > 3 ? std::stof(a[3]) : 1.0f;
    auto refs = map_unicode(utf8_decode(text), *font, fd.kanji.get(), &fd.jis);
    TextLayout lay = layout_text(refs, *font, fd.kanji.get(), 8, 8, opt);
    u32 w = u32(std::ceil(lay.width)) + 16, h = u32(std::ceil(lay.height)) + 16;
    std::vector<u32> img(size_t(w) * 2 * h, 0xFF603020u);
    draw_layout(img, w, 2 * h, lay, *font, fd.kanji.get(), FontVariant::Outlined, ~0u);
    TextLayout lay2 = layout_text(refs, *font, fd.kanji.get(), 8, 8 + float(h), opt);
    draw_layout(img, w, 2 * h, lay2, *font, fd.kanji.get(), FontVariant::Plain, 0xFF40C0FFu);
    std::printf("%zu glyphs, %.1f x %.1f px, %d line(s)\n", lay.quads.size(), lay.width, lay.height, lay.lines);
    return save(a[2], encode_png(w, 2 * h, img)) ? 0 : 1;
}

} // namespace

GT3TOOL_COMMAND(text, "text", "<outdir>", "dump string tables (UTF-8) and font atlases (PNG)", 1, cmd_text);
GT3TOOL_COMMAND(text_render, "text-render", "<font> <utf8> <out.png> [scale]",
                "render a string with a game font (outlined and plain variants)", 3, cmd_text_render);

} // namespace gt3r::tool
