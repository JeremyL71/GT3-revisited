#include "formats/png_reader.h"

#include "base/gzip.h"

#include <cstdlib>
#include <cstring>

namespace gt3r {

namespace {
u32 be32(ByteSpan d, size_t o) { return u32(d[o]) << 24 | u32(d[o + 1]) << 16 | u32(d[o + 2]) << 8 | d[o + 3]; }

u8 paeth(int a, int b, int c) {
    int p = a + b - c, pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
    return u8(pa <= pb && pa <= pc ? a : pb <= pc ? b : c);
}
} // namespace

PngImage decode_png(ByteSpan d, u32 max_dim) {
    static const u8 kSig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (d.size() < 33 || std::memcmp(d.data(), kSig, 8) != 0) throw FormatError("PNG: bad signature");
    u32 w = 0, h = 0;
    int depth = 0, ctype = 0, interlace = 0;
    Bytes idat, palette;
    std::vector<u8> trns;
    for (size_t o = 8; o + 12 <= d.size();) {
        u32 len = be32(d, o);
        if (o + 12 + len > d.size()) throw FormatError("PNG: truncated chunk");
        const char* type = reinterpret_cast<const char*>(d.data() + o + 4);
        ByteSpan body = d.subspan(o + 8, len);
        if (!std::memcmp(type, "IHDR", 4) && len >= 13) {
            w = be32(body, 0), h = be32(body, 4);
            depth = body[8], ctype = body[9], interlace = body[12];
        } else if (!std::memcmp(type, "PLTE", 4)) {
            palette.assign(body.begin(), body.end());
        } else if (!std::memcmp(type, "tRNS", 4)) {
            trns.assign(body.begin(), body.end());
        } else if (!std::memcmp(type, "IDAT", 4)) {
            idat.insert(idat.end(), body.begin(), body.end());
        } else if (!std::memcmp(type, "IEND", 4)) {
            break;
        }
        o += 12 + len;
    }
    if (!w || !h || w > max_dim || h > max_dim) throw FormatError("PNG: bad size");
    if (interlace) throw FormatError("PNG: interlaced images are not supported");
    int channels = ctype == 0 ? 1 : ctype == 2 ? 3 : ctype == 3 ? 1 : ctype == 4 ? 2 : ctype == 6 ? 4 : 0;
    if (!channels) throw FormatError("PNG: unknown colour type");
    if (depth != 8 && !(ctype == 3 && (depth == 1 || depth == 2 || depth == 4)))
        throw FormatError("PNG: only 8-bit images (and 1/2/4-bit palettes) are supported");
    if (idat.size() < 2) throw FormatError("PNG: no image data");
    const size_t stride = (size_t(w) * channels * depth + 7) / 8, bpp = std::max<size_t>(1, size_t(channels) * depth / 8);
    Bytes raw = inflate_raw(ByteSpan(idat).subspan(2), (stride + 1) * h); // skip the zlib header
    Bytes prev(stride, 0), cur(stride);
    PngImage img;
    img.width = w, img.height = h;
    img.rgba.resize(size_t(w) * h);
    for (u32 y = 0; y < h; ++y) {
        const u8* line = raw.data() + y * (stride + 1);
        u8 filter = line[0];
        for (size_t x = 0; x < stride; ++x) {
            int a = x >= bpp ? cur[x - bpp] : 0, b = prev[x], c = x >= bpp ? prev[x - bpp] : 0, v = line[1 + x];
            switch (filter) {
            case 0: break;
            case 1: v += a; break;
            case 2: v += b; break;
            case 3: v += (a + b) / 2; break;
            case 4: v += paeth(a, b, c); break;
            default: throw FormatError("PNG: bad filter");
            }
            cur[x] = u8(v);
        }
        for (u32 x = 0; x < w; ++x) {
            u8 r, g, b, al = 255;
            if (ctype == 3) {
                int bits = depth, per = 8 / bits;
                u8 byte = cur[x / per];
                int idx = (byte >> ((per - 1 - int(x % per)) * bits)) & ((1 << bits) - 1);
                if (size_t(idx) * 3 + 2 >= palette.size()) throw FormatError("PNG: palette index out of range");
                r = palette[idx * 3], g = palette[idx * 3 + 1], b = palette[idx * 3 + 2];
                if (size_t(idx) < trns.size()) al = trns[size_t(idx)];
            } else {
                const u8* p = cur.data() + size_t(x) * channels;
                if (channels <= 2) r = g = b = p[0];
                else r = p[0], g = p[1], b = p[2];
                if (channels == 2) al = p[1];
                if (channels == 4) al = p[3];
            }
            img.rgba[size_t(y) * w + x] = u32(r) | u32(g) << 8 | u32(b) << 16 | u32(al) << 24;
        }
        std::swap(prev, cur);
    }
    return img;
}

std::optional<std::pair<u32, u32>> png_size(ByteSpan d) {
    static const u8 kPng[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (d.size() < 24 || std::memcmp(d.data(), kPng, 8) != 0 || std::memcmp(d.data() + 12, "IHDR", 4) != 0) return std::nullopt;
    auto be = [&](size_t o) { return u32(d[o]) << 24 | u32(d[o + 1]) << 16 | u32(d[o + 2]) << 8 | d[o + 3]; };
    return std::make_pair(be(16), be(20));
}

} // namespace gt3r
