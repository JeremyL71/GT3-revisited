#include "formats/png_writer.h"

#include "base/gzip.h"

#include <zlib.h>

namespace gt3r {

namespace {

void put32be(Bytes& b, u32 v) {
    b.push_back(u8(v >> 24));
    b.push_back(u8(v >> 16));
    b.push_back(u8(v >> 8));
    b.push_back(u8(v));
}
void put32le(Bytes& b, u32 v) {
    for (int i = 0; i < 4; ++i) b.push_back(u8(v >> (8 * i)));
}
void put16le(Bytes& b, u16 v) {
    b.push_back(u8(v));
    b.push_back(u8(v >> 8));
}

void chunk(Bytes& out, const char* type, const Bytes& data) {
    put32be(out, u32(data.size()));
    size_t start = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    put32be(out, u32(crc32(0, out.data() + start, uInt(out.size() - start))));
}

} // namespace

Bytes encode_png(u32 width, u32 height, const std::vector<u32>& rgba) {
    Bytes raw;
    raw.reserve(size_t(height) * (width * 4 + 1));
    for (u32 y = 0; y < height; ++y) {
        raw.push_back(0); // filter: none
        const u8* row = reinterpret_cast<const u8*>(rgba.data() + size_t(y) * width);
        raw.insert(raw.end(), row, row + size_t(width) * 4);
    }
    Bytes out = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    Bytes ihdr;
    put32be(ihdr, width);
    put32be(ihdr, height);
    ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0}); // 8-bit RGBA
    chunk(out, "IHDR", ihdr);
    chunk(out, "IDAT", deflate_zlib(raw));
    chunk(out, "IEND", {});
    return out;
}

Bytes encode_wav(u32 sample_rate, u32 channels, const std::vector<s16>& pcm) {
    Bytes out;
    u32 data_bytes = u32(pcm.size() * 2);
    out.insert(out.end(), {'R', 'I', 'F', 'F'});
    put32le(out, 36 + data_bytes);
    out.insert(out.end(), {'W', 'A', 'V', 'E', 'f', 'm', 't', ' '});
    put32le(out, 16);
    put16le(out, 1);
    put16le(out, u16(channels));
    put32le(out, sample_rate);
    put32le(out, sample_rate * channels * 2);
    put16le(out, u16(channels * 2));
    put16le(out, 16);
    out.insert(out.end(), {'d', 'a', 't', 'a'});
    put32le(out, data_bytes);
    const u8* p = reinterpret_cast<const u8*>(pcm.data());
    out.insert(out.end(), p, p + data_bytes);
    return out;
}

} // namespace gt3r
