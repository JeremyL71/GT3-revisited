#include "base/gzip.h"

#include <zlib.h>

#include <algorithm>

namespace gt3r {

// `expected_size` comes from the data, so the output grows as the stream fills it rather than being allocated up front:
// a wrong size costs no more memory than the stream really holds.
static Bytes inflate_with(ByteSpan src, size_t expected_size, int window_bits) {
    // One byte past the expected size: a stream that runs longer fills it, and an empty file (some VOL entries are gzip
    // members of empty files) still gets the non-null buffer zlib needs.
    Bytes out(std::min(expected_size, src.size() * 4) + 1);
    z_stream zs{};
    if (inflateInit2(&zs, window_bits) != Z_OK) throw FormatError("inflateInit2 failed");
    zs.next_in = const_cast<Bytef*>(src.data());
    zs.avail_in = static_cast<uInt>(src.size());
    size_t produced = 0;
    int rc;
    for (;;) {
        zs.next_out = out.data() + produced;
        zs.avail_out = static_cast<uInt>(out.size() - produced);
        rc = inflate(&zs, Z_FINISH); // Z_BUF_ERROR: out of input, or (buffer full) out of room
        produced = size_t(zs.next_out - out.data());
        if (rc != Z_BUF_ERROR || produced < out.size() || out.size() > expected_size) break;
        out.resize(std::min(expected_size + 1, out.size() * 2));
    }
    inflateEnd(&zs);
    if (rc != Z_STREAM_END || produced != expected_size) throw FormatError("inflate failed or size mismatch");
    out.resize(expected_size);
    return out;
}

Bytes inflate_raw(ByteSpan src, size_t expected_size) { return inflate_with(src, expected_size, -MAX_WBITS); }

Bytes inflate_gzip(ByteSpan src, size_t expected_size) { return inflate_with(src, expected_size, MAX_WBITS + 16); }

Bytes deflate_zlib(ByteSpan src) {
    uLongf len = compressBound(static_cast<uLong>(src.size()));
    Bytes out(len);
    if (compress2(out.data(), &len, src.data(), static_cast<uLong>(src.size()), 6) != Z_OK)
        throw FormatError("deflate failed");
    out.resize(len);
    return out;
}

} // namespace gt3r
