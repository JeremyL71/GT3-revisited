#include "base/gzip.h"

#include <zlib.h>

namespace gt3r {

static Bytes inflate_with(ByteSpan src, size_t expected_size, int window_bits) {
    if (expected_size == 0) {
        // Some VOL entries are gzip members of empty files; zlib needs a
        // non-null output buffer, so decode into a scratch byte.
        z_stream zs{};
        if (inflateInit2(&zs, window_bits) != Z_OK) throw FormatError("inflateInit2 failed");
        u8 scratch;
        zs.next_in = const_cast<Bytef*>(src.data());
        zs.avail_in = static_cast<uInt>(src.size());
        zs.next_out = &scratch;
        zs.avail_out = 1;
        int rc = inflate(&zs, Z_FINISH);
        size_t produced = zs.total_out;
        inflateEnd(&zs);
        if (rc != Z_STREAM_END || produced != 0) throw FormatError("inflate failed or size mismatch");
        return {};
    }
    z_stream zs{};
    if (inflateInit2(&zs, window_bits) != Z_OK) throw FormatError("inflateInit2 failed");
    Bytes out(expected_size);
    zs.next_in = const_cast<Bytef*>(src.data());
    zs.avail_in = static_cast<uInt>(src.size());
    zs.next_out = out.data();
    zs.avail_out = static_cast<uInt>(out.size());
    int rc = inflate(&zs, Z_FINISH);
    size_t produced = zs.total_out;
    inflateEnd(&zs);
    if (rc != Z_STREAM_END || produced != expected_size)
        throw FormatError("inflate failed or size mismatch");
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
