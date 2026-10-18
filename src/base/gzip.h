#pragma once

#include "base/bytes.h"

namespace gt3r {

// Raw DEFLATE stream (no zlib/gzip header). Used by CORE.GT3.
Bytes inflate_raw(ByteSpan src, size_t expected_size);

// gzip member (RFC 1952). Used by compressed GT3.VOL entries.
Bytes inflate_gzip(ByteSpan src, size_t expected_size);

// zlib-wrapped deflate, for writing PNGs.
Bytes deflate_zlib(ByteSpan src);

} // namespace gt3r
