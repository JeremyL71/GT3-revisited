#pragma once

#include "base/bytes.h"

namespace gt3r::video {

// MSB-first bit reader for MPEG video syntax. The buffer must be followed by
// at least 8 readable padding bytes (the decoder copies each start-code unit
// into a zero-padded buffer), so peeks never need a bounds check; reading
// past `size` just returns zeros, which ends slices cleanly.
class BitReader {
public:
    BitReader(const u8* data, size_t size) : p_(data), bits_(u64(size) * 8) {}

    // Peeks 1..32 bits.
    u32 peek(int n) const {
        const u8* q = p_ + (pos_ >> 3);
        u64 w = (u64(q[0]) << 56) | (u64(q[1]) << 48) | (u64(q[2]) << 40) | (u64(q[3]) << 32) |
                (u64(q[4]) << 24) | (u64(q[5]) << 16) | (u64(q[6]) << 8) | u64(q[7]);
        return u32((w << (pos_ & 7)) >> (64 - n));
    }
    void skip(int n) { pos_ += u64(n); }
    u32 get(int n) {
        u32 v = peek(n);
        pos_ += u64(n);
        return v;
    }
    bool get1() { return get(1) != 0; }

    u64 position() const { return pos_; }
    u64 size_bits() const { return bits_; }
    bool exhausted() const { return pos_ >= bits_; }
    s64 bits_left() const { return s64(bits_) - s64(pos_); }

private:
    const u8* p_;
    u64 bits_;
    u64 pos_ = 0;
};

} // namespace gt3r::video
