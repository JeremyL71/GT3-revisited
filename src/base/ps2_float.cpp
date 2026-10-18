#include "base/ps2_float.h"

#include "base/bytes.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::ps2 {

namespace {

u32 bits(float f) {
    u32 b;
    std::memcpy(&b, &f, 4);
    return b;
}
float from_bits(u32 b) {
    float f;
    std::memcpy(&f, &b, 4);
    return f;
}
// Value of a PS2 float (exponent 0 reads as zero, exponent 255 is ordinary).
double value(float f) {
    u32 b = bits(f);
    int e = int((b >> 23) & 0xFF);
    if (e == 0) return (b & 0x80000000u) ? -0.0 : 0.0;
    double v = std::ldexp(double((b & 0x7FFFFFu) | 0x800000u), e - 127 - 23);
    return (b & 0x80000000u) ? -v : v;
}
// Double -> PS2 float, mantissa truncated.
float chop(double v) {
    u32 sign = std::signbit(v) ? 0x80000000u : 0;
    if (v == 0) return from_bits(sign);
    int e;
    double m = std::frexp(std::fabs(v), &e);
    u64 mm = u64(std::trunc(std::ldexp(m, 24)));
    int field = e + 126;
    if (field <= 0) return from_bits(sign);
    if (field > 255) return from_bits(sign | 0x7FFFFFFFu);
    return from_bits(sign | u32(field) << 23 | (u32(mm) & 0x7FFFFFu));
}

} // namespace

float mul(float a, float b) {
    double p = value(a) * value(b);
    return p == 0 ? from_bits((bits(a) ^ bits(b)) & 0x80000000u) : chop(p);
}

float add(float a, float b) {
    u32 x = bits(a), y = bits(b);
    if (((x >> 23) & 0xFF) == 0) x &= 0x80000000u;
    if (((y >> 23) & 0xFF) == 0) y &= 0x80000000u;
    int d = int((x >> 23) & 0xFF) - int((y >> 23) & 0xFF);
    if (d >= 25) y &= 0x80000000u;
    else if (d > 1) y &= 0xFFFFFFFFu << (d - 1);
    else if (d <= -25) x &= 0x80000000u;
    else if (d < -1) x &= 0xFFFFFFFFu << (-d - 1);
    double s = value(from_bits(x)) + value(from_bits(y));
    return s == 0 ? 0.0f : chop(s);
}

int to_int(float f) {
    double v = std::trunc(value(f));
    return int(std::clamp(v, -2147483648.0, 2147483647.0));
}

} // namespace gt3r::ps2
