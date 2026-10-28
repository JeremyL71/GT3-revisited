// PS2 single-precision arithmetic; see ee_float.h.

#include "base/ee_float.h"

#include <cfloat>
#include <cmath>
#include <cstring>

namespace gt3r::ee {

namespace {
inline u64 dbits(double d) {
    u64 b;
    std::memcpy(&b, &d, 8);
    return b;
}
inline double from_dbits(u64 b) {
    double d;
    std::memcpy(&d, &b, 8);
    return d;
}
} // namespace

// Bit manipulation, not libm: these run once or twice per emulated FPU operation.
double to_double(u32 bits) {
    u32 e = (bits >> 23) & 0xFF;
    u64 sign = u64(bits & 0x80000000u) << 32;
    if (e == 0) return from_dbits(sign);
    return from_dbits(sign | (u64(e + (1023 - 127)) << 52) | (u64(bits & 0x7FFFFF) << 29));
}

u32 from_double(double v, Round r, Flags* f) {
    u64 b = dbits(v);
    u32 sign = u32(b >> 32) & 0x80000000u;
    int de = int((b >> 52) & 0x7FF);
    if (de == 0x7FF) {
        if (b & 0xFFFFFFFFFFFFFull) { // NaN
            if (f) f->invalid = true;
            return kMaxPos;
        }
        if (f) f->overflow = true;
        return sign | 0x7FFFFFFFu;
    }
    if (de == 0) { // zero, or a double denormal: far below the smallest PS2 normal
        if ((b << 1) && f) f->underflow = true;
        return sign;
    }
    u64 m = b & 0xFFFFFFFFFFFFFull;
    u32 top = u32(m >> 29);
    if (r == Round::Nearest) { // ties to even
        u64 rest = m & ((1ull << 29) - 1), half = 1ull << 28;
        if (rest > half || (rest == half && (top & 1))) {
            if (++top == 0x800000u) {
                top = 0;
                ++de;
            }
        }
    }
    int field = de - 1023 + 127;
    if (field <= 0) {
        if (f) f->underflow = true;
        return sign;
    }
    if (field > 255) {
        if (f) f->overflow = true;
        return sign | 0x7FFFFFFFu;
    }
    return sign | (u32(field) << 23) | top;
}

u32 from_host(float v) {
    u32 b;
    std::memcpy(&b, &v, 4);
    u32 e = (b >> 23) & 0xFF;
    if (e == 0) return b & 0x80000000u;
    if (e == 255) return (b & 0x7FFFFF) ? kMaxPos : (b & 0x80000000u) | 0x7FFFFFFFu;
    return b;
}

float to_host(u32 bits) {
    u32 e = (bits >> 23) & 0xFF;
    if (e == 0) return (bits & 0x80000000u) ? -0.0f : 0.0f;
    if (e == 255) return (bits & 0x80000000u) ? -FLT_MAX : FLT_MAX;
    float v;
    std::memcpy(&v, &bits, 4);
    return v;
}

// Drops the low bits of the smaller operand like the PS2 adder's alignment
// (one guard bit, no sticky bit). Operands with exponent 0 are already zero.
static void align_operands(u32& a, u32& b) {
    int ea = int((a >> 23) & 0xFF), eb = int((b >> 23) & 0xFF);
    int d = ea - eb;
    if (d >= 25) b &= 0x80000000u;
    else if (d > 1) b &= 0xFFFFFFFFu << (d - 1);
    else if (d <= -25) a &= 0x80000000u;
    else if (d < -1) a &= 0xFFFFFFFFu << (-d - 1);
}

static u32 zero_if_denormal(u32 a) { return ((a >> 23) & 0xFF) ? a : (a & 0x80000000u); }

bool g_nearest_override = false;

u32 add(u32 a, u32 b, Round r, Flags* f) {
    if (g_nearest_override) r = Round::Nearest;
    a = zero_if_denormal(a);
    b = zero_if_denormal(b);
    align_operands(a, b);
    double s = to_double(a) + to_double(b); // exact after alignment
    if (s == 0.0) {
        // IEEE: x + (-x) = +0 (round toward zero too); -0 + -0 = -0.
        bool neg = (a & b & 0x80000000u) != 0;
        return neg ? 0x80000000u : 0;
    }
    return from_double(s, r, f);
}

u32 sub(u32 a, u32 b, Round r, Flags* f) { return add(a, neg(b), r, f); }

u32 mul(u32 a, u32 b, Round r, Flags* f) {
    if (g_nearest_override) r = Round::Nearest;
    u32 sign = (a ^ b) & 0x80000000u;
    double p = to_double(a) * to_double(b); // exact: 48-bit product
    if (p == 0.0) return sign;
    return from_double(p, r, f);
}

u32 div(u32 a, u32 b, Round r, Flags* f) {
    u32 sign = (a ^ b) & 0x80000000u;
    if (((b >> 23) & 0xFF) == 0) {
        if (f) {
            if (((a >> 23) & 0xFF) == 0) f->invalid = true;
            else f->divzero = true;
        }
        return sign | 0x7FFFFFFFu;
    }
    double q = to_double(a) / to_double(b);
    if (q == 0.0) return sign;
    return from_double(q, r, f);
}

u32 sqrt(u32 a, Round r, Flags* f) {
    if (a & 0x80000000u) {
        if (f && ((a >> 23) & 0xFF)) f->invalid = true;
    }
    double v = std::sqrt(std::fabs(to_double(a)));
    if (v == 0.0) return 0;
    return from_double(v, r, f);
}

u32 rsqrt(u32 a, u32 b, Round r, Flags* f) {
    double d = std::sqrt(std::fabs(to_double(b)));
    if (d == 0.0) {
        if (f) f->divzero = true;
        return (a & 0x80000000u) | 0x7FFFFFFFu;
    }
    if ((b & 0x80000000u) && f) f->invalid = true;
    double q = to_double(a) / d;
    if (q == 0.0) return a & 0x80000000u;
    return from_double(q, r, f);
}

int compare(u32 a, u32 b) {
    double x = to_double(a), y = to_double(b);
    return x < y ? -1 : (x > y ? 1 : 0);
}

u32 max(u32 a, u32 b) { return compare(a, b) >= 0 ? a : b; }
u32 min(u32 a, u32 b) { return compare(a, b) <= 0 ? a : b; }

s32 to_int(u32 a) {
    double v = std::trunc(to_double(a));
    if (v >= 2147483647.0) return 0x7FFFFFFF;
    if (v <= -2147483648.0) return s32(0x80000000u);
    return s32(v);
}

u32 from_int(s32 v, Round r) { return from_double(double(v), r); }

s32 to_fixed(u32 a, int frac_bits) {
    double v = std::trunc(std::ldexp(to_double(a), frac_bits));
    if (v >= 2147483647.0) return 0x7FFFFFFF;
    if (v <= -2147483648.0) return s32(0x80000000u);
    return s32(v);
}

u32 from_fixed(s32 v, int frac_bits) { return from_double(std::ldexp(double(v), -frac_bits), Round::Chop); }

} // namespace gt3r::ee
