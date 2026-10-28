// ee_float (src/base/ee_float.h): the bit-manipulation conversions against a plain frexp/ldexp model of the same rules, on random
// and edge-case operands of every operation (the rules themselves are checked against known PS2 results elsewhere).

#include "base/ee_float.h"
#include "base/ee_math.h"
#include "test_framework.h"

#include <cmath>
#include <random>
#include <vector>

using namespace gt3r;

namespace {

// The model: exact values through libm.
double m_to_double(u32 bits) {
    u32 e = (bits >> 23) & 0xFF;
    if (e == 0) return (bits & 0x80000000u) ? -0.0 : 0.0;
    double v = std::ldexp(double((bits & 0x7FFFFF) | 0x800000), int(e) - 127 - 23);
    return (bits & 0x80000000u) ? -v : v;
}

u32 m_from_double(double v, ee::Round r, ee::Flags* f) {
    u32 sign = std::signbit(v) ? 0x80000000u : 0;
    if (v == 0.0) return sign;
    if (std::isnan(v)) {
        f->invalid = true;
        return ee::kMaxPos;
    }
    if (std::isinf(v)) {
        f->overflow = true;
        return sign | 0x7FFFFFFFu;
    }
    int e;
    double mm = std::ldexp(std::frexp(std::fabs(v), &e), 24);
    mm = r == ee::Round::Chop ? std::trunc(mm) : std::nearbyint(mm);
    if (mm >= 16777216.0) {
        mm /= 2;
        ++e;
    }
    int field = e + 126;
    if (field <= 0) {
        f->underflow = true;
        return sign;
    }
    if (field > 255) {
        f->overflow = true;
        return sign | 0x7FFFFFFFu;
    }
    return sign | (u32(field) << 23) | (u32(mm) & 0x7FFFFF);
}

bool same(const ee::Flags& a, const ee::Flags& b) {
    return a.overflow == b.overflow && a.underflow == b.underflow && a.divzero == b.divzero && a.invalid == b.invalid;
}

// Random operands: any bit pattern, exponents near each other (the adder's alignment), near the ends of the range, zeros.
u32 operand(std::mt19937& rng, u32 near) {
    u32 r = rng();
    switch (rng() % 6) {
    case 0: return r;
    case 1: return (r & 0x807FFFFFu) | (((near >> 23) & 0xFF) + (rng() % 60) - 30) % 256 << 23;
    case 2: return (r & 0x807FFFFFu) | ((rng() % 8) << 23);
    case 3: return (r & 0x807FFFFFu) | ((248 + rng() % 8) << 23);
    case 4: return r & 0x807FFFFFu; // exponent 0
    default: return (r & 0x80000000u) | (near & 0x7FFFFFFFu) | (rng() & 1);
    }
}

} // namespace

TEST(ee_float_conversions_match_model) {
    std::mt19937 rng(7);
    int bad = 0;
    for (int i = 0; i < 400000 && bad < 10; ++i) {
        u32 a = rng();
        double d = m_to_double(a);
        if (ee::to_double(a) != d || std::signbit(ee::to_double(a)) != std::signbit(d)) ++bad;
        // doubles beyond float precision and range: products, quotients and far exponents
        double v = std::ldexp(double(rng()) * 4294967296.0 + double(rng()), int(rng() % 700) - 380 - 64) * ((rng() & 1) ? -1 : 1);
        if (i % 97 == 0) v = std::ldexp(double(rng() | 1u), int(rng() % 40) - 20); // ties and exact values
        for (ee::Round r : {ee::Round::Chop, ee::Round::Nearest}) {
            ee::Flags f1, f2;
            if (ee::from_double(v, r, &f1) != m_from_double(v, r, &f2) || !same(f1, f2)) ++bad;
        }
    }
    for (double v : {0.0, -0.0, HUGE_VAL, -HUGE_VAL, std::nan(""), 5e-324, -5e-324}) {
        for (ee::Round r : {ee::Round::Chop, ee::Round::Nearest}) {
            ee::Flags f1, f2;
            if (ee::from_double(v, r, &f1) != m_from_double(v, r, &f2) || !same(f1, f2)) ++bad;
        }
    }
    CHECK(bad == 0);
}

TEST(ee_float_operations_match_model) {
    std::mt19937 rng(11);
    int bad = 0;
    auto add_model = [](u32 a, u32 b, ee::Round r, ee::Flags* f) {
        auto dz = [](u32 x) { return ((x >> 23) & 0xFF) ? x : (x & 0x80000000u); };
        a = dz(a);
        b = dz(b);
        int d = int((a >> 23) & 0xFF) - int((b >> 23) & 0xFF);
        if (d >= 25) b &= 0x80000000u;
        else if (d > 1) b &= 0xFFFFFFFFu << (d - 1);
        else if (d <= -25) a &= 0x80000000u;
        else if (d < -1) a &= 0xFFFFFFFFu << (-d - 1);
        double s = m_to_double(a) + m_to_double(b);
        if (s == 0.0) return (a & b & 0x80000000u) ? 0x80000000u : 0u;
        return m_from_double(s, r, f);
    };
    for (int i = 0; i < 400000 && bad < 10; ++i) {
        u32 a = operand(rng, rng()), b = operand(rng, a);
        ee::Flags f1, f2;
        if (ee::add(a, b, ee::Round::Chop, &f1) != add_model(a, b, ee::Round::Chop, &f2) || !same(f1, f2)) ++bad;
        if (ee::add(a, b, ee::Round::Nearest) != add_model(a, b, ee::Round::Nearest, &f2)) ++bad;
        if (ee::sub(a, b) != add_model(a, b ^ 0x80000000u, ee::Round::Chop, &f2)) ++bad;
        double p = m_to_double(a) * m_to_double(b);
        u32 pm = p == 0.0 ? (a ^ b) & 0x80000000u : m_from_double(p, ee::Round::Chop, &f2);
        if (ee::mul(a, b) != pm) ++bad;
        if (((b >> 23) & 0xFF) != 0) {
            double q = m_to_double(a) / m_to_double(b);
            u32 qm = q == 0.0 ? (a ^ b) & 0x80000000u : m_from_double(q, ee::Round::Nearest, &f2);
            if (ee::div(a, b) != qm) ++bad;
        }
        double sq = std::sqrt(std::fabs(m_to_double(a)));
        if (ee::sqrt(a) != (sq == 0.0 ? 0u : m_from_double(sq, ee::Round::Nearest, &f2))) ++bad;
        double ma = m_to_double(a), mb = m_to_double(b);
        if (ee::compare(a, b) != (ma < mb ? -1 : ma > mb ? 1 : 0)) ++bad;
        double t = std::trunc(ma);
        s32 ti = t >= 2147483647.0 ? 0x7FFFFFFF : t <= -2147483648.0 ? s32(0x80000000u) : s32(t);
        if (ee::to_int(a) != ti) ++bad;
        s32 iv = s32(rng());
        if (ee::from_int(iv) != m_from_double(double(iv), ee::Round::Chop, &f2)) ++bad;
    }
    CHECK(bad == 0);
}

TEST(ee_float_fast_paths_match) {
    // The inline operators of ee::F against the functions they stand for.
    std::mt19937 rng(23);
    int bad = 0;
    for (int i = 0; i < 400000 && bad < 10; ++i) {
        u32 a = operand(rng, rng()), b = operand(rng, a);
        ee::F x = ee::F::raw(a), y = ee::F::raw(b);
        if ((x + y).bits != ee::add(a, b) || (x - y).bits != ee::sub(a, b) || (x * y).bits != ee::mul(a, b)) ++bad;
        int c = ee::compare(a, b);
        if ((x < y) != (c < 0) || (x <= y) != (c <= 0) || (x == y) != (c == 0) || (x > y) != (c > 0) || (x >= y) != (c >= 0)) ++bad;
        if (ee::max(x, y).bits != ee::max(a, b) || ee::min(x, y).bits != ee::min(a, b)) ++bad;
        if ((x / y).bits != ee::div(a, b) || ee::sqrt(x).bits != ee::sqrt(a)) ++bad;
        u32 z = rng();
        if (ee::madd(ee::F::raw(z), x, y).bits != ee::madd(z, a, b) || ee::msub(ee::F::raw(z), x, y).bits != ee::msub(z, a, b)) ++bad;
        // float to int on any operand, int to float on any int and on small ones
        const s32 n = s32(rng()) >> (rng() % 32);
        if (ee::to_int(x) != ee::to_int(a) || ee::int_to_f(n).bits != ee::from_int(n, ee::Round::Chop)) ++bad;
    }
    for (s32 n : {0, 1, -1, 0x7FFFFFFF, s32(0x80000000u), 0xFFFFFF, 0x1000000, 0x1000001, -0x1000001})
        if (ee::int_to_f(n).bits != ee::from_int(n, ee::Round::Chop)) ++bad;
    for (u32 a : {0x4EFFFFFFu, 0x4F000000u, 0xCF000000u, 0xCEFFFFFFu, 0x3F7FFFFFu, 0xBF800000u, 0x7FFFFFFFu, 0xFFFFFFFFu, 0x80000000u})
        if (ee::to_int(ee::F::raw(a)) != ee::to_int(a)) ++bad;
    CHECK(bad == 0);
}

// ee::side_sign never claims a sign the chopped arithmetic does not give: random side tests at every scale, and tests made to
// land next to zero (a point by a line through two others, the products' exact values a few ulps apart or equal).
TEST(ee_float_side_sign_is_certain) {
    std::mt19937 rng(5);
    int wrong = 0;
    long decided = 0, random = 0;
    auto value = [&](int scale) { // a course-like coordinate or difference: |x| around 2^scale, either sign, sometimes zero
        if (rng() % 16 == 0) return ee::F::raw(0);
        const u32 e = u32(127 + scale + int(rng() % 9) - 4);
        return ee::F::raw((rng() & 0x807FFFFFu) | (e << 23));
    };
    auto check = [&](ee::F a, ee::F b, ee::F c, ee::F d, ee::F e, ee::F f) {
        const ee::F s = (a * (b - c)) - (d * (e - f));
        const int k = ee::side_sign(a, b, c, d, e, f);
        if ((k > 0 && !(ee::F::raw(0) <= s)) || (k < 0 && !(s <= ee::F::raw(0)))) ++wrong;
        return k != 0;
    };
    for (int i = 0; i < 400000; ++i) {
        const int sa = int(rng() % 24) - 12, sb = int(rng() % 24) - 4;
        ee::F a = value(sa), b = value(sb), c = value(sb), d = value(sa), e = value(sb), f = value(sb);
        decided += check(a, b, c, d, e, f), ++random;
        // next to zero: e - f chosen so d (e - f) is a (b - c) give or take a few ulps (f = e - a (b - c) / d)
        if (!(d == ee::F::raw(0))) {
            const ee::F t = (a * (b - c)) / d;
            ee::F f2 = e - t;
            for (int k = -3; k <= 3; ++k) check(a, b, c, d, e, ee::F::raw(f2.bits + u32(k)));
        }
        // collinear: b - c = (e - f) d / a exactly in the reals for small integers
        const s32 m = s32(rng() % 2001) - 1000, n = s32(rng() % 2001) - 1000, p = s32(rng() % 41) - 20;
        check(ee::int_to_f(p), ee::int_to_f(m), ee::int_to_f(n), ee::int_to_f(p), ee::int_to_f(m), ee::int_to_f(n));
    }
    CHECK(wrong == 0);
    CHECK(decided > random * 9 / 10); // (random tests are nearly all decided: the skip is worth having)
}

// The memo of sinf / cosf results (ee_math.cpp) gives what the routines give: arguments that share its entries, again and in
// turn, and every one the platform hash below takes.
TEST(ee_math_memo_matches) {
    std::mt19937 rng(11);
    int bad = 0;
    std::vector<ee::F> args;
    for (int i = 0; i < 4000; ++i) args.push_back(ee::F::raw((rng() & 0x807FFFFFu) | ((100 + rng() % 40) << 23)));
    args.push_back(ee::F::raw(0)), args.push_back(ee::F::raw(1)), args.push_back(ee::F::raw(0x80000000u));
    for (int pass = 0; pass < 3; ++pass)
        for (size_t i = 0; i < args.size(); ++i) {
            const ee::F t = args[pass == 1 ? args.size() - 1 - i : (i * 7919) % args.size()];
            if (ee::sin(t).bits != ee::sin_uncached(t).bits || ee::cos(t).bits != ee::cos_uncached(t).bits) ++bad;
        }
    CHECK(bad == 0);
}

// The same bits on every platform and compiler setting: a hash of ee::F and ee_math results over fixed random operands
// (checked equal on arm64 and x86-64, with and without FMA contraction). A failure here means a platform's build changes PS2
// float results, which would make replays and online races differ between platforms.
TEST(ee_float_platform_hash) {
    std::mt19937 rng(7);
    u64 h[4] = {1469598103934665603ull, 1469598103934665603ull, 1469598103934665603ull, 1469598103934665603ull};
    auto mix = [&](int k, u32 v) { h[k] = (h[k] ^ v) * 1099511628211ull; };
    for (int i = 0; i < 200000; ++i) {
        const u32 a = operand(rng, rng()), b = operand(rng, a), c = operand(rng, b);
        const ee::F x = ee::F::raw(a), y = ee::F::raw(b), z = ee::F::raw(c);
        mix(0, (x + y).bits ^ (x - z).bits);
        mix(1, (x * y).bits ^ ee::madd(z, x, y).bits);
        mix(2, (x / y).bits ^ ee::sqrt(x).bits);
        if (i % 8 == 0) {
            ee::F s, co;
            const ee::F t = ee::F::raw((a & 0x807FFFFFu) | ((110 + (a >> 9) % 30) << 23)); // |t| up to 2^12
            ee::sincos(t, s, co);
            mix(3, ee::sin(t).bits ^ ee::cos(t).bits ^ s.bits ^ co.bits ^ ee::atan2(x, y).bits ^ ee::atan(z).bits);
        }
    }
    CHECK(h[0] == 0x534ba27d4f41d126ull);
    CHECK(h[1] == 0x8f92a08065f8d753ull);
    CHECK(h[2] == 0x1e7698169f3a6065ull);
    CHECK(h[3] == 0xadfcb3d4d26d08caull);
}

