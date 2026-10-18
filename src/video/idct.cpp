#include "video/idct.h"

#include <cmath>
#include <numbers>

namespace gt3r::video {

namespace {

constexpr int kConstBits = 13;
constexpr int kPass1Bits = 2;

// round(x * 2^13)
constexpr s32 k0_298631336 = 2446;
constexpr s32 k0_390180644 = 3196;
constexpr s32 k0_541196100 = 4433;
constexpr s32 k0_765366865 = 6270;
constexpr s32 k0_899976223 = 7373;
constexpr s32 k1_175875602 = 9633;
constexpr s32 k1_501321110 = 12299;
constexpr s32 k1_847759065 = 15137;
constexpr s32 k1_961570560 = 16069;
constexpr s32 k2_053119869 = 16819;
constexpr s32 k2_562915447 = 20995;
constexpr s32 k3_072711026 = 25172;

// One 1-D 8-point IDCT on in[0], in[s], ..., in[7s]; results are scaled by
// 2^13 * sqrt(8) relative to the orthonormal transform (i.e. not descaled).
inline void idct_1d(const s32* in, int s, s32 out[8]) {
    // Even part: rotation of coefficients 2 and 6, butterflies with 0 and 4.
    s32 z2 = in[2 * s], z3 = in[6 * s];
    s32 z1 = (z2 + z3) * k0_541196100;
    s32 tmp2 = z1 - z3 * k1_847759065;
    s32 tmp3 = z1 + z2 * k0_765366865;
    s32 tmp0 = (in[0] + in[4 * s]) * (1 << kConstBits);
    s32 tmp1 = (in[0] - in[4 * s]) * (1 << kConstBits);
    s32 e0 = tmp0 + tmp3, e3 = tmp0 - tmp3, e1 = tmp1 + tmp2, e2 = tmp1 - tmp2;

    // Odd part: coefficients 7, 5, 3, 1.
    s32 o0 = in[7 * s], o1 = in[5 * s], o2 = in[3 * s], o3 = in[1 * s];
    s32 a = o0 + o3, b = o1 + o2, c = o0 + o2, d = o1 + o3;
    s32 z5 = (c + d) * k1_175875602;
    o0 *= k0_298631336;
    o1 *= k2_053119869;
    o2 *= k3_072711026;
    o3 *= k1_501321110;
    a *= -k0_899976223;
    b *= -k2_562915447;
    c = c * -k1_961570560 + z5;
    d = d * -k0_390180644 + z5;
    o0 += a + c;
    o1 += b + d;
    o2 += b + c;
    o3 += a + d;

    out[0] = e0 + o3;
    out[7] = e0 - o3;
    out[1] = e1 + o2;
    out[6] = e1 - o2;
    out[2] = e2 + o1;
    out[5] = e2 - o1;
    out[3] = e3 + o0;
    out[4] = e3 - o0;
}

inline s32 descale(s32 x, int n) { return (x + (1 << (n - 1))) >> n; }

} // namespace

void idct_8x8(s16 block[64]) {
    s32 ws[64];
    // Pass 1: rows, keeping kPass1Bits of extra precision.
    for (int r = 0; r < 8; ++r) {
        const s16* row = block + r * 8;
        s32* w = ws + r * 8;
        if ((row[1] | row[2] | row[3] | row[4] | row[5] | row[6] | row[7]) == 0) {
            s32 dc = s32(row[0]) * (1 << kPass1Bits);
            for (int i = 0; i < 8; ++i) w[i] = dc;
            continue;
        }
        s32 in[8], out[8];
        for (int i = 0; i < 8; ++i) in[i] = row[i];
        idct_1d(in, 1, out);
        for (int i = 0; i < 8; ++i) w[i] = descale(out[i], kConstBits - kPass1Bits);
    }
    // Pass 2: columns; remove the extra precision and the factor of 8.
    for (int c = 0; c < 8; ++c) {
        s32 out[8];
        idct_1d(ws + c, 8, out);
        for (int i = 0; i < 8; ++i) block[i * 8 + c] = s16(descale(out[i], kConstBits + kPass1Bits + 3));
    }
}

void idct_8x8_reference(const s16 in[64], double out[64]) {
    static double cosines[8][8];
    static bool init = false;
    if (!init) {
        for (int x = 0; x < 8; ++x)
            for (int u = 0; u < 8; ++u)
                cosines[x][u] = (u == 0 ? std::sqrt(0.125) : 0.5) * std::cos((2 * x + 1) * u * std::numbers::pi / 16.0);
        init = true;
    }
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x) {
            double s = 0;
            for (int v = 0; v < 8; ++v)
                for (int u = 0; u < 8; ++u) s += cosines[x][u] * cosines[y][v] * in[v * 8 + u];
            out[y * 8 + x] = s;
        }
}

} // namespace gt3r::video
