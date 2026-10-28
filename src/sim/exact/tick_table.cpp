// The table lookups and the tire record index; see tick.h.

#include "sim/exact/tick.h"

namespace gt3r::sim::exact {

F lookup(s16 n, const F* x, const F* y, F v) {
    const u32 last = u32(s32(n) - 1);
    if (v <= x[0]) return y[0];
    if (x[last] <= v) return y[last];
    u32 k = 1; // the first point above v (the last when none)
    while (k < last && !(v < x[k])) ++k;
    return y[k - 1] + (v - x[k - 1]) * (y[k] - y[k - 1]) / (x[k] - x[k - 1]);
}

F lookup_bisect(s16 n, const F* x, const F* y, F v) {
    const u32 last = u32(s32(n) - 1);
    if (v <= x[0]) return y[0];
    if (x[last] <= v) return y[last];
    u32 lo = 0, hi = last;
    if (!(hi < 4)) do {
            const u32 mid = (hi + lo) >> 1;
            if (v < x[mid]) hi = mid;
            else if (v == x[mid]) return y[mid];
            else lo = mid;
        } while (!(hi - lo < 4));
    u32 k = lo;
    while (k < hi && !(v < x[k])) ++k;
    return y[k - 1] + (y[k] - y[k - 1]) * (v - x[k - 1]) / (x[k] - x[k - 1]);
}

int tire_record_index(const Params& p, int wheel, u8 surface) {
    const size_t k = size_t(16 * (wheel >> 1)) + surface; // the byte P + 0xDD0 + 16 axle + surface
    return p.tire_index[k >> 4][k & 15];
}

const TireRecord& tire_record(const Params& p, int wheel, u8 surface) {
    return p.tire[size_t(tire_record_index(p, wheel, surface))];
}

} // namespace gt3r::sim::exact
