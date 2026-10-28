// Phase 4, the wheels' steer angles; see tick.h.

#include "base/ee_math.h"
#include "sim/exact/tick.h"

namespace gt3r::sim::exact {

F wheel_steer_angle(const Params& p, int i, F delta) {
    F a{};
    if (i < 2) a = delta * p.steer_ratio[size_t(F() <= delta ? i : 1 - i)];
    const F toe = p.toe[size_t(i >> 1)];
    return a + ((i & 1) ? toe : -toe);
}

void wheel_steer(Car& c) {
    for (int i = 0; i < 4; ++i) c.w[size_t(i)].steer = wheel_steer_angle(c.p, i, c.steer_cmd);
    for (size_t i = 0; i < 4; ++i) {
        c.t[i].steer_cos = ee::cos(c.w[i].steer);
        c.t[i].steer_sin = ee::sin(c.w[i].steer);
    }
}

} // namespace gt3r::sim::exact
