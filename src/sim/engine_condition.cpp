#include "sim/engine_condition.h"

#include <algorithm>

namespace gt3r::sim {

float engine_condition_factor(float E, float O) {
    const float E1 = -300000, c1 = 0.03f, E2 = 500000, E3 = 600000, c2 = 0.02f, O1 = -100000, cO = 0.05f;
    float fe = E < E1 ? 1 - c1 : E < 0 ? 1 - c1 * E / E1 : E <= E2 ? 1.0f : E < E3 ? 1 - c2 * (E - E2) / (E3 - E2) : 1 - c2;
    float fo = O < O1 ? 1 + cO : O < 0 ? 1 + cO * O / O1 : 1.0f;
    return fe * fo;
}

void engine_condition_step(float& E, float& O, float d) {
    E = std::min(E + d, 600000.0f);
    if (O < 0) {
        float o = O + d;
        O = o >= 0 ? 300000.0f : o;
    } else {
        O = std::min(O + d, 300000.0f);
    }
}

float odometer_step(u32& odometer, float x) {
    if (x <= 0) return 0;
    const s32 n = s32(x) / 2; // truncated metres, halved towards zero
    const u32 w = odometer + u32(n);
    if (w >= kOdometerMax) {
        odometer = kOdometerMax;
        return 0;
    }
    odometer = w;
    return x - float(2 * n);
}

bool oil_worn(float O) { return O >= 300000.0f; }

} // namespace gt3r::sim
