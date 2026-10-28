// The tick's bookkeeping phases; see tick.h.

#include "sim/exact/tick.h"

namespace gt3r::sim::exact {

void save_previous(Car& c) {
    Previous& p = c.prev;
    p.pitch = c.pitch, p.roll = c.roll, p.heading = c.heading;
    p.ground_pitch = c.ground_pitch, p.ground_roll = c.ground_roll;
    for (size_t i = 0; i < 4; ++i) {
        const Wheel& w = c.w[i];
        p.susp[i] = w.susp;
        p.steer[i] = w.steer;
        p.camber[i] = w.camber;
        p.half_track[i] = w.half_track;
        p.spin[i] = w.spin;
        p.brake_heat[i] = w.brake_heat;
        p.attr_a[i] = c.g[i].attr_a;
        p.attr_b[i] = c.g[i].attr_b;
        p.hub[i] = {c.g[i].x, w.ground, c.g[i].z};
        p.flap[i] = w.flap;
    }
    p.lag_heading_diff = c.lag_heading_diff;
    p.lag_pitch_diff = c.lag_pitch_diff;
    p.f3e4 = c.f3e4;
}

void clear_feedback(Car& c) { c.feedback = 0; }

} // namespace gt3r::sim::exact
