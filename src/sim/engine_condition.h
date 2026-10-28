#pragma once

// [physics §10.3] The engine condition's rules outside a race: GT mode's garage cars carry the odometer, the running-in /
// wear counter E and the oil counter O (sim::EngineCondition) from run to run; the race itself advances them in the car's
// record (the exact core).

#include "sim/sim_types.h"

namespace gt3r::sim {

// Engine-condition torque factor f_E(E) x f_O(O) from the running-in/wear counter E and the oil counter O (metres; zero
// counters give exactly 1). E1 = -300000, c1 = 0.03, E2 = 500000, E3 = 600000, c2 = 0.02, O1 = -100000, cO = 0.05.
float engine_condition_factor(float E, float O);
// Counters after driving d metres: E = min(E + d, 600000); O < 0: O + d, jumping to 300000 when it reaches 0; O >= 0:
// min(O + d, 300000).
void engine_condition_step(float& E, float& O, float d);
// Odometer step: x = carry + d metres; the odometer (2 m units) gains trunc(trunc(x) / 2) and saturates at kOdometerMax
// (carry 0 then); returns the new carry x - 2 * gained (0 when x <= 0).
constexpr u32 kOdometerMax = 0xEE6B27A6u;
float odometer_step(u32& odometer, float x);
// The race HUD's oil lamp: the oil counter at its worn value (O >= 300000).
bool oil_worn(float O);

} // namespace gt3r::sim
