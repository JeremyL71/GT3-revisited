#pragma once

// Electric motor sound for mod EVs (GT3 has none): engine banks synthesised
// in the game's own ENGN layout, so EngineSound plays them exactly like a
// recorded engine (rpm pitch tracking, layer crossfades, Doppler, pan,
// surround cue).
//
//   engine bank  - the motor/gear whine: a tone at whine_order x 0.15 x rpm Hz
//                  with a few harmonics
//   exhaust bank - the inverter and load sound: a higher, rougher tone over
//                  band-limited hiss. EngineSound raises this bank with the
//                  throttle, so it swells under acceleration and fades on
//                  lift-off, like the real thing.

#include "formats/sound/engn.h"

#include <memory>

namespace gt3r {

struct MotorSoundParams {
    float whine_order = 1.0f;    // pitch multiplier (1 = Model 3/Y-like)
    float whine_level = 1.0f;    // 0..2
    float inverter_level = 1.0f; // 0..2
};

struct MotorBanks {
    std::shared_ptr<const sound::EngineBank> whine, inverter;
};

MotorBanks make_motor_banks(const MotorSoundParams& p);

} // namespace gt3r
