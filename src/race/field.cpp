#include "race/field.h"
#include "race/exact_field.h"

#include "base/bytes.h"
#include "game/data/car_database.h"
#include "sim/car_spec.h"

#include <algorithm>

namespace gt3r::race {

EventStart event_start(const data::Event& ev) {
    EventStart e;
    e.laps = ev.laps;
    e.cars = ev.cars_in_race;
    ByteSpan b = ev.bytes;
    if (b.size() < 0x1F4) return e;
    e.initial_kmh = float(b[0x198]);
    e.time_limit_s = 60.0 * double(b[0x1E4]); // u8 (+0x1E5 is the league byte)
    e.flying_m = -float(s16(rd16(b, 0x1E6)));
    e.type = b[0x1F3];
    switch (e.type) {
    case 0: e.countdown = double(b[0x1F2]); break;
    case 1: e.countdown = 4.0; break;
    case 2: e.countdown = 10.0; break;
    default: e.countdown = 0; break;
    }
    return e;
}

// kg per PS: mass over the peak of torque x rpm on the full-throttle curve.
float power_to_weight(const sim::CarSpec& s) {
    float best_w = 0;
    for (size_t i = 0; i < s.engine.curve_rpm.size() && i < s.engine.curve_torque.size(); ++i)
        best_w = std::max(best_w, s.engine.curve_torque[i] * s.engine.curve_rpm[i] * 0.10471976f); // N*m * rad/s
    float ps = best_w / 735.49875f;
    return ps > 1 ? s.mass / ps : 1e9f;
}

bool ranks_ahead(const RaceField::Standing& a, const RaceField::Standing& b) {
    if (a.laps != b.laps) return a.laps > b.laps;
    if (a.finished != b.finished) return !a.finished; // a finished car's place on the lap stopped at the line
    if (a.finished) return a.time < b.time;
    return a.distance > b.distance;
}

std::unique_ptr<RaceField> RaceField::create(const RaceTrack& track, const data::CarDatabase& db,
                                             const FieldConfig& cfg, std::string* error) {
    return make_exact_field(track, db, cfg, error);
}

} // namespace gt3r::race
