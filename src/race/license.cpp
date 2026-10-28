#include "race/license.h"

#include "game/data/car_database.h"

#include <algorithm>
#include <cmath>

namespace gt3r::race {

LicenseTimes license_times(ByteSpan b) {
    LicenseTimes t;
    if (b.size() < 0x1D0) return t;
    for (int i = 0; i < 3; ++i) {
        size_t o = 0x1C4 + 4 * size_t(i);
        int m = b[o], s = b[o + 1], ms = rd16(b, o + 2);
        int v = m * 60000 + s * 1000 + ms;
        t.ms[size_t(i)] = v > 0 ? v : -1;
    }
    t.kind = b[0x19A];
    return t;
}

LicenseTimes license_times(const data::Event& ev) { return license_times(ev.bytes); }

int license_medal(const LicenseTimes& t, double seconds) {
    if (seconds < 0) return -1;
    long long ms = std::llround(seconds * 1000.0);
    for (int i = 0; i < 3; ++i)
        if (t.ms[size_t(i)] > 0 && ms <= t.ms[size_t(i)]) return i;
    return -1;
}

LicenseOutcome license_outcome(int result, u32 result_time_ms, bool failed_event, double best) {
    LicenseOutcome o;
    if (result >= 2 && result <= 4) {
        o.time = double(result_time_ms) / 1000.0, o.medal = result - 2;
    } else if (result < 0) {
        o.state = LicenseRun::State::Failed, o.failure = "out of the course";
    } else if (failed_event) {
        o.time = result_time_ms != 0 ? double(result_time_ms) / 1000.0 : best, o.failure = "over the times";
    } else {
        o.time = best;
    }
    return o;
}

} // namespace gt3r::race
