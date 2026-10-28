#pragma once

// A replay's race record for tests of the replay file and the theater (a replay without one is refused): one car, no
// frames; its contents do not matter to those tests.

#include "sim/exact/race_record.h"

#include <memory>

namespace gt3r::test {

inline std::shared_ptr<const sim::exact::RaceRecord> race_record_stub() {
    sim::exact::RaceRecord r;
    r.entries.resize(1);
    r.pads.assign(1, {});
    return std::make_shared<const sim::exact::RaceRecord>(std::move(r));
}

} // namespace gt3r::test
