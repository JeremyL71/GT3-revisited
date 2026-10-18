#pragma once

// A CarSpec as bytes: every field (also the derived ones finalize() fills), floats as their
// bits, so a spec read back is identical to the one written and a re-simulation with it is
// bit-exact. Replays carry the spec their run used (garage cars with parts and settings,
// arcade cars with their tyre stage, the ASM / TCS options), so a saved replay plays back
// with the same car wherever it is loaded. The layout is this program's own (versioned).

#include "base/bytes.h"
#include "sim/car_spec.h"

#include <optional>

namespace gt3r::sim {

Bytes serialize_car_spec(const CarSpec& spec);
// nullopt on malformed or truncated input (or an unknown layout version).
std::optional<CarSpec> parse_car_spec(ByteSpan data);

} // namespace gt3r::sim
