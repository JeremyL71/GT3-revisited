#pragma once

// The car a player races online, as the lobby sends it (docs/dev/ONLINE.md "Cars").
//
// - Disc car: its database label.
// - Garage car (GT Mode): the base car, its 27 fitted part ids, the user settings, the
//   engine condition and wheel - about 450 bytes. Every machine rebuilds the exact spec
//   from its own disc (sim::build_car_spec), so nothing but these choices travels and
//   every player can check the car.
// - Mod car: its label plus the SHA-256 of its package (game/mod_package.h); a player
//   who lacks the car is offered the package from its owner.

#include "base/bytes.h"
#include "sim/car_spec.h"

#include <array>
#include <optional>
#include <string>

namespace gt3r::data {
class CarDatabase;
}

namespace gt3r::game {

struct GarageCar;

struct OnlineCar {
    enum class Kind : u8 { Disc = 0, Garage = 1, Mod = 2 };
    Kind kind = Kind::Disc;
    std::string car;  // database car label (garage: the base car)
    std::string name; // display name as the owner's game shows it
    u8 paint = 0;
    // Garage car.
    std::array<u64, 27> parts{};
    sim::CarSettings settings;
    float engine_wear_m = 0, oil_m = 0; // CarCondition (engine output factor)
    std::string wheel;                  // fitted wheel (wheel/<label>), "" = stock
    // Mod car.
    std::string package; // lower-case hex SHA-256 of the car's package
    u32 package_size = 0;

};

constexpr size_t kMaxOnlineCarBytes = 1024;

Bytes encode_online_car(const OnlineCar& c);
std::optional<OnlineCar> decode_online_car(ByteSpan data); // nullopt on malformed input

// A disc or mod car from the database (kind Disc or Mod; a mod car's package fields are
// filled by the caller from game/mod_package.h).
OnlineCar online_car_from_db(const data::CarDatabase& db, const std::string& label, u8 paint);
// A GT Mode garage car.
OnlineCar online_car_from_garage(const data::CarDatabase& db, const GarageCar& g);

// The car's spec on this machine: the disc car, the garage car rebuilt from its parts and
// settings (with its engine condition), or the installed mod car. nullopt with *error when
// the car (or a part) is not in this database.
std::optional<sim::CarSpec> online_car_spec(const data::CarDatabase& db, const OnlineCar& c, std::string* error = nullptr);

// Max power (PS) and its rpm the way the original computes the figure its menus show for a
// built car: the full-throttle torque curve (with
// the engine condition already applied by the caller's spec) sampled every 100 rpm from the
// first curve point (rounded to 100 rpm) up to the rev limit; power = (N*m / 9.8) * rpm / 716.2.
struct PowerPeak {
    float ps = 0;
    int rpm = 0;
};
PowerPeak spec_power_peak(const sim::CarSpec& spec);
// Peak power at the crank (PS) of a spec (spec_power_peak().ps).
float spec_power_ps(const sim::CarSpec& spec);
// The integer a menu shows for a power in PS: NTSC-U menus convert to HP (x 0.98619), and the
// value is truncated, not rounded. power_unit() is the label printed after it ("HP" / "PS").
int display_power(float ps, bool imperial);
const char* power_unit(bool imperial);

// Highest speed the car's gearing allows at its rev limit (m/s): no driven car goes faster on
// the flat (plausibility checks).
float geared_top_speed(const sim::CarSpec& spec);

// Whether a car may race under a lobby's rules (stock only: no garage cars; max_ps: power
// limit, 0 = none). `why` says what breaks them.
bool car_meets_rules(const OnlineCar& c, const sim::CarSpec* spec, bool stock_only, int max_ps, std::string* why = nullptr);

// The garage car's gearbox is a racing one (gear part +0x10 != 0): the transmission whine.
bool online_car_racing_gearbox(const data::CarDatabase& db, const OnlineCar& c);

} // namespace gt3r::game
