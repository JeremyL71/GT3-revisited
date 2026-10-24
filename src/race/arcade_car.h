#pragma once

// Arcade cars (database table 35 variants) for a race: built with their own parts, the tyres
// following the car page's RACING / DRIFT choice. GT3 switches both tyre parts to the car's
// stage-10 ("_k", racing) or stage-11 ("_l", drift) records of tables 28/29; dirt courses force
// stage 12 ("_m"), the wet Route 5 (course category 1) adds 13 (wet variants "_n".."_y").

#include "sim/car_spec.h"

#include <optional>
#include <string>

namespace gt3r::data {
class CarDatabase;
}

namespace gt3r::race {

struct ArcadeCar {
    std::string label, car; // variant label, base car
    int stage = 10;
    sim::CarSpec spec;
};

// `id` = the menu's car id (the variant label, with or without its leading '_'); tire_mode
// 0 racing / 1 drift (-1 = racing). nullopt when `id` is not an arcade car.
std::optional<ArcadeCar> arcade_car(const data::CarDatabase& db, const std::string& id, int tire_mode,
                                    const std::string& course, std::string* error = nullptr);

// The AI's built-in driver parameter set for a course (ai §9.1: the course category,
// course table 34 byte +8: 1 -> set 1, 2 (dirt) -> set 2, else 0), used where the course file
// has no set for a car's driveline class (ai::course_params).
int driver_param_set(const data::CarDatabase& db, const std::string& course);

} // namespace gt3r::race
