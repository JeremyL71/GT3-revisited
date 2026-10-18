#pragma once

// Stand-ins used while the real track runtime / vehicle simulation are not
// linked into the build (see backend.h). Deliberately simple:
//
// RoadTrack: surface probe = highest GTRW road triangle (Road::height_at),
//   walls = GTRW wall polylines tested in the XZ plane, lap distance from the
//   checkpoint ring.
// KinematicVehicle: a bicycle model with an engine torque curve, gearbox,
//   aerodynamic and rolling drag, a lateral-grip limit and a body that
//   follows the road under its four wheels. Enough to drive a lap and test
//   the cameras, HUD and audio; not a physics model.

#include "formats/course/road.h"
#include "race/backend.h"

namespace gt3r::race {

std::unique_ptr<RaceTrack> make_road_track(course::Road road, const std::string& label);
std::unique_ptr<RaceVehicle> make_kinematic_vehicle(const CarSetup& car, const RaceTrack& track);

} // namespace gt3r::race
