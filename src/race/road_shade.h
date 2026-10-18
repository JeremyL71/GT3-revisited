#pragma once

// Road shade at a point (GT3: the two u16 attributes of each road vertex, A and B, / 65535,
// interpolated over the highest road triangle under the point, like the height). On day
// courses B is an open-sky mask (0 in tunnels and under bridges) and A a direct-sun mask;
// on point-light courses (GTRW +0x21 = 1) A holds the nearest lamp's index. GT3 averages them
// over the four wheels and uses them for the car's lighting and shadow strength.

#include "formats/course/road.h"

namespace gt3r::race {

struct RoadShade {
    bool hit = false;
    float a = 1, b = 1; // interpolated attributes A, B (0..1)
    u16 raw_a = 0;      // attribute A of the triangle's first vertex (point-light index)
};

// `use_grid` = false: test every triangle (slow; for checks).
RoadShade road_shade(const course::Road& road, float x, float z, float y_max, bool use_grid = true);

} // namespace gt3r::race
