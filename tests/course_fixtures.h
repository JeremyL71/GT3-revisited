#pragma once

// A disc-free course for the AI and race tests: a "stadium" oval built in code with its road
// mesh, checkpoints, walls, grid and AI lines. All geometry and AI line values are INVENTED
// (round numbers chosen for these tests), not taken from any course file.
//
// The lap runs anticlockwise seen from above (+Y), every corner a left-hander:
//   s = 0 at (50, 0), heading -Z, half of straight A (100 m) to (50, -100);
//   arc 1: radius 50 around (0, -100) to (-50, -100);
//   straight B (200 m, heading +Z) to (-50, 100);
//   arc 2: radius 50 around (0, 100) to (50, 100);
//   the other half of straight A back to the line. Length 400 + 100 pi = 714.16 m.
// The road is 16 m wide; walls run 1 m beyond each edge. Grid slots stand on straight A
// behind the line. AI lines (game numbering): 6 = the centre line, 1 = a lane 3.5 m right of
// it (outside), 2 = 3.5 m left (inside), as control points with straights and arcs.

#include "formats/course/ai_lines.h"
#include "formats/course/road.h"
#include "sim/sim_types.h"
#include "track/track.h"

#include <cmath>
#include <memory>
#include <vector>

namespace gt3r::test::stadium {

constexpr float kPi = 3.14159265358979f;
constexpr float kRadius = 50, kStraight = 200, kHalfWidth = 8, kLane = 3.5f;
constexpr float kArc = kPi * kRadius;
constexpr float kLength = 2 * kStraight + 2 * kArc;
// Where each part of the lap starts (track distance).
constexpr float kArc1 = kStraight / 2, kStraightB = kArc1 + kArc, kArc2 = kStraightB + kStraight,
                kStraightA = kArc2 + kArc;

// Centre-line point and unit travel direction at track distance s.
inline void centre(float s, sim::Vec3& p, sim::Vec3& d) {
    s = std::fmod(s, kLength);
    if (s < 0) s += kLength;
    if (s < kArc1) {
        p = {kRadius, 0, -s};
        d = {0, 0, -1};
    } else if (s < kStraightB) {
        float th = (s - kArc1) / kRadius;
        p = {kRadius * std::cos(th), 0, -kStraight / 2 - kRadius * std::sin(th)};
        d = {-std::sin(th), 0, -std::cos(th)};
    } else if (s < kArc2) {
        p = {-kRadius, 0, -kStraight / 2 + (s - kStraightB)};
        d = {0, 0, 1};
    } else if (s < kStraightA) {
        float th = (s - kArc2) / kRadius;
        p = {-kRadius * std::cos(th), 0, kStraight / 2 + kRadius * std::sin(th)};
        d = {std::sin(th), 0, std::cos(th)};
    } else {
        p = {kRadius, 0, kStraight / 2 - (s - kStraightA)};
        d = {0, 0, -1};
    }
}

// Left of the travel direction (Y up): the infield on this course.
inline sim::Vec3 left_of(const sim::Vec3& d) { return {d.z, 0, -d.x}; }

// The point `lateral` metres left of the centre line at s.
inline sim::Vec3 at(float s, float lateral = 0) {
    sim::Vec3 p, d;
    centre(s, p, d);
    return p + left_of(d) * lateral;
}

// Car pose (origin and orientation, nose along the lap) at s, `lateral` metres left.
inline void pose(float s, float lateral, sim::Vec3& position, sim::Quat& orientation, float extra_yaw = 0) {
    sim::Vec3 p, d;
    centre(s, p, d);
    position = p + left_of(d) * lateral;
    float yaw = std::atan2(-d.x, -d.z) + extra_yaw; // rotation about +Y of the car's -Z nose
    orientation = {std::cos(yaw / 2), 0, std::sin(yaw / 2), 0};
}

inline course::Road make_road(int grid_slots = 6) {
    course::Road r;
    r.length = kLength;
    const int n = 144; // ~5 m segments
    auto v3 = [](const sim::Vec3& p) { return course::Vec3{p.x, p.y, p.z}; };
    for (int i = 0; i < n; ++i) {
        float s0 = kLength * float(i) / n, s1 = kLength * float(i + 1) / n;
        course::RoadSegment seg;
        for (sim::Vec3 p : {at(s0, kHalfWidth), at(s0, -kHalfWidth), at(s1, -kHalfWidth), at(s1, kHalfWidth)})
            seg.vertices.push_back(v3(p));
        seg.triangles.push_back({0, 1, 2, 0, 0, 0, 0});
        seg.triangles.push_back({0, 2, 3, 0, 0, 0, 0});
        seg.bounds_min = {-kRadius - 20, 0, -kStraight / 2 - kRadius - 20}; // loose bounds suffice
        seg.bounds_max = {kRadius + 20, 0, kStraight / 2 + kRadius + 20};
        r.segments.push_back(std::move(seg));
        course::Checkpoint c;
        sim::Vec3 a = at(s0, kHalfWidth), b = at(s0, -kHalfWidth), p, d;
        centre(s0, p, d);
        c.x0 = a.x, c.z0 = a.z, c.x1 = b.x, c.z1 = b.z;
        c.distance = s0;
        c.dir_x = -d.x, c.dir_z = -d.z; // stored against travel, like the game data
        r.checkpoints.push_back(c);
    }
    // Closed wall loops, drivable side on the left of the polyline: the outer wall runs with
    // the lap, the inner one against it.
    const int m = 288;
    for (int w = 0; w < 2; ++w) {
        for (int i = 0; i < m; ++i) {
            float s = kLength * float(w == 0 ? i : m - i) / m;
            sim::Vec3 p = at(s, w == 0 ? -(kHalfWidth + 1) : kHalfWidth + 1);
            u32 tag = i == 0 ? u32(m) : i == m - 1 ? u32(0x10000 - m) : 0;
            r.wall_points.push_back({v3(p), tag});
        }
    }
    // Grid: two columns 3 m either side of the centre, 8 m apart, behind the line.
    for (int k = 0; k < grid_slots; ++k) {
        course::GridSlot g;
        g.pos = v3(at(kLength - 6 - 8.0f * float(k), k % 2 ? -3.0f : 3.0f));
        g.yaw = -kPi; // heading = yaw + pi: nose along -Z, the lap direction on straight A
        r.grid.push_back(g);
    }
    return r;
}

// AI line `lateral` metres left of the centre: control points at the start of each straight
// and arc, every 50 m on the straights and at the middle of each arc (arcs: flags bit 0 and
// the signed radius, > 0 turning left).
inline course::AiLine make_line(float lateral) {
    course::AiLine line;
    line.closed = true;
    auto add = [&](float s, bool arc) {
        course::AiPoint p;
        sim::Vec3 q = at(s, lateral), c, d;
        centre(s, c, d);
        p.pos = {q.x, 0, q.z};
        p.flags = arc ? 1u : 0u;
        p.lateral = arc ? kRadius - lateral : 0;
        p.heading = std::atan2(-d.x, d.z); // travel direction (-sin h, cos h)
        line.points.push_back(p);
    };
    for (float s = 0; s < kArc1; s += 50) add(s, false);
    add(kArc1, true);
    add(kArc1 + kArc / 2, true);
    for (float s = kStraightB; s < kArc2; s += 50) add(s, false);
    add(kArc2, true);
    add(kArc2 + kArc / 2, true);
    for (float s = kStraightA; s < kLength - 1; s += 50) add(s, false);
    return line;
}

inline course::AiLines make_ai_lines() {
    course::AiLines ai;
    ai.lines.resize(7);
    ai.lines[0] = make_line(-kLane); // game line 1: outside lane
    ai.lines[1] = make_line(kLane);  // game line 2: inside lane
    ai.lines[5] = make_line(0);      // game line 6: the main line
    ai.build = "synthetic";
    return ai;
}

// The course's AI driver parameter set (course file slots 37-40), INVENTED for this course: corner speed 70 % and no
// corner bonus, pit speed 60 km/h; otherwise the built-in set's bytes. The same set for every driveline class.
inline course::DriverSets make_driver_sets() {
    const course::DriverSets::Set set = {100, 70, 100, 24, 100, 0, 90, 15, 40, 0, 0, 10, 0, 60, 100, 0, 9, 0, 60};
    course::DriverSets sets;
    for (auto& s : sets.by_class) s = set;
    return sets;
}

inline std::unique_ptr<track::Track> make_track(int grid_slots = 6) {
    auto t = std::make_unique<track::Track>(make_road(grid_slots), make_ai_lines(), "stadium");
    t->set_driver_sets(make_driver_sets());
    return t;
}

} // namespace gt3r::test::stadium
