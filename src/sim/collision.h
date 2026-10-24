#pragma once

// 2-D rigid-body collision: the car is an
// octagon in the ground plane; walls are segments in the course's (x, z)
// plane. Pure functions over plain structs so they can be checked against the
// original game's collision behaviour.
//
// Frames: world x/z (sim_types.h, Y up). Body-local points: x = lateral
// (right), y = forward, relative to the centre of gravity. Heading c is the
// spec's (physics §1.3): the car's forward axis in world is (-sin c, -cos c).

#include <array>
#include <vector>

namespace gt3r::sim::collide {

struct V2 {
    double x = 0, z = 0;
    V2 operator+(const V2& o) const { return {x + o.x, z + o.z}; }
    V2 operator-(const V2& o) const { return {x - o.x, z - o.z}; }
    V2 operator*(double s) const { return {x * s, z * s}; }
    double dot(const V2& o) const { return x * o.x + z * o.z; }
    double cross(const V2& o) const { return z * o.x - x * o.z; } // (a x b).y for vectors in the x/z plane
};

struct Body {
    V2 pos;                  // CoG, world x/z
    double heading = 0;      // rad
    V2 vel;                  // horizontal velocity, m/s
    double vy = 0;           // vertical velocity (only wall friction touches it)
    double yaw_rate = 0;     // rad/s about +Y (left turn > 0)
    double inv_mass = 0;     // 1 / m
    double inv_inertia = 0;  // 1 / (10 I_z) [physics §5.3]
    std::array<V2, 8> local; // octagon, counter-clockwise (see octagon())
    double radius = 0;       // bounding radius
};

// [physics §5.3] Octagon outline: a = CoG->front, b = CoG->rear,
// h = width / 2, chamfer d = 0.2 width; points (x lateral, y forward):
// (h, a-d), (h-d, a), (-h+d, a), (-h, a-d), (-h, -b+d), (-h+d, -b), (h-d, -b), (h, -b+d).
std::array<V2, 8> octagon(double front, double rear, double width);
double bounding_radius(const std::array<V2, 8>& pts);

// [physics §5.3] World position of a body-local point.
V2 world_point(const V2& pos, double heading, const V2& local);
// Rotates a world vector into / from the body frame.
V2 to_world_dir(double heading, const V2& local);

struct Segment {
    V2 a, b;
    bool sensor = false; // crossing only records an event
};

// [physics §5.3] Impulse response of a body point at r
// (relative to the CoG, world) against a wall with unit normal n (pointing to
// the side the body is on): j = -(1 + e)(v_p . n) / K, K = 1/m + (1/I)(r x n)^2;
// friction min(mu |j|, m |v_t|) against the tangential velocity (incl.
// vertical; no torque). Returns |j| (0 when separating).
double wall_impulse(Body& b, const V2& r, const V2& n, double restitution = 0.05, double friction = 0.5);

struct WallHit {
    V2 point;     // world hit point
    V2 normal;    // wall normal towards the body
    double impulse = 0;
    bool sensor = false;
    int body_point = -1; // the octagon point (0..7) that hit
};

// [ai §7] Which end of a car a collision response acted on, per collision step: kFrontHit for
// the octagon's points 0-3 (wall or car-car) and its front edges 0-2 (car-car), kRearHit for
// points 4-7 and the rear edges 4-6. The side edges 3 and 7 count as neither.
enum HitSide : unsigned { kFrontHit = 1, kRearHit = 2 };
inline unsigned point_side(int point) {
    return point >= 0 && point < 4 ? kFrontHit : point >= 4 && point < 8 ? kRearHit : 0u;
}
inline unsigned edge_side(int edge) { return edge >= 0 && edge < 3 ? kFrontHit : edge >= 4 && edge < 7 ? kRearHit : 0u; }

// Wall contacts that persist across steps [physics §5.3]: a body point
// that hit a wall (impulse, or a push-out of at most 0.5 m) stays flagged with
// its wall segment. Until the flag drops, the point is held on the wall line
// and the body's velocity is projected on `tangent` (the last hit wall's unit
// tangent) every round, and the point is skipped by the crossing test and the
// push-out. Keep one per body; clear it when the car is placed.
struct WallContacts {
    std::array<bool, 8> on{};
    std::array<Segment, 8> seg{};
    V2 tangent{1, 0};
    void clear() { on = {}; }
};

// [physics §5.3] Swept wall collision of a body that moved
// from pose `start` (position, heading) to its current pose during dt:
// up to `iterations` earliest-crossing passes (advance to the hit, impulse,
// re-sweep the remainder), then a perpendicular push-out of points still
// beyond a wall. With `contacts`, each round starts with the persistence
// pass of flagged points (see WallContacts). Walls are one-sided: the track is on the LEFT of a->b (the
// positive side, normal (b-a).z, -(b-a).x) and only points that start on that
// side and cross it are stopped [physics §5.3; verified on the US disc by track].
// Returns the hits.
std::vector<WallHit> collide_walls(Body& b, const V2& start_pos, double start_heading,
                                   const std::vector<Segment>& walls, double dt, int iterations = 2,
                                   WallContacts* contacts = nullptr);

// [physics §5.3] Static car-car contact (no sweep). Separating-axis test of the
// two octagons; on overlap, the contact feature is the deepest vertex of either
// body inside the other against its nearest edge. Impulse as the swept
// response (restitution 0.5, no friction, normal = that edge's normal, rA at
// the vertex projected onto the edge, rB at the vertex, not clamped for
// separating contact), then the overlap split half and half. `normal` points
// from a towards b. Returns |j| (0 = no contact, 1e-6 = touching only).
double collide_pair(Body& a, Body& b, V2* point = nullptr, V2* normal = nullptr);

// [physics §5.3, all-bodies mode] Swept car-car step. On entry the
// bodies hold velocities (pose ignored); the step starts from the given start
// poses. At most two rounds of: static separation along the line of centres
// (both orderings), persistence of vertices flagged by an earlier response,
// end pose = start + v dt (1 - t), the pair sweep (each vertex of one body on a
// straight line in the other's frame against its edges, both orderings, ties
// to body 0, flagged vertices skipped), advance to the earliest hit, impulse at
// the crossing vertex with the hit edge's normal plus the touch-up. The bodies
// end at the last end pose computed (an impulse in the last round changes only
// velocities). `point`/`normal` give the first contact (normal from body 0 to
// body 1). Returns the total |j| (0 = no contact).
// `steer` (optional, two values): each car's steering input as a fraction of
// full lock, -1 left .. +1 right (the game's signed percent / 100). When a body
// ends nearly stopped (|vx|, |vy| < 1 m/s) and the last hit was early (t < 0.1),
// besides recomputing its end pose it gets the stuck-steer yaw nudge
// yaw rate += -steer x 20 x dt (positive yaw = left) [physics §5.3].
// `first_t` (optional): the first contact's time as a fraction of the step
// (the straight-chord crossing time; unchanged when there is no contact).
// `sides` (optional, two values): ORed with the HitSide of every response, per body: the
// hitting vertex's point_side for its body, the hit edge's edge_side for the other.
double collide_pair_swept(Body& a, const V2& a_start, double a_start_heading, Body& b, const V2& b_start,
                          double b_start_heading, double dt, int iterations = 2, V2* point = nullptr, V2* normal = nullptr,
                          const float* steer = nullptr, double* first_t = nullptr, unsigned* sides = nullptr);

} // namespace gt3r::sim::collide
