#pragma once

// Course rigid bodies (pylons / cones / small signboards) knocked about by the cars
// (docs/formats/course.md "Rigid bodies", docs/dev/RACE.md "Pylons").
//   - Placements: GTRW header +0x50 count, +0x54 offset of 32-byte records
//     {u32 type, u32 0, f32 x y z, f32 Euler a b c}; at most 100 bodies (pool size),
//     the rest are skipped.
//   - Types: course slot 36 (file offset-table entry 0x90): +4 zero-terminated offsets
//     (block-relative) of 0xB0-byte descriptors {N hull points, Ix Iy Iz, points,
//     +0x90 radius, +0x94 mass, +0x98 1/mass, +0xAC GTM1 offset}, each followed by its
//     own GTM1 (stream 0 = the model, stream 1 = a planar shadow).
//   - Physics per frame: asleep while >= 3 hull points rest on the ground; awake bodies
//     tumble (torque-free rotation, omega x 0.999, |omega| <= 20 rad/s, gravity), the
//     deepest ground point gets an impulse (restitution 0.25, 0 under 0.9 m/s; friction
//     0.5); a car's box (front / rear extents, half width, 1 m high, sides leaning in 5 %)
//     pushes a body out and hits it (restitution 0.01, inertia x 1.5) without any effect
//     on the car itself. Any touch is reported (licences fail on it). Awake bodies then meet
//     the course walls: the centre against each wall edge in x/z with the type's radius
//     (overlapping: put at the radius on the edge's drivable side; crossed this frame: back
//     along the motion onto the wall line), then the ground-contact impulse along the wall's
//     horizontal normal at the hull point deepest towards it.
//   - Drawing: every body (asleep too), its planar shadow first: the type's GTM1 node 1
//     projected along Env0 scene light 0's direction (+0xAD0) onto the plane of the road
//     triangle under the body's centre (kept while asleep), no depth writes, nudged 5 mm
//     towards the camera (GT3 moves near / far by 0.005); coloured like the car shadows
//     (Env0 +0xA60 colour, vertex alpha x +0xA6C: the state the car shadows leave, not set by
//     the pylon code itself).

#include "base/bytes.h"
#include "render/view3d.h"
#include "scene/model.h"
#include "sim/sim_types.h"
#include "sim/vehicle.h"

#include <array>
#include <memory>
#include <vector>

namespace gt3r::render {
class GpuContext;
class GpuModel;
} // namespace gt3r::render

namespace gt3r::race {

class Pylons {
public:
    struct Type {
        std::vector<sim::Vec3> hull; // body frame, origin at the centre of mass
        sim::Vec3 inertia{1, 1, 1};
        float radius = 1, mass = 1;
        std::unique_ptr<render::GpuModel> model, shadow;
        scene::Model shadow_src; // GTM1 node 1 (set_shadow colours it)
    };
    struct Body {
        int type = 0;
        sim::Vec3 pos, vel, omega; // omega in the body frame
        float rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // body -> world, row-major
        unsigned char contact[8] = {};
        bool active = true;
        sim::Vec3 plane_n{0, 1, 0}; // the road plane under the centre: n.p + d = 0 (shadow)
        float plane_d = 0;
    };
    // A car as the original's hit box: frame origin (wheelbase middle on the ground),
    // orientation, velocity, yaw rate, front / rear extents and half width.
    struct Car {
        sim::Vec3 pos, vel;
        sim::Quat orientation;
        float yaw_rate = 0;
        float front = 2.2f, rear = 2.2f, half_width = 0.9f;
    };
    struct Record {
        int type = 0;
        sim::Vec3 pos;
        float angles[3] = {};
    };

    Pylons();
    ~Pylons();
    // Parses the course's types and placements (no GPU work when ctx is null: tests).
    bool load(render::GpuContext* ctx, ByteSpan course_file);
    // (Re)places every body on the ground (race start, retry).
    void reset(const sim::TrackQuery& ground);
    // One frame; cars[0..n). Returns a bit mask of the cars that touched a body this frame.
    // `walls` (optional): the course walls awake bodies collide with.
    unsigned step(const sim::TrackQuery& ground, const std::vector<Car>& cars, double dt,
                  const sim::WorldQuery* walls = nullptr);
    // The shadows' colour (Env0 +0xA60) and alpha scale (+0xA6C); without it no shadows.
    void set_shadow(const std::array<float, 3>& rgb, float alpha);
    // Shadows (projected along `light`, from Env0 +0xAD0) then bodies; `eye` for the depth nudge.
    void add_items(std::vector<render::DrawItem>& items, const render::Vec3& eye, const render::Vec3& light) const;
    // The shadow transform of a body (world): its matrix projected onto its ground plane along
    // `light`; false when the light runs parallel to the plane.
    static bool shadow_matrix(const Body& b, const render::Vec3& light, render::Mat4& out);

    size_t count() const { return bodies_.size(); }
    const std::vector<Body>& bodies() const { return bodies_; }
    const std::vector<Record>& records() const { return records_; }
    const std::vector<Type>& types() const { return types_; }
    static constexpr size_t kPool = 100;

private:
    void ground_contact(Body& b, const Type& t, const sim::TrackQuery& ground);
    void wall_contact(Body& b, const Type& t, const sim::Vec3& old_pos, const sim::WorldQuery& walls);
    render::GpuContext* ctx_ = nullptr;
    std::vector<Type> types_;
    std::vector<Record> records_;
    std::vector<Body> bodies_;
};

} // namespace gt3r::race
