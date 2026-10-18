#pragma once

// Shared contract for the driving simulation (milestone M2). Physics
// (src/sim/), track runtime (src/track/), the race scene (src/race/) and
// audio all exchange these plain types, so they can be developed in parallel.
//
// World frame (matches decoded course data): right-handed, +Y up, metres.
// Car frame (matches decoded car models): +X right, +Y up, nose towards -Z,
// origin at the wheelbase midpoint on the ground. Note that decoded AI line
// points store X negated; src/formats/course/ai_lines.* already corrects it.

#include "base/bytes.h"

#include <array>
#include <cmath>
#include <limits>

namespace gt3r::sim {

struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    float length() const { return std::sqrt(dot(*this)); }
};

// Unit quaternion, rotating car frame -> world frame.
struct Quat {
    float w = 1, x = 0, y = 0, z = 0;
};

// Simulation rate: confirmed 60 Hz, one step per frame, no substeps.
// Keep code parameterised on it anyway.
constexpr double kDefaultTickHz = 60.0;

// What the driver asks for this tick, already mapped from the virtual pad
// (pressure buttons and analog sticks) by the race scene.
struct ControlInputs {
    float steer = 0;    // -1 full left .. +1 full right
    // AI drivers set the road-wheel angle directly (radians, + = right); when
    // finite it replaces steer × lock and the pad steering curve.
    float steer_angle = std::numeric_limits<float>::quiet_NaN();
    // Steering device kind (spec §4.4): 2 analog, 3 digital pad; 0 = keep the
    // previous kind. Selects the steering follow rates.
    u8 controller_kind = 0;
    float throttle = 0; // 0..1 (e.g. Cross pressure / 255)
    float brake = 0;    // 0..1 (e.g. Square pressure / 255)
    bool handbrake = false;
    bool shift_up = false;   // edge-triggered requests (manual gearbox)
    bool shift_down = false;
    bool automatic = true;
};

// Result of probing the track surface below a point.
struct SurfaceHit {
    bool hit = false;
    float height = 0;          // world Y of the surface
    Vec3 normal{0, 1, 0};
    u32 surface = 0;           // raw GTRW surface value (meaning: see docs/formats/course.md)
    int segment = -1;          // road segment index, -1 if not on the road mesh
};

// Implemented by the track runtime (src/track/); consumed by physics.
class TrackQuery {
public:
    virtual ~TrackQuery() = default;
    // Highest surface at (x, z) at or below `y_max` (pass the wheel's
    // position plus a margin, so bridges/tunnels resolve correctly).
    virtual SurfaceHit probe(float x, float z, float y_max) const = 0;
};

struct WheelState {
    Vec3 contact;            // world contact point
    float steer_angle = 0;   // radians, positive = right
    float spin_angle = 0;    // radians, accumulated rotation (for rendering)
    float angular_velocity = 0; // rad/s
    float suspension = 0;    // compression in metres (0 = fully extended)
    float slip_ratio = 0;
    float slip_angle = 0;    // radians
    float load = 0;          // vertical load in N
    bool grounded = false;
    u32 surface = 0;
};

// Snapshot published by physics after each tick. Rendering, camera, HUD and
// audio only read this (interpolating between two ticks when rendering).
struct VehicleState {
    Vec3 position;           // world position of the car-frame origin
    Quat orientation;
    Vec3 velocity;           // world, m/s
    Vec3 angular_velocity;   // world, rad/s
    std::array<WheelState, 4> wheels; // FL, FR, RL, RR
    float engine_rpm = 0;
    int gear = 1;            // -1 reverse, 0 neutral, 1..N forward
    float throttle = 0;      // effective throttle after aids/limiter (for audio)
    float speed_kmh = 0;
    bool rev_limiter = false;
};

} // namespace gt3r::sim
