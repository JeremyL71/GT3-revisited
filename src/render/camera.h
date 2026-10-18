#pragma once

// Orbit camera for model viewers: rotates around a target point.
//
// Controls (virtual pad): left stick / d-pad orbit, right stick Y or
// L1/R1 / L2/R2 zoom. Desktop: left mouse drag orbits, right/middle drag
// pans, wheel zooms. Touch: one finger orbits, two-finger pinch zooms.
// While nothing is touched the camera can slowly turn (turntable mode).

#include "input/virtual_pad.h"
#include "render/math3d.h"

#include <SDL3/SDL_events.h>

namespace gt3r::render {

class OrbitCamera {
public:
    // Centres the camera on an axis-aligned box and picks a distance that fits it.
    void frame(const float bmin[3], const float bmax[3]);
    void update(const PadState& pad, double dt);
    // Mouse / wheel / touch, in UI (render) coordinates as screens receive
    // them. Returns true when the event was used.
    bool handle_event(const SDL_Event& e);

    Vec3 eye() const;
    Mat4 view() const { return look_at(eye(), target_, {0, 1, 0}); }
    Mat4 projection(float aspect) const;

    float yaw = 0.6f;     // radians around +Y
    float pitch = 0.35f;  // radians above the horizon
    float distance = 5.0f;
    float fovy = 0.8f;    // ~45 degrees
    bool turntable = true; // auto-rotate while idle
    float turntable_speed = 0.35f; // rad/s

private:
    void zoom(float factor);

    Vec3 target_{0, 0, 0};
    float radius_ = 1.0f; // scene radius, sets near/far planes and zoom limits
    double idle_ = 10.0;  // seconds since the last user input
    bool dragging_ = false, panning_ = false;
    // Touch tracking (two fingers for pinch).
    SDL_FingerID fingers_[2] = {0, 0};
    float finger_pos_[2][2] = {};
    int finger_count_ = 0;
};

} // namespace gt3r::render
