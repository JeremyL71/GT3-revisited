#include "render/camera.h"

#include <algorithm>
#include <cmath>

namespace gt3r::render {

namespace {

constexpr float kMaxPitch = 1.50f; // just under 90 degrees

float stick(const PadState& pad, PadAxis a) {
    float v = pad.axis_f(a);
    return std::fabs(v) < 0.15f ? 0.0f : v; // dead zone
}

} // namespace

void OrbitCamera::frame(const float bmin[3], const float bmax[3]) {
    target_ = {(bmin[0] + bmax[0]) * 0.5f, (bmin[1] + bmax[1]) * 0.5f, (bmin[2] + bmax[2]) * 0.5f};
    Vec3 ext{bmax[0] - bmin[0], bmax[1] - bmin[1], bmax[2] - bmin[2]};
    radius_ = std::max(length(ext) * 0.5f, 1e-3f);
    distance = radius_ / std::sin(fovy * 0.5f) * 1.05f;
}

void OrbitCamera::zoom(float factor) {
    distance = std::clamp(distance * factor, radius_ * 0.2f, radius_ * 50.0f);
}

void OrbitCamera::update(const PadState& pad, double dt) {
    float fdt = float(dt);
    float ox = stick(pad, PadAxis::LX), oy = stick(pad, PadAxis::LY);
    if (pad.down(PadButton::Left)) ox = -1;
    if (pad.down(PadButton::Right)) ox = 1;
    if (pad.down(PadButton::Up)) oy = -1;
    if (pad.down(PadButton::Down)) oy = 1;
    float z = stick(pad, PadAxis::RY);
    if (pad.down(PadButton::L1) || pad.down(PadButton::L2)) z -= 1; // zoom in
    if (pad.down(PadButton::R1) || pad.down(PadButton::R2)) z += 1; // zoom out
    if (ox != 0 || oy != 0 || z != 0) idle_ = 0;
    else idle_ += dt;

    yaw += ox * 2.0f * fdt;
    pitch = std::clamp(pitch - oy * 1.5f * fdt, -kMaxPitch, kMaxPitch);
    if (z != 0) zoom(std::exp(z * 1.5f * fdt));
    if (turntable && idle_ > 3.0 && !dragging_ && finger_count_ == 0) yaw += turntable_speed * fdt;
}

bool OrbitCamera::handle_event(const SDL_Event& e) {
    switch (e.type) {
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        if (e.button.which == SDL_TOUCH_MOUSEID) return false; // handled as touch
        if (e.button.button == SDL_BUTTON_LEFT) dragging_ = true;
        else panning_ = true;
        idle_ = 0;
        return true;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (e.button.button == SDL_BUTTON_LEFT) dragging_ = false;
        else panning_ = false;
        return true;
    case SDL_EVENT_MOUSE_MOTION:
        if (e.motion.which == SDL_TOUCH_MOUSEID) return false;
        if (dragging_) {
            yaw += e.motion.xrel * 0.01f;
            pitch = std::clamp(pitch + e.motion.yrel * 0.01f, -kMaxPitch, kMaxPitch);
            idle_ = 0;
            return true;
        }
        if (panning_) {
            // Pan in the view plane, scaled so the target follows the cursor roughly.
            Vec3 f = normalize(target_ - eye());
            Vec3 s = normalize(cross(f, {0, 1, 0}));
            Vec3 u = cross(s, f);
            float k = distance * 0.002f;
            target_ = target_ - s * (e.motion.xrel * k) + u * (e.motion.yrel * k);
            idle_ = 0;
            return true;
        }
        return false;
    case SDL_EVENT_MOUSE_WHEEL:
        zoom(std::exp(-e.wheel.y * 0.1f));
        idle_ = 0;
        return true;
    case SDL_EVENT_FINGER_DOWN:
        if (finger_count_ < 2) {
            fingers_[finger_count_] = e.tfinger.fingerID;
            finger_pos_[finger_count_][0] = e.tfinger.x;
            finger_pos_[finger_count_][1] = e.tfinger.y;
            ++finger_count_;
        }
        idle_ = 0;
        return true;
    case SDL_EVENT_FINGER_UP:
    case SDL_EVENT_FINGER_CANCELED:
        for (int i = 0; i < finger_count_; ++i)
            if (fingers_[i] == e.tfinger.fingerID) {
                if (i == 0 && finger_count_ == 2) {
                    fingers_[0] = fingers_[1];
                    finger_pos_[0][0] = finger_pos_[1][0];
                    finger_pos_[0][1] = finger_pos_[1][1];
                }
                --finger_count_;
                break;
            }
        return true;
    case SDL_EVENT_FINGER_MOTION: {
        int i = -1;
        for (int k = 0; k < finger_count_; ++k)
            if (fingers_[k] == e.tfinger.fingerID) i = k;
        if (i < 0) return false;
        if (finger_count_ == 2) {
            float old_d = std::hypot(finger_pos_[0][0] - finger_pos_[1][0], finger_pos_[0][1] - finger_pos_[1][1]);
            finger_pos_[i][0] = e.tfinger.x;
            finger_pos_[i][1] = e.tfinger.y;
            float new_d = std::hypot(finger_pos_[0][0] - finger_pos_[1][0], finger_pos_[0][1] - finger_pos_[1][1]);
            if (old_d > 1e-4f && new_d > 1e-4f) zoom(old_d / new_d);
        } else {
            finger_pos_[i][0] = e.tfinger.x;
            finger_pos_[i][1] = e.tfinger.y;
            yaw += e.tfinger.dx * 0.012f; // UI units (App converts events to render coordinates)
            pitch = std::clamp(pitch + e.tfinger.dy * 0.012f, -kMaxPitch, kMaxPitch);
        }
        idle_ = 0;
        return true;
    }
    default:
        return false;
    }
}

Vec3 OrbitCamera::eye() const {
    float cp = std::cos(pitch);
    return target_ + Vec3{std::sin(yaw) * cp, std::sin(pitch), std::cos(yaw) * cp} * distance;
}

Mat4 OrbitCamera::projection(float aspect) const {
    float zn = std::max(distance - radius_ * 2.0f, distance * 0.01f);
    zn = std::max(zn, radius_ * 0.01f);
    float zf = distance + radius_ * 4.0f;
    return perspective(fovy, aspect, zn, zf);
}

} // namespace gt3r::render
