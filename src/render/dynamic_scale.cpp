#include "render/dynamic_scale.h"

#include "platform/render_scale.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace gt3r::render {

void DynamicScale::reset(float max_scale, float refresh_hz, float start_scale) {
    *this = DynamicScale{};
    max_ = std::clamp(max_scale, kMin, platform::kMaxRenderScale);
    scale_ = start_scale > 0 ? std::clamp(start_scale, kMin, max_) : max_;
    if (refresh_hz > 1) {
        refresh_ms_ = 1000.0 / refresh_hz;
        target_ms_ = std::ceil(1000.0 / 60.0 / refresh_ms_ - 0.05) * refresh_ms_;
    }
}

float DynamicScale::frame(double frame_ms, double present_ms) {
    if (frame_ms <= 0) return scale_;
    if (frame_ms > 250) { // a load or a pause: the next ~5 s settle (first draws, pipelines warming)
        window_ms_ = present_ms_ = late_ms_ = 0, frames_ = missed_ = 0;
        settle_ = 5;
        quick_ = true, probed_ = false;
        return scale_;
    }
    window_ms_ += frame_ms, present_ms_ += present_ms, ++frames_;
    // A late frame and the next together over two targets by most of a refresh: shown a refresh
    // late, a judder (on the Pixel's capped 120 Hz screen the wobble came to 33-39 ms, a real miss
    // to 42 ms or more).
    if (late_ms_ > 0 && late_ms_ + frame_ms > 2 * target_ms_ + 0.75 * refresh_ms_) ++missed_;
    late_ms_ = frame_ms > target_ms_ * 1.25 ? frame_ms : 0;
    if (window_ms_ < 1000) return scale_;
    const double avg = window_ms_ / frames_;
    const bool gpu_bound = present_ms_ >= 0.25 * window_ms_;
    const int missed = missed_;
    window_ms_ = present_ms_ = 0, frames_ = missed_ = 0;
    const bool settling = settle_ > 0;
    if (settling) --settle_;
    if (settling && !quick_) return scale_;
    if (ceiling_windows_ > 0 && --ceiling_windows_ == 0) ceiling_ = kNoCeiling;
    const bool probed = std::exchange(probed_, false);
    if (probed && (missed > 1 || avg > target_ms_ * 1.05)) {
        // The step up was one too many: back down, and not again for a minute (two, then three
        // after more misses).
        good_windows_ = 0;
        probe_after_ = std::min(probe_after_ * 2, 30);
        ceiling_ = scale_, ceiling_windows_ = 6 * probe_after_;
        scale_ = before_probe_;
        settle_ = 1;
        quick_ = false;
        return scale_;
    }
    if (avg > target_ms_ * 1.1 || missed >= 3) {
        good_windows_ = 0;
        if (settling) return scale_; // slow seconds after a load are not judged
        // CPU-bound frames: a smaller picture would not help.
        if (!gpu_bound) return scale_;
        // Fill time goes with the pixel count, the scale squared.
        float to = std::floor(scale_ * float(std::sqrt(target_ms_ / avg)) / kStep + 1e-3f) * kStep;
        const float before = scale_;
        scale_ = std::clamp(std::min(to, scale_ - kStep), kMin, max_);
        if (scale_ != before) settle_ = 1, quick_ = false; // the next second shows the new size
        return scale_;
    }
    if (missed > 0) { // the odd miss: hold
        good_windows_ = 0;
        return scale_;
    }
    const float up = std::min(max_, scale_ + kStep);
    if (++good_windows_ >= (quick_ ? 1 : probe_after_) && up > scale_ + 1e-4f && up < ceiling_ - 1e-4f) {
        before_probe_ = scale_;
        scale_ = up;
        good_windows_ = 0;
        probed_ = true;
    }
    return scale_;
}

} // namespace gt3r::render
