#include "race/hud_clocks.h"

#include <algorithm>

namespace gt3r::race {

void TitleClock::start(bool long_form) {
    on_ = true;
    if (long_form) t_ = -1.0f, duration_ = 4.0f, fade_in_ = 0.3f, fade_out_ = 1.0f;
    else t_ = -0.2f, duration_ = 2.0f, fade_in_ = 0.1f, fade_out_ = 0.3f;
}

void TitleClock::step() {
    if (on_ && t_ < duration_) t_ += 1.0f / 60.0f;
}

float TitleClock::alpha() const {
    if (!on_ || t_ < 0 || t_ > duration_) return 0;
    const float f = t_ < fade_in_ ? t_ / fade_in_ : (duration_ - t_) / fade_out_;
    return std::clamp(f * f, 0.0f, 1.0f);
}

void ReplayMarkTimer::step() {
    switch (state_) {
    case 0: // the start: hidden 3 s
        frames_ = 180;
        state_ = 1;
        [[fallthrough]];
    case 1: // hidden
        if (--frames_ < 0) state_ = 2;
        break;
    case 2: // fading in
        alpha_ += 1.0f / 120.0f;
        if (alpha_ > 1.0f) alpha_ = 1.0f, frames_ = 1800, state_ = 3;
        break;
    case 3: // shown
        if (--frames_ < 0) state_ = 4;
        break;
    case 4: // fading out
        alpha_ -= 1.0f / 120.0f;
        if (alpha_ < 0.0f) alpha_ = 0.0f, state_ = 5;
        break;
    default: // hidden 30 s
        frames_ = 1800;
        state_ = 1;
        break;
    }
}

} // namespace gt3r::race
