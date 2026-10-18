#pragma once

// Full-screen "loading" transitions (the original's four kinds:
// comb-out, line-out, delayed fade-out, fade-out). They are drawn every frame on top
// of whatever was last shown, without clearing, so their effects accumulate;
// after 3 s a small blinking square in the top-left corner signals that the
// game is still loading.
//
// Usage: keep the last menu frame in a 640x448 target texture, call start(),
// then each 60 Hz frame call draw_frame() with that texture as the render
// target and present the texture.

#include "base/bytes.h"

#include <SDL3/SDL_render.h>

#include <array>

namespace gt3r::menu::flow {

// The original's type numbers.
enum class TransitionType : u8 { Fadeout = 0, DelayFadeout = 1, Lineout = 2, Combout = 3 };

class ScreenTransition {
public:
    void start(TransitionType type);
    void stop() { active_ = false; }
    bool active() const { return active_; }
    TransitionType type() const { return type_; }
    int frames() const { return frame_; }
    // Frames after which the effect has (practically) covered the screen.
    static int settle_frames(TransitionType type);

    // Draws one frame of the effect into `dst` (the 640x448 menu frame, y down).
    void draw_frame(SDL_Renderer* r, const SDL_FRect& dst);

private:
    bool active_ = false;
    TransitionType type_ = TransitionType::Fadeout;
    int frame_ = 0;      // frames drawn
    int wait_ = 0;       // frames before the loading indicator appears
    int blink_ = 0;      // indicator blink counter
    int delay_ = 0;      // DelayFadeout: frames before it starts drawing
    int lines_ = 0;      // Lineout: current sweep width
    std::array<float, 256> rows_{}; // Combout: per row-pair sweep position
    u32 rng_ = 0;
};

} // namespace gt3r::menu::flow
