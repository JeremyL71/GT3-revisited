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
    // The DelayFadeout's wash (menus §6) after `frames` frames, as one fill over the
    // picture it started on: grey 0.1 (12.8 / 255 drawn) at alpha 0 for the first 10
    // frames, then 1 - 0.5^(frames - 10). The race exit draws it this way (presentation §C1).
    static SDL_FColor delay_fadeout_cover(int frames);

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

// What stays on screen while a race loads (menus §6): the menu's last frame (a 640 x 448
// render target owned by the menu screen, which stays under the race) with the transition
// that was running on it when the menu was left. The game keeps drawing that transition
// every frame for as long as the race takes to load (with the "still loading" square after
// 3 s); the race then appears as a cut.
struct LoadingPicture {
    SDL_Texture* frame = nullptr;
    ScreenTransition transition;
    bool valid() const { return frame != nullptr && transition.active(); }
    // Frames of the transition owed after `elapsed_ns` of wall time (60 Hz, at least one per
    // drawn frame, at most 10 s worth at once).
    static int frames_for(u64 elapsed_ns);
};
// The hand-over slot: the menu screen fills it as it starts a race, the race screen takes it.
LoadingPicture& race_loading_picture();

} // namespace gt3r::menu::flow
