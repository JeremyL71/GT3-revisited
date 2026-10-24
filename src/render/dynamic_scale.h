#pragma once

// Dynamic 3D resolution ([video] dynamic_resolution): lowers the 3D render scale when frames
// miss the target while the time goes to waiting on the GPU, and steps back up towards the
// configured scale (the 3D resolution setting) while frames keep up. The target is 60 fps or,
// with vsync, the first whole number of display refreshes at or above 1/60 s (45 fps on a
// 90 Hz phone, whose frames last 11.1, 22.2 or 33.3 ms). Fed every frame; decides once a
// second, in 5% steps between 50% and the configured scale; the 5 s after a load or pause are
// not judged (a Galaxy A15 runs its first race seconds at half speed). Calm on purpose: every
// change is a visible twitch, and a frame that misses its refresh is a visible judder (33 ms on
// a 60 Hz phone). So it steps down when frames keep missing (3 a second) as well as when they
// are slow on average, waits a second after a step down, and steps up only after 5 seconds
// without a single miss; a step up that misses is undone and not tried again for a minute (two,
// then three minutes after more misses).

namespace gt3r::render {

class DynamicScale {
public:
    static constexpr float kMin = 0.5f, kStep = 0.05f;

    // Starts over at `start_scale` (default: `max_scale`), between 50% and `max_scale`, the most it
    // goes up to; `refresh_hz` = the display's (0: unknown).
    void reset(float max_scale, float refresh_hz, float start_scale = 0);
    // One frame: its wall time and the part spent presenting (waiting for the GPU or the
    // display). Returns the scale to render at.
    float frame(double frame_ms, double present_ms);
    float scale() const { return scale_; }
    float max_scale() const { return max_; }
    double target_ms() const { return target_ms_; }

private:
    double target_ms_ = 1000.0 / 60.0;
    float max_ = 1, scale_ = 1, before_probe_ = 1;
    double window_ms_ = 0, present_ms_ = 0;
    int frames_ = 0, missed_ = 0;            // this window's frames; those over a refresh late
    int good_windows_ = 0, probe_after_ = 5; // windows without a miss before a step up
    float ceiling_ = 2;                      // a step up that failed: not tried again until...
    int ceiling_windows_ = 0;                // ...this many windows have passed
    bool probed_ = false;                    // the last change was a step up
    int settle_ = 0;                         // windows not judged after a load or pause
};

} // namespace gt3r::render
