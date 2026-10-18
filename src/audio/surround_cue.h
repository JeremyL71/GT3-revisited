#pragma once

// GT3's matrix-surround cue for positional sounds (engine units and per-car
// effects): while a source is behind the listener one output channel is
// phase-inverted, which a Pro Logic style decoder steers to the rear. A
// two-flag state with hysteresis decides which side:
//   in front (z > 0):  x < 0 -> R follows L;  x >= 0 -> L follows R
//   behind  (z <= 0):  x < 0 -> R = not L;    x >= 0 -> L = not R
// (x right, z forward, both over the horizontal distance). Off by default
// in our mixer (VoiceMixer::set_surround_cue); on plain stereo it smears
// the image.

namespace gt3r {

struct SurroundCue {
    bool invert_l = false, invert_r = false;

    void update(float x_over_d, float z_over_d) {
        if (z_over_d > 0) {
            if (x_over_d < 0) invert_r = invert_l;
            else invert_l = invert_r;
        } else {
            if (x_over_d < 0) invert_r = !invert_l;
            else invert_l = !invert_r;
        }
    }
    void reset() { invert_l = invert_r = false; }
};

} // namespace gt3r
