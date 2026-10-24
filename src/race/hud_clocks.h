#pragma once
// The race display's two clocks that start with every run and replay (presentation §B3, §G):
// the course name over the start and the replay mark's fades. Both advance once per race
// step (60 Hz), so they stand still while the race is paused.

namespace gt3r::race {

// The course name in capitals over the start of a live run or a replay (not the attract demo
// or the menu backdrops). Long form (runs from rest under the race rules: standing-start
// races, standing licence tests, machine tests): shown from 1.0 s to 5.0 s after the start,
// fading in over 0.3 s and out over the last 1.0 s. Short form (moving starts, the time trial,
// free run and free practice rules): 0.2 s to 2.2 s, fades 0.1 / 0.3 s. Fades are quadratic.
class TitleClock {
public:
    void start(bool long_form);
    void stop() { on_ = false; }
    void step(); // one race step
    // 0 when hidden, else the title's opacity 0..1.
    float alpha() const;

private:
    bool on_ = false;
    float t_ = 0, duration_ = 0, fade_in_ = 0, fade_out_ = 0;
};

// The replay mark's opacity (gt_logo_mini): hidden 3 s, fading in over 2 s, shown 30 s, fading
// out over 2 s, hidden 30 s, fading in ... (a 64 s cycle, linear fades), from the start of the
// replay or demonstration.
class ReplayMarkTimer {
public:
    void start() { *this = {}; }
    void step(); // one race step
    float alpha() const { return alpha_; }

private:
    int state_ = 0, frames_ = 0;
    float alpha_ = 0;
};

} // namespace gt3r::race
