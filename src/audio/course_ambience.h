#pragma once

// Course ambience as GT3 runs it (facts in docs/formats/sound.md "Course
// ambience"): the CRSS emitters of a course drive four looping effects of
// sound/gt3se02.ins (air, city, crowds, sea), and the surface flags under
// each listener switch the two tunnel loops. Evaluated once per game frame.
//
// Each emitter has a gain that slews by 1/60 per frame towards its target:
// 1 for ungated emitters, and for gated ones (crowds along a stretch of
// track) the sum over the race's cars of 0.8^index for every car between
// the gates, capped at 1. Each point adds weight * gain, attenuated by
// 32/d beyond 32 m, panned with sin((1 - p) * pi/2) / sin(p * pi/2) where
// p = (dot(dir, listener_right) + 1) / 2, and a Doppler pitch offset
// c / (c - dot(dir, listener_velocity)) - 1 (c = 331.45 m/s) averaged over
// the points and the emitters of the channel. Channel L/R are clamped to
// 1; with more than one listener both sides take the larger value and the
// pitch is 1.

#include "audio/loop_effect.h"
#include "formats/sound/crss.h"

#include <array>
#include <functional>
#include <vector>

namespace gt3r {

class CourseAmbience {
public:
    enum Channel : int { Air = 0, City, Unused, Crowds, Sea, kChannels };
    // se.inf name of a channel's loop (nullptr for the unused channel 2).
    static const char* effect_name(int channel);

    struct Listener {
        std::array<float, 3> pos{};
        std::array<float, 3> velocity{}; // m/s
        std::array<float, 3> right{};    // unit vector to the listener's right
        u32 surface_flags = 0;           // road surface attribute under the listener
    };
    struct Out {
        float l = 0, r = 0, pitch = 1;
    };

    // `track_distance(x, y, z, &d)` returns false when the point is not on
    // the road; `course_length` is the lap length in the same units. With
    // gating off (`gated` false) every emitter plays at full gain from the
    // start, as the game does for its ungated mode.
    void load(const sound::CrssData& data, const std::function<bool(float, float, float, float*)>& track_distance,
              float course_length, bool gated = true);
    void clear();

    // One game frame. `car_distances` lists the track distance of every car
    // in the race (in entry order).
    void update(const std::vector<Listener>& listeners, const std::vector<float>& car_distances);

    const Out& channel(int c) const { return out_[size_t(c)]; }
    float tunnel() const { return tunnel_; }   // "tunnel" loop gain (flags & 0x18 == 0x08)
    float tunnel2() const { return tunnel2_; } // "tunnel2" loop gain (flags & 0x18 == 0x10)

private:
    struct Emitter {
        sound::CrssEmitter src;
        float lo = 0, hi = 0; // gate track distances (equal = ungated)
        float gain = 0;
    };
    std::array<std::vector<Emitter>, kChannels> emitters_;
    std::array<Out, kChannels> out_{};
    float tunnel_ = 0, tunnel2_ = 0;
};

// Plays a CourseAmbience through looping effects (the game's SE handles).
class CourseAmbiencePlayer {
public:
    // banks: the three se.inf banks (index = se.inf bank number).
    bool load(const std::vector<std::shared_ptr<const sound::InstBank>>& banks,
              const std::vector<sound::SoundEffect>& effects);
    // master: the SE master volume (1.0 in the game).
    void update(VoiceMixer& mixer, const CourseAmbience& a, float master = 1.0f);
    void stop(VoiceMixer& mixer);

private:
    std::array<LoopEffect, CourseAmbience::kChannels> channels_;
    LoopEffect tunnel_l_, tunnel_r_, tunnel2_l_, tunnel2_r_;
};

} // namespace gt3r
