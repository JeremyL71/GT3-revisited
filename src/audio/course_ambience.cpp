#include "audio/course_ambience.h"

#include <algorithm>
#include <cmath>

namespace gt3r {

namespace {

constexpr float kSlew = 1.0f / 60.0f;        // gain change per frame
constexpr float kCarWeightDecay = 0.8f;      // weight of each later car
constexpr float kNearDistance = 32.0f;       // full level inside this radius
constexpr float kSpeedOfSound = 331.45f;     // m/s
constexpr float kHalfPi = 1.5707963f;

float dot(const std::array<float, 3>& a, const std::array<float, 3>& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

} // namespace

const char* CourseAmbience::effect_name(int c) {
    static const char* names[kChannels] = {"air", "city", nullptr, "crowds", "sea"};
    return c >= 0 && c < kChannels ? names[c] : nullptr;
}

void CourseAmbience::clear() {
    for (auto& e : emitters_) e.clear();
    out_ = {};
    tunnel_ = tunnel2_ = 0;
}

void CourseAmbience::load(const sound::CrssData& data,
                          const std::function<bool(float, float, float, float*)>& track_distance, float course_length,
                          bool gated) {
    clear();
    for (int c = 0; c < kChannels; ++c) {
        for (const sound::CrssEmitter& src : data.channels[size_t(c)]) {
            Emitter e;
            e.src = src;
            e.gain = gated ? 0.0f : 1.0f;
            if (gated && src.gated() && track_distance) {
                float da = 0, db = 0;
                if (track_distance(src.gate_a[0], src.gate_a[1], src.gate_a[2], &da) &&
                    track_distance(src.gate_b[0], src.gate_b[1], src.gate_b[2], &db)) {
                    // The span is the shorter arc between the two gates; it
                    // may wrap over the start line (hi < lo).
                    float lo = std::min(da, db), hi = std::max(da, db);
                    if (hi - lo > course_length * 0.5f) std::swap(lo, hi);
                    e.lo = lo;
                    e.hi = hi;
                }
            }
            emitters_[size_t(c)].push_back(std::move(e));
        }
    }
}

void CourseAmbience::update(const std::vector<Listener>& listeners, const std::vector<float>& car_distances) {
    // Emitter gains (once per frame, independent of the listeners).
    for (auto& list : emitters_)
        for (Emitter& e : list) {
            float target = 1.0f;
            if (e.lo != e.hi) {
                target = 0;
                float w = 1.0f;
                for (float d : car_distances) {
                    bool inside = e.lo <= e.hi ? (d >= e.lo && d <= e.hi) : (d >= e.lo || d <= e.hi);
                    if (inside) target += w;
                    w *= kCarWeightDecay;
                }
                target = std::min(target, 1.0f);
            }
            if (e.gain < target) e.gain += std::min(target - e.gain, kSlew);
            else e.gain -= std::min(e.gain - target, kSlew);
        }

    out_ = {};
    tunnel_ = tunnel2_ = 0;
    for (const Listener& L : listeners) {
        for (int c = 0; c < kChannels; ++c) {
            const auto& list = emitters_[size_t(c)];
            Out o;
            float pitch_sum = 0;
            for (const Emitter& e : list) {
                float pl = 0, pr = 0, doppler = 0;
                for (const sound::CrssEmitter::Point& p : e.src.points) {
                    std::array<float, 3> d{p.x - L.pos[0], p.y - L.pos[1], p.z - L.pos[2]};
                    float dist = std::sqrt(dot(d, d));
                    if (dist == 0) dist = 1;
                    for (float& v : d) v /= dist;
                    float w = p.weight * e.gain;
                    if (dist >= kNearDistance) w /= dist / kNearDistance;
                    doppler += (kSpeedOfSound / (kSpeedOfSound - dot(d, L.velocity)) - 1.0f) * w;
                    float pan = std::clamp((dot(d, L.right) + 1.0f) * 0.5f, 0.0f, 1.0f);
                    pl += std::sin((1.0f - pan) * kHalfPi) * w;
                    pr += std::sin(pan * kHalfPi) * w;
                }
                if (!e.src.points.empty()) doppler /= float(e.src.points.size());
                o.l += pl;
                o.r += pr;
                pitch_sum += doppler;
            }
            if (!list.empty()) o.pitch = 1.0f + pitch_sum / float(list.size());
            Out& acc = out_[size_t(c)];
            acc.l += o.l;
            acc.r += o.r;
            acc.pitch += o.pitch - 1.0f;
        }
        switch (L.surface_flags & 0x18) {
        case 0x08: tunnel_ = 1.0f; break;
        case 0x10: tunnel2_ = 1.0f; break;
        default: break;
        }
    }
    for (Out& o : out_) {
        o.l = std::clamp(o.l, -1.0f, 1.0f);
        o.r = std::clamp(o.r, -1.0f, 1.0f);
        if (listeners.size() > 1) {
            o.l = o.r = std::max(o.l, o.r);
            o.pitch = 1.0f;
        }
    }
}

bool CourseAmbiencePlayer::load(const std::vector<std::shared_ptr<const sound::InstBank>>& banks,
                                const std::vector<sound::SoundEffect>& effects) {
    auto set = [&](LoopEffect& fx, const char* name, int sel) {
        int b = 0, r = 0;
        if (!name || !LoopEffect::resolve(effects, name, sel, b, r) || size_t(b) >= banks.size()) return false;
        fx.set(banks[size_t(b)], r);
        return fx.valid();
    };
    bool ok = true;
    for (int c = 0; c < CourseAmbience::kChannels; ++c)
        if (CourseAmbience::effect_name(c)) ok &= set(channels_[size_t(c)], CourseAmbience::effect_name(c), 1);
    ok &= set(tunnel_l_, "tunnel", 1);
    ok &= set(tunnel_r_, "tunnel", 2);
    ok &= set(tunnel2_l_, "tunnel2", 1);
    ok &= set(tunnel2_r_, "tunnel2", 2);
    return ok;
}

void CourseAmbiencePlayer::update(VoiceMixer& mixer, const CourseAmbience& a, float master) {
    for (int c = 0; c < CourseAmbience::kChannels; ++c) {
        if (!CourseAmbience::effect_name(c)) continue;
        const CourseAmbience::Out& o = a.channel(c);
        // Louder sounds get a lower (safer) stealing priority.
        int prio = int(std::clamp(1.0f - std::max(std::fabs(o.l), std::fabs(o.r)) * master, 0.0f, 1.0f) * 255.0f);
        channels_[size_t(c)].update(mixer, o.l * master, o.r * master, o.pitch, prio);
    }
    // The tunnel loops are stereo pairs: left half, right half.
    float t = a.tunnel() * master, t2 = a.tunnel2() * master;
    int p1 = int((1.0f - std::min(t, 1.0f)) * 255.0f), p2 = int((1.0f - std::min(t2, 1.0f)) * 255.0f);
    tunnel_l_.update(mixer, t, 0, 1, p1);
    tunnel_r_.update(mixer, 0, t, 1, p1);
    tunnel2_l_.update(mixer, t2, 0, 1, p2);
    tunnel2_r_.update(mixer, 0, t2, 1, p2);
}

void CourseAmbiencePlayer::stop(VoiceMixer& mixer) {
    for (LoopEffect& fx : channels_) fx.stop(mixer);
    tunnel_l_.stop(mixer);
    tunnel_r_.stop(mixer);
    tunnel2_l_.stop(mixer);
    tunnel2_r_.stop(mixer);
}

} // namespace gt3r
