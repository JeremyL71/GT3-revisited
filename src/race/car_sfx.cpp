#include "race/car_sfx.h"

#include "base/log.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace gt3r::race {

namespace {

constexpr float kGains[5][kChCount] = {
    // engine exhaust turbo b_off whine clash rumble in_car wind road rain squeal unused slide dirt
    {3.00f, 2.50f, 0.50f, 0.20f, 0.80f, 0.30f, 1.40f, 0.60f, 0.00f, 0.90f, 0.90f, 0.60f, 1.0f, 0.80f, 1.40f},
    {0.30f, 1.30f, 0.00f, 0.00f, 0.00f, 0.30f, 0.20f, 0.00f, 0.30f, 0.30f, 0.60f, 0.30f, 1.0f, 0.60f, 1.00f},
    {1.00f, 3.00f, 0.00f, 0.00f, 0.30f, 0.50f, 1.00f, 0.00f, 1.40f, 0.60f, 0.90f, 0.60f, 1.2f, 0.60f, 1.00f},
    {0.30f, 1.30f, 0.00f, 0.00f, 0.00f, 0.00f, 0.20f, 0.00f, 0.30f, 0.30f, 0.60f, 0.30f, 0.6f, 0.60f, 1.00f},
    {0.50f, 3.30f, 0.00f, 0.00f, 0.20f, 0.50f, 0.20f, 0.00f, 0.80f, 0.90f, 1.00f, 0.50f, 1.0f, 0.30f, 1.00f},
};

using Pt = std::pair<float, float>;

// Clamped piecewise-linear curve.
template <size_t N>
float curve(const std::array<Pt, N>& p, float x) {
    if (x <= p[0].first) return p[0].second;
    for (size_t i = 1; i < N; ++i)
        if (x <= p[i].first) {
            float t = (x - p[i - 1].first) / std::max(p[i].first - p[i - 1].first, 1e-6f);
            return p[i - 1].second + (p[i].second - p[i - 1].second) * t;
        }
    return p[N - 1].second;
}

struct Class {
    std::array<Pt, 4> a;
    std::array<Pt, 8> b;
    float offset;
    float extra_from_kmh, extra_at_100; // extra = max(0, h - v0/50) * slope, reaching extra_at_100 at 100 km/h
};

// Shipped curve records (volume, pitch) per surface class: tarmac, loose, wet.
const Class kVol[3] = {
    {{{{1.11f, 0.0f}, {9.56f, 1.19f}, {12.2f, 1.6f}, {15.0f, 2.0f}}},
     {{{-1.0f, 2.39f}, {-0.58f, 2.39f}, {-0.31f, 1.18f}, {-0.035f, 0.0f}, {0.035f, 0.0f}, {0.31f, 0.78f}, {0.58f, 1.59f}, {1.0f, 1.59f}}},
     0, 0, 0},
    {{{{1.56f, 0.0f}, {3.33f, 0.063f}, {13.1f, 0.364f}, {14.1f, 0.4f}}},
     {{{-0.50f, 0.2f}, {-0.44f, 0.18f}, {-0.125f, 0.025f}, {-0.047f, 0.0f}, {0.047f, 0.0f}, {0.11f, 0.041f}, {0.235f, 0.176f}, {0.267f, 0.2f}}},
     0.07f, 1, 0.30f},
    {{{{1.56f, 0.0f}, {3.33f, 0.047f}, {13.1f, 0.273f}, {14.1f, 0.3f}}},
     {{{-0.50f, 0.4f}, {-0.44f, 0.35f}, {-0.125f, 0.05f}, {-0.047f, 0.0f}, {0.047f, 0.0f}, {0.11f, 0.061f}, {0.235f, 0.264f}, {0.267f, 0.3f}}},
     0, 10, 0.12f},
};
const Class kPitch[3] = {
    {{{{0.44f, 0.0f}, {3.33f, 0.11f}, {6.0f, 0.227f}, {9.22f, 0.351f}}},
     {{{-1.0f, 0.461f}, {-0.659f, 0.289f}, {-0.251f, 0.113f}, {-0.016f, 0.0f}, {0.0f, 0.0f}, {0.204f, 0.088f}, {0.722f, 0.326f}, {1.0f, 0.461f}}},
     0.75f, 0, 0},
    {{{{1.11f, 0.0f}, {6.89f, 0.471f}, {18.0f, 1.54f}, {28.3f, 1.59f}}},
     {{{-0.345f, 0.8f}, {-0.298f, 0.703f}, {-0.11f, 0.1f}, {-0.047f, 0.0f}, {0.047f, 0.0f}, {0.078f, 0.163f}, {0.188f, 0.703f}, {0.282f, 0.8f}}},
     0.5f, 10, 0.10f},
    {{{{1.56f, 0.0f}, {5.11f, 0.151f}, {11.8f, 0.301f}, {28.3f, 0.293f}}},
     {{{-0.50f, 0.3f}, {-0.44f, 0.264f}, {-0.125f, 0.038f}, {-0.047f, 0.0f}, {0.047f, 0.0f}, {0.11f, 0.061f}, {0.235f, 0.264f}, {0.267f, 0.3f}}},
     0.5f, 30, 0.10f},
};

float eval(const Class& c, float v_lat, float slip, float hub_speed, bool is_volume) {
    float a = curve(c.a, v_lat), b = curve(c.b, slip);
    float v = std::sqrt(a * a + b * b);
    if (c.extra_at_100 > 0) {
        float h = std::min(1.0f, hub_speed / 50.0f), v0 = c.extra_from_kmh / 3.6f / 50.0f;
        float slope = c.extra_at_100 / std::max(100.0f / 3.6f / 50.0f - v0, 1e-3f);
        v += std::max(0.0f, h - v0) * slope;
    }
    if (!is_volume || v != 0) v += c.offset;
    return v;
}

float lerp_clamped(float x, float x0, float y0, float x1, float y1) {
    float t = std::clamp((x - x0) / (x1 - x0), 0.0f, 1.0f);
    return y0 + (y1 - y0) * t;
}

// Surface material (GTRW triangle +0x08, SurfaceHit::surface low byte) and the wet
// flag (0x02 in the flags byte) -> GT3's tyre surface index.
int surface_index(u32 raw) {
    int m = int(raw & 0xFF);
    bool wet = (raw >> 8) & 0x02;
    if (wet) {
        if (m == 0 || m == 7) return 12;
        if (m == 1) return 13;
        if (m == 9) return 14;
    }
    return m;
}

} // namespace

float sfx_gain(SoundView v, SfxChannel ch) { return kGains[int(v)][int(ch)]; }

TyreSound tyre_squeal(int cls, float v_lat, float slip, float hub_speed) {
    cls = std::clamp(cls, 0, 2);
    return {eval(kVol[cls], v_lat, slip, hub_speed, true), eval(kPitch[cls], v_lat, slip, hub_speed, false)};
}

TyreSound kerb_rumble(float load, float spd) {
    float l = std::clamp(load / 2450.0f - 0.25f, 0.0f, 1.0f);
    float vol = std::clamp((spd - 1.389f) * 0.144f, 0.0f, 1.0f) * l * 0.5f;
    return {vol, lerp_clamped(spd, 2.78f, 0.5f, 55.6f, 4.3f)};
}

TyreSound road_noise(float width, float spd) {
    float fw = lerp_clamped(width, 0.135f, 0.5f, 0.355f, 1.0f);
    return {std::min(0.3f, 2.55f * (spd / 100.0f) * (spd / 100.0f) * fw), lerp_clamped(spd, 8.33f, 0.5f, 41.7f, 1.0f)};
}

float wall_strength(float j) {
    float s = std::min(std::fabs(j) / 50000.0f, 1.0f);
    return s < 0.2f ? 0.0f : s;
}

float car_strength(float j) {
    float s = std::min(std::fabs(j) / 7500.0f, 1.0f);
    return s < 0.2f ? 0.0f : s;
}

float clash_volume(SoundView view, float strength, float distance) {
    return std::min(1.0f, sfx_gain(view, kChClash) * strength * 0.6f * 32.0f / std::max(distance, 0.25f));
}

void CarSfx::update(RaceSfx& sfx, VoiceMixer& mixer, const sim::VehicleState& s, const sim::VehicleSetup& setup,
                    SoundView view, float wall_impulse, float car_impulse, float distance) {
    // Per effect, the loudest wheel (one voice per effect instead of one per wheel).
    struct Acc {
        float vol = 0, pitch = 1;
        void add(TyreSound t, float gain) {
            if (t.vol * gain > vol) vol = t.vol * gain, pitch = t.pitch;
        }
    } squeal, slide, dirt, rumble, road, rain;
    float speed = s.speed_kmh / 3.6f;
    for (size_t w = 0; w < 4; ++w) {
        const sim::WheelState& wh = s.wheels[w];
        airborne_[w] = wh.grounded || wh.load > 0 ? 6 : std::max(0, airborne_[w] - 1);
        if (airborne_[w] == 0) continue;
        int si = surface_index(wh.surface);
        float wheel_speed = std::fabs(wh.angular_velocity) * setup.wheels[w].radius;
        float v_lat = speed * std::sin(std::min(std::fabs(wh.slip_angle), 1.5f));
        switch (si) {
        case 0: case 7: case 8: case 9:
            squeal.add(tyre_squeal(0, v_lat, wh.slip_ratio, speed), sfx_gain(view, kChSquealTarmac));
            road.add(road_noise(setup.wheels[w].width, wheel_speed), sfx_gain(view, kChRoad));
            break;
        case 1: case 13: rumble.add(kerb_rumble(wh.load, wheel_speed), sfx_gain(view, kChRumble)); break;
        case 2: case 3: case 4: slide.add(tyre_squeal(1, v_lat, wh.slip_ratio, speed), sfx_gain(view, kChSlide)); break;
        case 5: dirt.add(tyre_squeal(1, v_lat, wh.slip_ratio, speed), sfx_gain(view, kChDirt)); break;
        case 6: case 12: case 14: rain.add(tyre_squeal(2, v_lat, wh.slip_ratio, speed), sfx_gain(view, kChRain)); break;
        default: break;
        }
    }
    sfx.hold(mixer, prefix_ + "squeal", "on_skill", squeal.vol, squeal.pitch);
    sfx.hold(mixer, prefix_ + "slide", "off_skill", slide.vol, slide.pitch);
    sfx.hold(mixer, prefix_ + "dirt", "off_roadnoiz", dirt.vol, dirt.pitch);
    sfx.hold(mixer, prefix_ + "kerb", "rumble", rumble.vol, rumble.pitch);
    sfx.hold(mixer, prefix_ + "road", "on_road", road.vol, road.pitch);
    sfx.hold(mixer, prefix_ + "rain", "rain_noiz", rain.vol, rain.pitch);

    // Collisions: the strongest impact of the step, wall `clash` or car `clash2`, keyed again on
    // every step with a hit of strength 0.2 or more (no cooldown), centred. Only the followed car's
    // hits are heard; GT3 reads no other car's collisions.
    float ws = wall_strength(wall_impulse), cs = car_strength(car_impulse);
    if (ws > 0 || cs > 0) {
        bool car = cs >= ws;
        sfx.play(mixer, car ? "clash2" : "clash", clash_volume(view, car ? cs : ws, distance));
    }
}

void CarSfx::stop(RaceSfx& sfx, VoiceMixer& mixer) {
    for (const char* slot : {"squeal", "slide", "dirt", "kerb", "road", "rain"}) sfx.hold(mixer, prefix_ + slot, "", 0);
}

// --- CarExtraSounds ---------------------------------------------------------------------

bool CarExtraSounds::load(Vfs& vfs) {
    std::string err;
    if (!wind_.load(vfs, EngineUnit::kWindBank, &err)) {
        LOG_WARN("Race: wind bank: %s", err.c_str());
        return false;
    }
    return true;
}

void CarExtraSounds::step(const Step& s) {
    turbo_.step(s.turbo, s.throttle, prev_load_, s.load, s.torque, s.lag0, s.lag1);
    prev_load_ = s.load;
}

void CarExtraSounds::update(RaceSfx& sfx, VoiceMixer& mixer, const Frame& f) {
    float pan = EngineSound::pan_for_offset(f.x, f.z);
    if (wind_.loaded()) {
        car_sound::Wind w = car_sound::wind(f.u, f.slipstream, f.cd);
        wind_.update_at(mixer, float(w.rpm), sfx_gain(f.view, kChWind) * f.dist_gain * w.gain, f.x, f.z, f.doppler);
    }
    car_sound::Loop ic = car_sound::in_car(f.u, f.slipstream, f.cd);
    sfx.hold(mixer, prefix_ + "incar", car_sound::kInCar, sfx_gain(f.view, kChInCar) * f.dist_gain * ic.gain, ic.pitch,
             pan);
    car_sound::Loop wh = car_sound::whine(f.whine, f.clutch, f.torque, f.ratio, f.first_ratio, f.shaft);
    sfx.hold(mixer, prefix_ + "whine", car_sound::kWhine, sfx_gain(f.view, kChWhine) * f.dist_gain * wh.gain, wh.pitch,
             pan);
    turbo_.frame();
    car_sound::Loop sp = turbo_.spool();
    sfx.hold(mixer, prefix_ + "turbo", car_sound::kTurboSpool, sfx_gain(f.view, kChTurbo) * f.dist_gain * sp.gain,
             sp.pitch, pan);
    if (turbo_.blow_off() > 0) {
        float d = std::max(std::hypot(f.x, f.z), 0.25f); // the game does not clamp 32/d here
        sfx.play(mixer, car_sound::kBlowOff, sfx_gain(f.view, kChBlowOff) * turbo_.blow_off() * 32.0f / d, 1, pan);
    }
}

void CarExtraSounds::stop(RaceSfx& sfx, VoiceMixer& mixer) {
    wind_.stop(mixer);
    for (const char* k : {"incar", "whine", "turbo"}) sfx.hold(mixer, prefix_ + k, "", 0);
}

} // namespace gt3r::race
