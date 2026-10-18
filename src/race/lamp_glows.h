#pragma once

// Course point-light glows (street and stadium lamps): GTFX, course file slot 29, with
// the lamp types in Env0 +0xC60 (docs/formats/course.md). Per lamp (at most 100 per
// frame): view-space position; culled behind the camera, beyond the type's max
// distance or off screen; intensity I = 1 / (c + l z + q z^2) x directional factor
// max(0, dot(facing, to-camera))^n x blink 0.5 + 0.5 sin(2 pi (t / P + phase));
// visibility from an occlusion test at the lamp; an additive screen-aligned glow
// (race/flare.bin: a 0.425 m radial disc with thin streaks, scaled by glow size x 10)
// plus a halo disc (halo size, colour RGB x I x gain x 0.5). The flare.bin meshes are
// drawn here as an equivalent procedural sprite.

#include "base/bytes.h"
#include "render/math3d.h"

#include <SDL3/SDL_render.h>

#include <array>
#include <functional>
#include <vector>

namespace gt3r::race {

struct LampType {
    std::array<float, 3> rgb{};
    float intensity = 0, glow = 0, halo = 0, halo_gain = 0, exponent = 0;
    float att_c = 1, att_l = 0, att_q = 0, max_dist = 0, box = 0, blink = 0;
};

struct Lamp {
    int type = 0;
    render::Vec3 pos, dir;
    float phase = 0;
};

class LampGlows {
public:
    ~LampGlows();
    // `gtfx` = slot 29, `env0` = slot 32 (lamp types; flag 0x200000 turns the lamps on).
    void load(SDL_Renderer* r, ByteSpan gtfx, ByteSpan env0);
    size_t count() const { return lamps_.size(); }
    void tick(double seconds) { time_ += seconds; }
    // `blocked(from, to)`: true if the segment is occluded.
    void draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, const render::Vec3& eye, float fovy,
              const std::function<bool(const render::Vec3&, const render::Vec3&)>& blocked) const;

private:
    std::vector<Lamp> lamps_;
    std::array<LampType, 8> types_{};
    SDL_Texture* glow_ = nullptr;
    SDL_Texture* disc_ = nullptr;
    double time_ = 0;
};

// Car lamp glows (GT3's car lamp pass; on courses with Env0 flag 0x8000,
// docs/dev/RACE.md "Car lamps"). Lamps come from the car file's GTCI lists: +0x40 headlamps
// (always), +0x48 tail lamps (a halo only), +0x50 brake lamps (while braking: flag bit 0
// at k/4, others at 1 while the pedal is down now). One lamp type in Env0 +0x9F0.
struct CarLampRecord {
    render::Vec3 xaxis, view_axis, pos, test;
    float a = 0, b = 0;
    std::array<float, 3> rgb{};
    int exponent = 0;
    bool fade = false; // brake list: scaled by k/4
};
struct CarLampSet {
    std::vector<CarLampRecord> head, tail, brake;
    // `gtci` = the car file's GTCI block.
    static CarLampSet parse(ByteSpan gtci);
    bool empty() const { return head.empty() && tail.empty() && brake.empty(); }
};

class CarLampGlows {
public:
    ~CarLampGlows();
    void load(SDL_Renderer* r, ByteSpan env0);
    bool active() const { return active_; }
    struct Car {
        const CarLampSet* lamps = nullptr;
        render::Mat4 model; // car (model) frame -> world
        float k = 0;        // lamp factor k/4
        bool brake_now = false;
    };
    void draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, const render::Vec3& eye, float fovy,
              const std::vector<Car>& cars,
              const std::function<bool(const render::Vec3&, const render::Vec3&)>& blocked) const;

private:
    bool active_ = false;
    LampType type_;
    SDL_Texture* glow_ = nullptr;
    SDL_Texture* disc_ = nullptr;
};

} // namespace gt3r::race
