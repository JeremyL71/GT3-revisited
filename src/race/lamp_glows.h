#pragma once

// Course point-light glows (street and stadium lamps): GTFX, course file slot 29, with
// the lamp types in Env0 +0xC60 (docs/formats/course.md "GTFX (slot 29)", docs/dev/RACE.md
// "Lamp glows"). GTFX holds one lamp list per slot 1 scenery node; each frame the game
// visits the lists of the nodes visible from the camera (the slot 2 Vls0 list of the view
// point nearest the camera) in that list's order, and keeps at most 100 lamps that pass the
// screen test: in front, within the type's max distance, the halo-sized square on screen.
// Per kept lamp: directional factor D = max(0, facing . normalize(camera -> lamp))^n with
// n = int(exponent + 0.5) (D = 1 for n = 0; a negative dot product drops the glow, not the
// halo), attenuation 1 / (c + l z + q z^2), blink 0.5 + 0.5 sin(2 pi (t / P + phase)).
// The glow is race/flare.bin (three untextured meshes: a 0.425 m disc and three thin rays
// 60 degrees apart, the six-point star) in view space at the lamp pulled 0.5 m towards the
// camera, scaled by glow size x 10, not depth-tested: vertex RGB x colour (RGB x intensity x
// attenuation x blink x D, clamped to 255), vertex alpha x visibility, added as Cs As / 128.
// Visibility is the GS depth test of a 10 x 10 pixel box at the lamp 0.2 m nearer than it,
// 2 x the passing fraction, capped at 1 (a ray grid against the course occluder here). The
// halo is a depth-tested disc of the halo size, colour x gain x 0.5, drawn for every kept
// lamp further than 1 m (the original's halo mesh is a procedural 72-spike burst).

#include "base/bytes.h"
#include "formats/course/visibility.h"
#include "render/math3d.h"

#include <SDL3/SDL_render.h>

#include <array>
#include <functional>
#include <vector>

namespace gt3r::race {

// race/flare.bin, the glow mesh of both lamp passes: three untextured meshes in view space
// (a 0.425 m disc and three thin rays 60 degrees apart: the six-point star), drawn at the
// lamp, scaled by the glow size, without depth test. The colour arithmetic is the original's
// vertex program: vertex RGB x colour (clamped to 255), vertex alpha x visibility; the GS
// adds Cs As / 128.
class Flare {
public:
    Flare() = default;
    Flare(const Flare&) = delete;
    Flare& operator=(const Flare&) = delete;
    ~Flare();
    struct Vertex {
        float x = 0, y = 0, z = 0; // metres at scale 1
        u8 rgba[4] = {};           // raw GS colour (0x80 = 1.0)
    };
    struct Mesh {
        std::vector<Vertex> verts;
        std::vector<int> tris; // three vertex indices each
    };
    // Empty (with a warning) when the file is missing or malformed.
    static std::vector<Mesh> parse(ByteSpan flare);
    void load(ByteSpan flare) { meshes_ = parse(flare); }
    bool empty() const { return meshes_.empty(); }
    // Appends the meshes centred at view-space (x, y) at depth `d` in front of the camera,
    // scaled by `s`, coloured `col` with visibility `vis` (0..1).
    void add(std::vector<SDL_Vertex>& verts, std::vector<int>& idx, const render::Frustum& fr, const SDL_FRect& rect,
             float x, float y, float d, float s, const std::array<float, 3>& col, float vis) const;
    // Draws a batch built for `rect`: Cs As / 128 + Cd, i.e. two passes of SDL's additive
    // Cs As / 255 + Cd. With `lines` > 0 the batch is drawn into a layer of that many rows over
    // `rect` (columns at the PS2 frame's pixel aspect, 640 x 448 shown at 4:3) and added to
    // the picture with bilinear filtering, so the thin rays look (and break up) as they do at
    // the original's resolution; 0 draws straight at the output resolution.
    void submit(SDL_Renderer* r, const SDL_FRect& rect, float lines, std::vector<SDL_Vertex>& verts,
                const std::vector<int>& idx) const;

private:
    std::vector<Mesh> meshes_;
    mutable SDL_Texture* layer_ = nullptr;
    mutable int layer_w_ = 0, layer_h_ = 0;
};

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
    // `gtfx` = slot 29, `env0` = slot 32 (lamp types; flag 0x200000 turns the lamps on),
    // `vls0` = slot 2 (which lamp lists to visit), `flare` = race/flare.bin (empty: a
    // procedural stand-in sprite).
    void load(SDL_Renderer* r, ByteSpan gtfx, ByteSpan env0, ByteSpan vls0, ByteSpan flare);
    size_t count() const { return count_; }
    void tick(double seconds) { time_ += seconds; }
    // `blocked(from, to)`: true if the segment is occluded. `lines`: the flare layer's rows
    // over `rect` (Flare::submit; 0 = output resolution).
    void draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, const render::Vec3& eye, float fovy,
              const std::function<bool(const render::Vec3&, const render::Vec3&)>& blocked, float lines = 0) const {
        draw(r, rect, view, eye, render::Frustum::symmetric(fovy, rect.w / rect.h), blocked, lines);
    }
    void draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, const render::Vec3& eye,
              const render::Frustum& fr,
              const std::function<bool(const render::Vec3&, const render::Vec3&)>& blocked, float lines = 0) const;

private:
    std::vector<std::vector<Lamp>> lists_; // GTFX lists, one per slot 1 node
    size_t count_ = 0;
    std::array<LampType, 8> types_{};
    course::Visibility vis_;
    Flare flare_;
    SDL_Texture* glow_ = nullptr; // stand-in when race/flare.bin is missing
    SDL_Texture* disc_ = nullptr;
    double time_ = 0;
};

// Car lamp glows (GT3's car lamp pass; on courses with Env0 flag 0x8000,
// docs/dev/RACE.md "Car lamps"). Lamps come from the car file's GTCI lists, visited per car
// as headlamps (+0x40, always), brake lamps (+0x50, while braking: flag bit 0 at k/4, others
// at 1 while the pedal is down now), tail lamps (+0x48); at most 100 records pass the screen
// test (the test point in front, within the max distance, a square of halo x max(a, b) on
// screen). One lamp type in Env0 +0x9F0 (colour x intensity, glow x 8, halo, gain,
// attenuation, max distance). Round head and brake lamps (|a - b| < 0.01) get the flare
// glow at the lamp, pulled along the viewing axis (depth + 0.5 x its view-space z), scaled by
// glow x a, colour lamp RGB x type colour x attenuation x D x I with D = (viewing axis .
// normalize(test point - camera))^n (a negative product: no glow) and visibility from the
// occlusion box at the test point. Every kept record gets a depth-tested halo, colour x gain
// x 0.5 (not D or I); bar lamps an elliptical one along their projected x axis.
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
    // `env0` = slot 32, `flare` = race/flare.bin (empty: a procedural stand-in sprite).
    void load(SDL_Renderer* r, ByteSpan env0, ByteSpan flare);
    bool active() const { return active_; }
    struct Car {
        const CarLampSet* lamps = nullptr;
        render::Mat4 model; // car (model) frame -> world
        float k = 0;        // lamp factor k/4
        bool brake_now = false;
    };
    // `lines`: as LampGlows::draw.
    void draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, const render::Vec3& eye, float fovy,
              const std::vector<Car>& cars,
              const std::function<bool(const render::Vec3&, const render::Vec3&)>& blocked, float lines = 0) const {
        draw(r, rect, view, eye, render::Frustum::symmetric(fovy, rect.w / rect.h), cars, blocked, lines);
    }
    void draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, const render::Vec3& eye,
              const render::Frustum& fr, const std::vector<Car>& cars,
              const std::function<bool(const render::Vec3&, const render::Vec3&)>& blocked, float lines = 0) const;

private:
    bool active_ = false;
    LampType type_;
    Flare flare_;
    SDL_Texture* glow_ = nullptr; // stand-in when race/flare.bin is missing
    SDL_Texture* disc_ = nullptr;
};

} // namespace gt3r::race
