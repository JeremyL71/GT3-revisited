#pragma once

// The course's sun glare (docs/dev/RACE.md "Sun glare"; facts from the course's Env0
// block, course file slot 32): flag 0x400 at +0x10 turns the sun on; +0x9C0 RGB,
// +0x9CC occlusion weight W, +0x9D0 glare size k (angular half-size atan k), +0x9D4 disc
// size, +0x9D8 disc gain, +0x9DC elevation / +0x9E0 azimuth (degrees; direction
// (sin a cos e, -sin e, cos a cos e) with e = -elev, a = -az), +0x9E4 style (0 =
// race/glare.img sprite, 1 star-burst, 2 glow disc), +0x9E8 occlusion box N.
// The original draws one additive screen-aligned billboard at the sun after the scene,
// coloured sun RGB x visibility (an N x N depth test at the sun pixel); no ghosts, no
// falloff towards the screen centre.

#include "base/bytes.h"
#include "render/math3d.h"

#include <SDL3/SDL_render.h>

#include <array>

namespace gt3r {
class Vfs;
}
namespace gt3r::render {
struct DrawOptions;
}

namespace gt3r::race {

struct SunParams {
    bool enabled = false;
    std::array<float, 3> rgb{};
    float weight = 0.5f, glare_size = 0, disc_size = 0, disc_gain = 0, elevation = 0, azimuth = 0;
    int style = 0;
    float test_box = 0;
    // Env0 fog (+0x60 RGB, +0x70 distance A, +0x74 minimum visibility b; b >= 1 = none)
    // and the colour-grade pass (flag 0x80: +0x980..988 curve g per channel, +0x994 saturation).
    bool has_env = false;
    std::array<float, 3> fog_rgb{};
    float fog_a = 0, fog_b = 1;
    bool grade = false;
    std::array<float, 3> grade_g{0.5f, 0.5f, 0.5f};
    float saturation = 1;
    // Car lighting (Env0 car light 0 and global ambient; GT3 lights car bodies with these and the
    // car's material: see scene::Material).
    bool car_light = false;
    std::array<float, 3> light_dir{0.3f, 0.85f, 0.35f}; // towards the light, y up
    std::array<float, 3> light_ambient{0.3f, 0.3f, 0.3f}, light_diffuse{1, 1, 1}, light_specular{0.7f, 0.7f, 0.7f};
    std::array<float, 3> global_ambient{};
    // Car shadows: +0xA60..+0xA68 colour, +0xA6C alpha scale, +0xA70 / +0xA74 elevation /
    // azimuth (degrees), +0xA78 length (shift per metre of height), +0xAB0 / +0xAB8 the light
    // shadow's brightness factor range; +0x58 headlamp pool strength (night courses).
    bool has_shadow = false;
    std::array<float, 3> shadow_rgb{};
    float shadow_alpha = 0.5f, shadow_elev = 0, shadow_az = 0, shadow_len = 1, shadow_f_lit = 1, shadow_f_shade = 1;
    float headlamp_pool = 0;
    // Scene light 0's direction (Env0 +0xAD0): the course pylons' planar shadows are projected
    // along it.
    std::array<float, 3> scene_light0_dir{0, 1, 0};
    float sky_yaw = 0, sky_pitch = 0; // Env0 +0x28 / +0x48 (degrees): the reflection map's sky dome rotation
    // Car light scaling by the road shade (a sun, b sky): s = P2 + (1 - P2) b (1 - (1 - P1)(1 - a))
    // with (P1, P2) = +0xAC8/+0xACC ambient, +0xAB4/+0xABC diffuse, +0xAC0/+0xAC4 specular.
    std::array<float, 2> shade_ambient{1, 1}, shade_diffuse{1, 1}, shade_specular{1, 1};
    // Car-light records 0..3 (Env0 +0xD0 + 0x80 n) as point-light courses use them.
    struct LampType {
        std::array<float, 3> ambient{}, diffuse{}, specular{};
        float spot_exp = 0, cutoff = 180, c0 = 1, c1 = 0, c2 = 0;
        std::array<float, 3> shadow_rgb{};
        float shadow_alpha = 1;
        float disc_size = 0;
        std::array<float, 4> disc_rgba{};
    };
    std::array<LampType, 4> lamps{};
    // Day car lighting: Env0 +0x10 bits 0..3 enable car-light records 0..3 (direction +0x00,
    // normalised; towards the light). Light 0 is the sun; many courses add a light from
    // below with a negative diffuse (e.g. (0, -0.29, 0), -0.21) that darkens the lower body.
    u32 car_light_mask = 0;
    std::array<std::array<float, 3>, 4> car_light_dir{};
    // Sets the GT3 car lighting of `o` from the enabled day car lights: light 0 as the main
    // light, the others as fills, ambient = global ambient + the enabled lights' ambients.
    void day_car_lights(render::DrawOptions& o) const;
    // Light scale (ambient, diffuse, specular) and the light-shadow factor F for road shade (a, b).
    std::array<float, 3> light_scale(float a, float b) const;
    float shadow_factor(float a, float b) const { return (shadow_f_shade + (shadow_f_lit - shadow_f_shade) * a) * b; }
    render::Vec3 direction() const; // towards the sun, y up
    static SunParams from_env0(ByteSpan env0);
};

class SunGlare {
public:
    ~SunGlare();
    // Loads race/glare.img once; `env0` = the course's slot 32.
    void load(const Vfs& vfs, SDL_Renderer* r, ByteSpan env0);
    const SunParams& params() const { return p_; }
    // Draws the glare into `rect` for a camera (view, vertical FOV); `visibility` 0..1.
    void draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, float fovy, float visibility) const;

private:
    SunParams p_;
    SDL_Texture* tex_ = nullptr;
};

} // namespace gt3r::race
