#pragma once

// Prize scenes: "PRZ0" / "PRM0" `.prm` files (gzip in the VOL). One GTM1 model
// plus a short key-framed animation (a camera/view matrix track and a model
// matrix track at a fixed frame rate) and up to 8 directional lights. Used for
// the trophies and medals of the race results (race/gold.prm, silver, bronze,
// consolation, race/{a,g}_<difficulty>_<medal>.prm), the license card of the
// license-get screen (menu/lic_<class>.prm) and the championship trophies
// (menu/champ_NN.prm). See docs/formats/prm.md.
//
// Header (offsets are file offsets; the game relocates the pointers in place):
//   0x00 "PRZ0" | "PRM0"      0x14 f32 last frame      0x20 u16 light count
//   0x04 u32 0 (self pointer) 0x18 f32 loop frame      0x22 u16 key count
//   0x0C u32 file size or 0   0x1C f32 frames/second   0x24 u32 view keys (or 0)
//   0x10 f32 start frame                               0x28 u32 model keys (or 0)
//   0x2C u32 GTM1 offset   0x30 u32 lights (or 0)   0x34 u16 flags (bit 0: one view
//   matrix, bit 1: one model matrix)   0x38 f32, 0x3C f32 menu display parameters

#include "formats/model/gtm1.h"
#include "scene/model.h"

#include <array>
#include <string>
#include <vector>

namespace gt3r::prm {

using Matrix = gtm1::Matrix; // row-vector convention: v' = v * M, translation in row 3

struct Light {
    std::array<float, 4> direction{}; // +0x00 (w = 0); also the shadow projection direction
    std::array<float, 4> diffuse{};   // +0x10
    std::array<float, 4> ambient{};   // +0x20
    std::array<float, 4> specular{};  // +0x30 (equals diffuse in every file)
};

struct PrmFile {
    bool prm0 = false;          // "PRM0" (older layout: no light pointer)
    float start_frame = 0;      // +0x10 initial time
    float last_frame = 0;       // +0x14
    float loop_frame = 0;       // +0x18 time continues here after last_frame + 1
    float fps = 60;             // +0x1C frames per second (the game steps fps / 60 per 60 Hz tick)
    u16 flags = 0;              // +0x34
    float display[2] = {0, 0};  // +0x38, +0x3C (read by the menu trophy display; meaning open)
    std::vector<Matrix> view_keys;  // 1 matrix when static, else key_count; may be empty
    std::vector<Matrix> model_keys; // idem
    std::vector<Light> lights;
    ByteSpan gtm1;

    static bool is_prm(ByteSpan data);
    static PrmFile parse(ByteSpan data);

    // Advances a time value like the game: t += dt * fps / 60, wrapping to
    // loop_frame once past last_frame + 1 (dt in 60 Hz ticks).
    float advance(float t, float ticks = 1.0f) const;
    // Interpolated matrices at time t (frames). Rotation is interpolated with
    // a quaternion slerp, translation linearly. The key after last_frame is
    // loop_frame. Identity when the track is absent.
    Matrix view_at(float t) const;
    Matrix model_at(float t) const;
};

// Planar shadow matrix the game uses for light `l` (index i): projects onto the
// plane y = -0.1 * (i + 1) along the light direction (straight down when the
// direction's |y| < 1e-4). Returns false when y is exactly 0 (no shadow drawn).
// Apply as v * model * shadow * view; the game draws it with GTM1 shadow mode.
bool shadow_matrix(const Light& l, int index, Matrix& out);

// Decodes the embedded GTM1 into a Model in the model's local space (apply
// model_at(t), then view_at(t) to place it; the game's camera looks down -Z
// with a 60 degree horizontal field of view). gtm1::Options::static_env_maps
// is always set: kEnvironmentMap primitives with a texture are sphere-mapped
// (see apply_sphere_map); kAdditive ones add to the frame buffer. Decodes
// node 0 (the visible model) unless opt.node says otherwise; node 1, present in
// some files, is the shadow silhouette (see load_prm_shadow).
scene::Model load_prm_model(ByteSpan data, const gtm1::Options& opt = {}, const std::string& name = {});

// The shadow node (GTM1 node 1), drawn once per light through shadow_matrix().
// Empty model when the file has none (the race trophies have no shadow node).
scene::Model load_prm_shadow(ByteSpan data, const std::string& name = {});

// a * b for row-vector matrices (apply a first): model_view = multiply(model_at(t), view_at(t)).
Matrix multiply(const Matrix& a, const Matrix& b);

// Writes sphere-map UVs into the vertices of the textured kEnvironmentMap
// primitives: u = 0.5 + 0.5 * n.x, v = 0.5 - 0.5 * n.y with n the vertex normal
// rotated into view space by `model_view`. CPU stand-in for what the PS2 VU
// program does per frame; a shader can do the same from the normal.
void apply_sphere_map(scene::Model& model, const Matrix& model_view);

} // namespace gt3r::prm
