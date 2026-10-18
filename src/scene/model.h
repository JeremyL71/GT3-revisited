#pragma once

// Format-independent, CPU-side 3D model. This is the contract between the
// decoders (cars, wheels, courses: src/formats/...) and the renderer
// (src/render/...): decoders fill a Model, the renderer uploads and draws it.
// Keep it plain data so tools (OBJ/glTF export) and tests can use it too.

#include "base/bytes.h"

#include <array>
#include <string>
#include <vector>

namespace gt3r::scene {

struct Vertex {
    float pos[3] = {0, 0, 0};
    float normal[3] = {0, 0, 0};
    float uv[2] = {0, 0}; // origin at the texture's top-left (GS convention)
    u32 color = 0xFFFFFFFFu; // RGBA8 (memory order R,G,B,A), alpha already expanded to 0..255
};

// Render state flags per primitive group. Extend as PS2 states are mapped.
enum PrimitiveFlags : u32 {
    kAlphaBlend = 1u << 0,   // GS ALPHA enabled (standard blend)
    kAdditive = 1u << 1,     // additive blend
    kAlphaTest = 1u << 2,    // GS TEST alpha test
    kDoubleSided = 1u << 3,  // no backface culling
    kEnvironmentMap = 1u << 4, // reflection pass (cars)
    kUnlit = 1u << 5,        // vertex colors only (pre-lit, e.g. courses)
    kNoDepthWrite = 1u << 6,
    kTintable = 1u << 7,     // material takes the car's body colour
    // With kAlphaBlend | kAlphaTest: GS TEST.AFAIL = RGB_ONLY. Fragments passing the alpha
    // test are drawn normally (depth write unless kNoDepthWrite); failing ones are still
    // blended, without depth write (course foliage: solid texels occlude, soft edges blend).
    kAlphaFailBlend = 1u << 8,
    // GS ALPHA 0x89 ((Cd - 0) * As + 0): the frame buffer is multiplied by the fragment alpha
    // (texel alpha x vertex alpha, 0x80 = 1.0); the fragment colour is not used. Prize cups
    // darken the inside of their bowls this way.
    kMultiplyDest = 1u << 9,
    // GS FBMSK 0x00FFFFFF: the pass writes only the frame-buffer alpha (course shadow masks
    // that switch the road sheen off). Drawn only for DrawItem::frame_alpha items.
    kAlphaOnly = 1u << 10,
    // GS FBMSK 0xFF000000: colour only, the frame-buffer alpha is kept (DrawItem::frame_alpha).
    kKeepAlpha = 1u << 11,
};

// Fixed-function lighting material of a lit primitive (GT3 cars: GTM1 material
// records, one set per paint colour). Lit colour = emission + light ambient x ambient
// + sum of light diffuse x diffuse x max(N.L, 0) + light specular x specular x
// max(N.H, 0)^exponent, times `scale`, clamped to 2.0 (PS2 0x80 = 1.0), then
// multiplied with the texel. Invalid = the renderer's generic lighting.
struct Material {
    float ambient[3] = {0.2f, 0.2f, 0.2f};
    float diffuse[3] = {0.8f, 0.8f, 0.8f};
    float specular[3] = {0, 0, 0};
    float emission[3] = {0, 0, 0};
    // GT3 lamp materials (flag bit 0: brake-lamp lenses): this emission is scaled at draw
    // time by the car's lamp factor (DrawItem::lamp, k/4 of the last 4 frames braking).
    float lamp_emission[3] = {0, 0, 0};
    float shininess = 0; // specular exponent
    float scale = 1.0f;  // 255/128 for GT3 packets lit at the 255 scale (chunk flag 0x100)
    bool valid = false;
};

struct Primitive {
    u32 first_index = 0;
    u32 index_count = 0; // triangle list
    int texture = -1;    // index into Model::textures, -1 = untextured
    u32 flags = 0;
    float alpha_ref = -1; // kAlphaTest: discard alpha <= this (0..1); < 0 = the renderer's default
    Material material{};
    // GT3 reflection (environment-map) packets: the vertex colour is scaled per channel by
    // env_c0 + env_c1 x min(V.N, 0) (V the unit vector from the eye, N the normal), then
    // clamped to 255/128. env_rows = false: the renderer's DrawOptions::reflection_fresnel.
    bool env_rows = false;
    float env_c0[3] = {1, 1, 1};
    float env_c1[3] = {0, 0, 0};
};

struct Mesh {
    std::string name;
    int lod = 0;  // 0 = highest detail
    std::array<float, 16> transform{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; // column-major
    std::vector<Vertex> vertices;
    std::vector<u32> indices;
    std::vector<Primitive> primitives;
};

struct Texture {
    std::string name;
    u32 width = 0, height = 0;
    std::vector<u32> rgba; // RGBA8 row-major
    bool clamp_u = false, clamp_v = false;
};

struct Model {
    std::string name;
    std::vector<Mesh> meshes;
    std::vector<Texture> textures;
    float bounds_min[3] = {0, 0, 0};
    float bounds_max[3] = {0, 0, 0};

    void compute_bounds();
};

inline void Model::compute_bounds() {
    bool first = true;
    for (const Mesh& m : meshes)
        for (const Vertex& v : m.vertices)
            for (int i = 0; i < 3; ++i) {
                if (first || v.pos[i] < bounds_min[i]) bounds_min[i] = v.pos[i];
                if (first || v.pos[i] > bounds_max[i]) bounds_max[i] = v.pos[i];
                if (i == 2) first = false;
            }
}

} // namespace gt3r::scene
