#pragma once

// A 3D viewport composited into the 2D UI.
//
//   View3D view(*app.gpu());
//   view.draw(ui.renderer(), {x, y, w, h}, camera, {{&gpu_model}}, options);
//
// The rectangle is in the renderer's logical (UI) coordinates; the
// offscreen target is sized to the matching number of physical pixels, so
// the 3D image is sharp on HiDPI screens. Rendering happens immediately in
// a separate GPU command buffer; the result is drawn as an SDL_Texture, so
// UI drawn afterwards appears on top.

#include "render/camera.h"
#include "render/gpu_context.h"
#include "render/gpu_model.h"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace gt3r::render {

enum class WireMode : u8 { Off, Overlay, Only };

struct DrawOptions {
    WireMode wire = WireMode::Off;
    int lod = 0;                       // draw only meshes of this LOD; -1 = all
    std::optional<CullMode> cull;      // override per-primitive culling (debugging decoders)
    float clear[4] = {0.10f, 0.11f, 0.14f, 1.0f};
    // Opacity of the finished picture over what the 2D renderer already holds (with a clear
    // alpha of 0, the background shows where nothing was drawn).
    float composite_alpha = 1.0f;
    Vec3 light_dir{0.45f, 0.8f, 0.4f}; // towards the light, world space
    float ambient = 0.35f;
    float alpha_ref = 0.5f;            // alpha test threshold for kAlphaTest (opaque primitives)
    float blend_alpha_ref = 0.02f;     // alpha test threshold for blended kAlphaTest primitives
    float env_strength = 0.35f;        // kEnvironmentMap placeholder reflection
    float sheen_strength = 0.6f;       // kAdditive | kEnvironmentMap, textured: specular sheen (course roads)
    float reflection_strength = 0.4f;  // kAdditive | kEnvironmentMap, untextured: car env reflection
                                       // (placeholder env is brighter than the game's dark reflection maps)
    // GT3 car lighting (course Env0 light 0): lit primitives get texel x min(ambient + diffuse x
    // max(N.L, 0), 2) per channel (PS2 0x80 = 1.0, so up to 2x the texel). Off: the generic
    // ambient + (1 - ambient) x diffuse above. Primitives with a valid scene::Material always
    // use these light colours with their material (see scene::Material).
    bool gt3_lighting = false;
    float light_ambient[3] = {0.3f, 0.3f, 0.3f};
    float light_diffuse[3] = {1.0f, 1.0f, 1.0f};
    float light_specular[3] = {0.7f, 0.7f, 0.7f}; // with scene::Material specular
    // Up to three more directional lights for the GT3 lighting (the menu showroom's fills):
    // world directions towards the light, diffuse and specular colours.
    int fill_count = 0;
    float fill_dir[3][3] = {};
    float fill_diffuse[3][3] = {};
    float fill_specular[3][3] = {};
    // Environment seen by the car reflection passes (stand-in for GT3's per-frame sphere map of
    // the course): colour above, at and below the horizon; `reflection_fresnel` = GT3's C1 term,
    // weight 1 - C1 cos(view, normal) (0 = constant weight).
    float env_sky[3] = {0.35f, 0.55f, 0.85f};
    float env_horizon[3] = {0.75f, 0.78f, 0.82f};
    float env_ground[3] = {0.18f, 0.17f, 0.16f};
    float reflection_fresnel = 0;
    // Environment cube for the car reflection passes (View3D::render_env_cube); null = the
    // gradient above. Sampled with the world-space reflection vector.
    SDL_GPUTexture* env_cube = nullptr;
    // Look the cube up the way GT3 reads its view-dependent sphere map: the angle from the
    // toward-viewer direction compressed (theta' = 2 asin(0.9 sin(theta / 2))) and the result
    // tilted 15 degrees up about the camera's x axis.
    bool env_gt3_lookup = false;
    // A sphere map in eye space for the car reflection passes, used instead of the cube /
    // gradient: GT3's menu showrooms reflect a fixed image (the car_render_* widget's), looked
    // up with the reflection kernel s = 0.5 + 0.5 Rx / m, t = 0.5 - 0.5 Ry / m,
    // m = |R + (0, 0, 1)|, R the reflected view vector in view space.
    SDL_GPUTexture* env_sphere = nullptr;
    // Store every fragment at 5 bits per channel (c8 >> 3 << 3): GT3's reflection map is a
    // PSMCT16 frame buffer. Set when rendering that map into the cube, so lookups filter
    // between quantised texels as the GS does, instead of quantising the filtered colour
    // (which draws hard contour bands that sweep over the car as it moves).
    bool quantize_rgb5 = false;
    // The view or projection mirrors the image (rear-view mirror, cube faces): front faces
    // wind the other way, so back-face culling is swapped.
    bool mirrored = false;
    float wire_color[4] = {0.55f, 0.95f, 0.55f, 1.0f};
    // Large scenes (race): skip meshes whose bounding sphere is outside the
    // view frustum or further than `draw_distance` (0 = unlimited) from the eye.
    bool frustum_cull = false;
    float draw_distance = 0;
    // Draw blended primitives back to front (by mesh bounding-sphere centre)
    // instead of in model order.
    bool sort_blended = false;
    // Linear distance fog; background items are never fogged.
    bool fog = false;
    float fog_color[3] = {0.7f, 0.75f, 0.8f};
    float fog_start = 300, fog_end = 2000, fog_max = 1.0f;
    // GT3 fog curve (course Env0 +0x70 / +0x74): visibility = b + (1 - b) min(1, A / z);
    // fog_gt3 = true uses fog_start = A and fog_max = 1 - b.
    bool fog_gt3 = false;
    // Course colour grade (Env0 flag 0x80): curve g per channel (0.5 = identity) and
    // saturation in w; w = 0 turns it off.
    float grade[4] = {0.5f, 0.5f, 0.5f, 0.0f};
    // Replay colour filter levels (GT3's per-shot filter): input black / white points of the
    // curve, and the replay pass's order (saturation before the curve). Off: 0 / 1 / false.
    float grade_black = 0, grade_white = 1;
    bool grade_saturation_first = false;
    // Fixed internal resolution (e.g. the PS2's 640x448), upscaled to the rectangle;
    // 0 = the rectangle's own pixel size. `nearest` upscales without filtering.
    int fixed_width = 0, fixed_height = 0;
    bool nearest = false;
};

// Per-draw counters (last View3D::draw call).
struct DrawStats {
    int meshes_drawn = 0, meshes_culled = 0, draw_calls = 0;
    size_t triangles = 0;
};

// A per-item light set for the GT3 lighting (GT3 point-light courses light each car with
// its own up to four nearby lamps, as directional lights): replaces the DrawOptions lights.
struct ItemLights {
    int count = 0;                 // 1..4
    float ambient[3] = {0, 0, 0};  // global ambient + the lights' ambients
    float dir[4][3] = {};          // towards the light (world)
    float diffuse[4][3] = {};
    float specular[4][3] = {};
};

struct DrawItem {
    const GpuModel* model = nullptr;
    Mat4 transform;
    int lod = -2; // -2 = use DrawOptions::lod
    bool helper = false; // grids/gizmos: always drawn solid, never wireframed
    // Sky domes and backdrops: drawn first, without depth writes, culling or
    // fog, so everything else covers them.
    bool background = false;
    float draw_distance = -1; // per-item override of DrawOptions::draw_distance (< 0 = use the option)
    // < 1: the whole item is drawn translucent (alpha-blended after the opaque
    // pass, depth writes kept), e.g. a ghost car.
    float alpha = 1.0f;
    // Prize scenes (.prm loaded with gtm1::Options::static_env_maps): textured
    // kEnvironmentMap primitives sample their own texture as a sphere map,
    // u = 0.5 + 0.5 n.x, v = 0.5 - 0.5 n.y with n the view-space normal
    // (texture x vertex colour, unlit; the primitive's blend mode applies).
    // No CPU apply_sphere_map / re-upload needed.
    bool sphere_map = false;
    // Per-item lighting (GT3 cars: the light colours scaled by the road shade under the car)
    // multipliers for the ambient, diffuse and specular light colours, and a factor on the car
    // reflection passes (GT3: 1 - dirt).
    float light_scale[3] = {1, 1, 1};
    float reflection_scale = 1;
    const ItemLights* lights = nullptr; // own light set (must outlive the draw call)
    // With DrawOptions::sort_blended: this item's blended meshes are drawn in model order, before
    // the sorted ones (course scenery: the PS2 draws its packets in file order with depth writes,
    // so e.g. translucent light-shaft sheets must come after the scenery behind them).
    bool in_order = false;
    // The PS2's frame-buffer alpha (course scenery): every pass writes its fragment alpha (texel x
    // vertex alpha) unless flagged kKeepAlpha, kAlphaOnly passes write only it, and the road
    // sheen (kAdditive | kEnvironmentMap, textured) is weighted by it (GS ALPHA 0x58,
    // Cs * Ad + Cd) instead of by its own vertex alpha.
    bool frame_alpha = false;
    // GT3 car lamp factor (0..1): lamp materials add Material::lamp_emission x this.
    float lamp = 0;
    // Samples this texture (bilinear, clamped) instead of the primitives' own, e.g. an
    // offscreen buffer from View3D::render_offscreen.
    SDL_GPUTexture* texture = nullptr;
    // Blended primitives weighted by the frame-buffer alpha instead (PS2 ALPHA): 1 = 0x54,
    // (Cs - Cd) Ad + Cd; 2 = 0x58, Cs Ad + Cd. The frame alpha is kept.
    u8 dest_alpha_blend = 0;
    // Double-sided primitives culled like the rest (back faces): a car body seen from inside
    // (VR's in-car view) shows its bonnet, not the inside of its roof and windows.
    bool single_sided = false;
};

class View3D {
public:
    explicit View3D(GpuContext& ctx) : ctx_(ctx) {}
    ~View3D();
    View3D(const View3D&) = delete;
    View3D& operator=(const View3D&) = delete;

    void draw(SDL_Renderer* renderer, const SDL_FRect& rect, const Mat4& view, const Mat4& proj, const Vec3& eye,
              const std::vector<DrawItem>& items, const DrawOptions& options);
    void draw(SDL_Renderer* renderer, const SDL_FRect& rect, const OrbitCamera& camera,
              const std::vector<DrawItem>& items, const DrawOptions& options) {
        draw(renderer, rect, camera.view(), camera.projection(rect.w / std::max(rect.h, 1.0f)), camera.eye(), items,
             options);
    }

    // Renders straight into `target` (`w` x `h`, the context's colour format: a VR eye's canvas),
    // the multisampled picture resolved into it; no composite pass. False on failure.
    bool draw_to(SDL_GPUTexture* target, int w, int h, const Mat4& view, const Mat4& proj, const Vec3& eye,
                 const std::vector<DrawItem>& items, const DrawOptions& options);

    // Renders `items` into a cube map (six 90-degree views from `pos`, `size` pixels per face)
    // for the car reflection passes (DrawOptions::env_cube). `faces` = bit mask of the faces to
    // refresh (+X, -X, +Y, -Y, +Z, -Z). Returns the cube (null on failure).
    SDL_GPUTexture* render_env_cube(const Vec3& pos, const std::vector<DrawItem>& items, const DrawOptions& options,
                                    int size = 128, unsigned faces = 0x3F);
    SDL_GPUTexture* env_cube() const { return cube_; }

    // Renders `items` into offscreen buffer `slot` (0..3; `w` x `h`, cleared to `options.clear`,
    // own depth) and returns it for sampling (DrawItem::texture); null on failure.
    SDL_GPUTexture* render_offscreen(int slot, int w, int h, const Mat4& view, const Mat4& proj, const Vec3& eye,
                                     const std::vector<DrawItem>& items, const DrawOptions& options);

    // Physical size of the last target, for stats overlays.
    int pixel_width() const { return width_; }
    int pixel_height() const { return height_; }
    const DrawStats& stats() const { return stats_; }

    // GT3R_CAPTURE3D=<first>,<count>,<path.png>: copies this view's frames <first>.. on the GPU
    // and writes them as <path>-<k>.png after the last one. Unlike --shot (SDL_RenderReadPixels
    // waits for the GPU every frame) the frames stay in flight as in play: for flicker checks.
    void capture_from_env();

private:
    bool frame_alpha_written_ = false; // a DrawItem::frame_alpha pass wrote the target alpha this draw
    void capture_frame(SDL_GPUCommandBuffer* cmd);
    void write_capture();
    bool ensure_target(SDL_Renderer* renderer, int w, int h);
    void release_target();
    bool ensure_cube(int size);
    void release_cube();
    SDL_GPUTexture* placeholder_cube();
    // Records the draws of one view into an open render pass.
    void encode(SDL_GPUCommandBuffer* cmd, SDL_GPURenderPass* pass, const Mat4& view, const Mat4& proj, const Vec3& eye,
                const std::vector<DrawItem>& items, const DrawOptions& options);

    GpuContext& ctx_;
    int width_ = 0, height_ = 0, samples_ = 0;
    SDL_GPUTexture* color_ = nullptr;   // single-sample, sampled by the 2D renderer
    SDL_GPUTexture* msaa_ = nullptr;    // multisampled colour (when MSAA is on)
    SDL_GPUTexture* depth_ = nullptr;
    SDL_Texture* wrapper_ = nullptr;    // color_ as an SDL_Renderer texture
    DrawStats stats_;
    SDL_GPUTexture* cube_ = nullptr;      // environment cube (6 layers)
    // Multisampled colour, one per face (resolved into cube_'s layer): SDL 3.4's Vulkan backend
    // keys its framebuffer cache by the resolve texture's layer 0 whatever resolve_layer is, so
    // faces sharing one target got another face's framebuffer and resolved into the wrong layer.
    SDL_GPUTexture* cube_msaa_[6] = {};
    SDL_GPUTexture* cube_depth_ = nullptr;
    int cube_size_ = 0, cube_samples_ = 0;
    SDL_GPUTexture* placeholder_cube_ = nullptr; // 1x1 grey, bound when no cube is given
    struct Offscreen {
        SDL_GPUTexture *color = nullptr, *msaa = nullptr, *depth = nullptr;
        int w = 0, h = 0, samples = 0;
    };
    Offscreen offscreen_[4];
    Offscreen direct_; // draw_to's multisampled colour and depth (no colour of its own)
    void release_offscreen();
    struct Capture {
        int first = 0, count = 0, frame = 0, w = 0, h = 0;
        std::string path;
        std::vector<SDL_GPUTexture*> frames;
    } capture_;
};

} // namespace gt3r::render
