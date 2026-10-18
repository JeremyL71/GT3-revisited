#include "render/view3d.h"

#include "base/file.h"
#include "base/log.h"
#include "formats/png_writer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#define GT3R_HAVE_GPU_RENDERER SDL_VERSION_ATLEAST(3, 4, 0)

namespace gt3r::render {

namespace {

// Uniform blocks; layouts match model.vert / model.frag (std140).
struct VertexUniforms {
    float mvp[16];
    float model[16];
    float normal_mat[16];
};
// model.vert's VertexLighting (slot 1): GT3's VU lighting per vertex.
struct VertexLighting {
    float eye_dir[4]; // towards the viewer (world), w = 1: on
    float light_dir[4][4];
    float light_diffuse[4][4];
    float light_specular[4][4];
    float light_ambient[4];
    float mat_ambient[4];
    float mat_diffuse[4];
    float mat_specular[4];
    float mat_emission[4];
};
struct FragmentUniforms {
    float light_dir[4];
    float camera_pos[4];
    float params[4]; // alpha ref, unlit, env strength, flat colour
    float flat_color[4];
    float fog_color[4];  // rgb, w = enabled
    float fog_params[4]; // start, 1 / (end - start), max amount
    float tint[4];       // final colour multiplier
    float view_row0[4];  // rows 0 and 1 of the view matrix (sphere-mapped primitives)
    float view_row1[4];
    float grade[4];      // colour-grade curve g (rgb), saturation (w; 0 = off)
    float light_ambient[4]; // rgb, w: 1 = GT3 car lighting
    float light_diffuse[4]; // rgb
    float env_sky[4];       // rgb, w: reflection Fresnel C1
    float env_horizon[4];
    float env_ground[4];
    float light_specular[4]; // rgb
    float mat_ambient[4];    // rgb, w: 1 = material valid
    float mat_diffuse[4];    // rgb, w: scale
    float mat_specular[4];   // rgb, w: exponent (0 = none)
    float mat_emission[4];
    float light2_dir[4]; // w: 1 = enabled
    float light2_diffuse[4];
    float light2_specular[4];
    float light3_dir[4];
    float light3_diffuse[4];
    float light3_specular[4];
    float light4_dir[4];
    float light4_diffuse[4];
    float light4_specular[4];
    float grade2[4]; // x black, y white, z 1 = saturation before the curve, w 1 = alpha = coverage
    float env_c0[4]; // reflection weight rows (scene::Primitive::env_c0/c1); c0.w = 1: valid
    float env_c1[4];
    float output[4]; // x = 1: quantise the colour to 5 bits per channel (DrawOptions::quantize_rgb5)
};

} // namespace

View3D::~View3D() {
    release_target();
    release_cube();
    release_offscreen();
    if (placeholder_cube_) SDL_ReleaseGPUTexture(ctx_.device(), placeholder_cube_);
}

void View3D::release_target() {
    if (wrapper_) SDL_DestroyTexture(wrapper_);
    wrapper_ = nullptr;
    SDL_GPUDevice* dev = ctx_.device();
    for (SDL_GPUTexture** t : {&color_, &msaa_, &depth_}) {
        if (*t) SDL_ReleaseGPUTexture(dev, *t);
        *t = nullptr;
    }
    width_ = height_ = 0;
}

bool View3D::ensure_target(SDL_Renderer* renderer, int w, int h) {
    if (w == width_ && h == height_ && samples_ == ctx_.msaa() && wrapper_) return true;
    release_target();
    SDL_GPUDevice* dev = ctx_.device();
    SDL_GPUTextureCreateInfo ti{};
    ti.type = SDL_GPU_TEXTURETYPE_2D;
    ti.width = u32(w);
    ti.height = u32(h);
    ti.layer_count_or_depth = 1;
    ti.num_levels = 1;

    ti.format = GpuContext::kColorFormat;
    ti.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    ti.sample_count = SDL_GPU_SAMPLECOUNT_1;
    color_ = SDL_CreateGPUTexture(dev, &ti);
    if (ctx_.sample_count() != SDL_GPU_SAMPLECOUNT_1) {
        ti.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
        ti.sample_count = ctx_.sample_count();
        msaa_ = SDL_CreateGPUTexture(dev, &ti);
    }
    ti.format = ctx_.depth_format();
    ti.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
    ti.sample_count = ctx_.sample_count();
    depth_ = SDL_CreateGPUTexture(dev, &ti);
    if (!color_ || !depth_ || (ctx_.sample_count() != SDL_GPU_SAMPLECOUNT_1 && !msaa_)) {
        LOG_WARN("3D target %dx%d creation failed: %s", w, h, SDL_GetError());
        release_target();
        return false;
    }
#if GT3R_HAVE_GPU_RENDERER
    SDL_PropertiesID props = SDL_CreateProperties();
    SDL_SetNumberProperty(props, SDL_PROP_TEXTURE_CREATE_FORMAT_NUMBER, SDL_PIXELFORMAT_ABGR8888);
    SDL_SetNumberProperty(props, SDL_PROP_TEXTURE_CREATE_ACCESS_NUMBER, SDL_TEXTUREACCESS_STATIC);
    SDL_SetNumberProperty(props, SDL_PROP_TEXTURE_CREATE_WIDTH_NUMBER, w);
    SDL_SetNumberProperty(props, SDL_PROP_TEXTURE_CREATE_HEIGHT_NUMBER, h);
    SDL_SetPointerProperty(props, SDL_PROP_TEXTURE_CREATE_GPU_TEXTURE_POINTER, color_);
    wrapper_ = SDL_CreateTextureWithProperties(renderer, props);
    SDL_DestroyProperties(props);
#else
    (void)renderer;
#endif
    if (!wrapper_) {
        LOG_WARN("Could not wrap the 3D target for the 2D renderer: %s", SDL_GetError());
        release_target();
        return false;
    }
    SDL_SetTextureBlendMode(wrapper_, SDL_BLENDMODE_NONE);
    SDL_SetTextureScaleMode(wrapper_, SDL_SCALEMODE_LINEAR); // render_scale < 1 upscales

    width_ = w;
    height_ = h;
    samples_ = ctx_.msaa();
    return true;
}

void View3D::draw(SDL_Renderer* renderer, const SDL_FRect& rect, const Mat4& view, const Mat4& proj, const Vec3& eye,
                  const std::vector<DrawItem>& items, const DrawOptions& opt) {
    // Logical (UI) rectangle -> physical pixels.
    float sx = 1, sy = 1;
    int lw = 0, lh = 0;
    SDL_RendererLogicalPresentation mode;
    SDL_FRect lr;
    if (SDL_GetRenderLogicalPresentation(renderer, &lw, &lh, &mode) && lw > 0 && lh > 0 &&
        SDL_GetRenderLogicalPresentationRect(renderer, &lr)) {
        sx = lr.w / float(lw);
        sy = lr.h / float(lh);
    }
    sx *= ctx_.render_scale(); // < 1: render smaller, the 2D renderer upscales it
    sy *= ctx_.render_scale();
    int pw = std::max(1, int(std::lround(rect.w * sx))), ph = std::max(1, int(std::lround(rect.h * sy)));
    if (opt.fixed_width > 0 && opt.fixed_height > 0) pw = opt.fixed_width, ph = opt.fixed_height;
    if (!ensure_target(renderer, pw, ph)) return;
    SDL_SetTextureScaleMode(wrapper_, opt.nearest ? SDL_SCALEMODE_NEAREST : SDL_SCALEMODE_LINEAR);

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(ctx_.device());
    if (!cmd) return;

    SDL_GPUColorTargetInfo ct{};
    ct.clear_color = {opt.clear[0], opt.clear[1], opt.clear[2], opt.clear[3]};
    ct.load_op = SDL_GPU_LOADOP_CLEAR;
    ct.cycle = true;
    if (msaa_) {
        ct.texture = msaa_;
        ct.store_op = SDL_GPU_STOREOP_RESOLVE;
        ct.resolve_texture = color_;
        ct.cycle_resolve_texture = true;
    } else {
        ct.texture = color_;
        ct.store_op = SDL_GPU_STOREOP_STORE;
    }
    SDL_GPUDepthStencilTargetInfo dt{};
    dt.texture = depth_;
    dt.clear_depth = 1.0f;
    dt.load_op = SDL_GPU_LOADOP_CLEAR;
    dt.store_op = SDL_GPU_STOREOP_DONT_CARE;
    dt.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    dt.cycle = true;
    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &ct, 1, &dt);
    frame_alpha_written_ = false;
    encode(cmd, pass, view, proj, eye, items, opt);
    SDL_EndGPURenderPass(pass);
    if (capture_.count > 0) capture_frame(cmd);
    SDL_SubmitGPUCommandBuffer(cmd);
    if (capture_.count > 0 && capture_.frame == capture_.first + capture_.count) write_capture();

    // Opaque pictures replace what is behind (texel alpha is reflectivity, not coverage);
    // a transparent clear or a partial composite blends over it.
    bool over = opt.clear[3] < 1.0f || opt.composite_alpha < 1.0f;
    SDL_SetTextureBlendMode(wrapper_, over ? SDL_BLENDMODE_BLEND : SDL_BLENDMODE_NONE);
    // DrawItem::frame_alpha left the PS2-style frame alpha in the target: copy the colour only
    // (the 2D frame keeps its own opaque alpha).
    if (!over && frame_alpha_written_) {
        static const SDL_BlendMode kColourOnly =
            SDL_ComposeCustomBlendMode(SDL_BLENDFACTOR_ONE, SDL_BLENDFACTOR_ZERO, SDL_BLENDOPERATION_ADD,
                                       SDL_BLENDFACTOR_ZERO, SDL_BLENDFACTOR_ONE, SDL_BLENDOPERATION_ADD);
        SDL_SetTextureBlendMode(wrapper_, kColourOnly);
    }
    SDL_SetTextureAlphaModFloat(wrapper_, std::clamp(opt.composite_alpha, 0.0f, 1.0f));
    SDL_RenderTexture(renderer, wrapper_, nullptr, &rect);
    SDL_SetTextureAlphaModFloat(wrapper_, 1.0f);
}

void View3D::encode(SDL_GPUCommandBuffer* cmd, SDL_GPURenderPass* pass, const Mat4& view, const Mat4& proj,
                    const Vec3& eye, const std::vector<DrawItem>& items, const DrawOptions& opt) {
    const Mat4 view_proj = proj * view;
    // Wireframe lines are pulled slightly towards the camera to avoid z-fighting.
    Mat4 line_proj = proj;
    for (int c = 0; c < 4; ++c) line_proj.at(2, c) -= 1e-4f * proj.at(3, c);
    const Mat4 line_view_proj = line_proj * view;

    // Frustum planes (clip depth 0..1): w+x, w-x, w+y, w-y, z, w-z.
    float planes[6][4];
    for (int i = 0; i < 4; ++i) {
        float r0 = view_proj.at(0, i), r1 = view_proj.at(1, i), r2 = view_proj.at(2, i), r3 = view_proj.at(3, i);
        planes[0][i] = r3 + r0, planes[1][i] = r3 - r0, planes[2][i] = r3 + r1;
        planes[3][i] = r3 - r1, planes[4][i] = r2, planes[5][i] = r3 - r2;
    }
    for (auto& p : planes) {
        float l = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        if (l > 0)
            for (float& v : p) v /= l;
    }

    Vec3 l = normalize(opt.light_dir);
    FragmentUniforms fu{};
    fu.light_dir[0] = l.x, fu.light_dir[1] = l.y, fu.light_dir[2] = l.z, fu.light_dir[3] = opt.ambient;
    fu.camera_pos[0] = eye.x, fu.camera_pos[1] = eye.y, fu.camera_pos[2] = eye.z;
    for (int i = 0; i < 4; ++i) fu.flat_color[i] = opt.wire_color[i];
    for (int i = 0; i < 3; ++i) fu.fog_color[i] = opt.fog_color[i];
    fu.fog_params[0] = opt.fog_start;
    fu.fog_params[1] = 1.0f / std::max(opt.fog_end - opt.fog_start, 1e-3f);
    fu.fog_params[2] = opt.fog_max;
    fu.fog_params[3] = opt.fog_gt3 ? 1.0f : 0.0f;
    for (int i = 0; i < 4; ++i) fu.grade[i] = opt.grade[i];
    fu.grade2[0] = opt.grade_black, fu.grade2[1] = opt.grade_white, fu.grade2[2] = opt.grade_saturation_first ? 1.0f : 0.0f;
    for (int i = 0; i < 3; ++i) {
        fu.light_ambient[i] = opt.light_ambient[i], fu.light_diffuse[i] = opt.light_diffuse[i];
        fu.env_sky[i] = opt.env_sky[i], fu.env_horizon[i] = opt.env_horizon[i], fu.env_ground[i] = opt.env_ground[i];
    }
    for (int i = 0; i < 3; ++i) fu.light_specular[i] = opt.light_specular[i];
    for (int k = 0; k < 3 && k < opt.fill_count; ++k) {
        float* d = k == 0 ? fu.light2_dir : k == 1 ? fu.light3_dir : fu.light4_dir;
        float* df = k == 0 ? fu.light2_diffuse : k == 1 ? fu.light3_diffuse : fu.light4_diffuse;
        float* sp = k == 0 ? fu.light2_specular : k == 1 ? fu.light3_specular : fu.light4_specular;
        for (int i = 0; i < 3; ++i) d[i] = opt.fill_dir[k][i], df[i] = opt.fill_diffuse[k][i], sp[i] = opt.fill_specular[k][i];
        d[3] = 1;
    }
    fu.light_ambient[3] = opt.gt3_lighting ? 1.0f : 0.0f;
    fu.env_sky[3] = opt.reflection_fresnel;
    SDL_GPUTexture* env_tex = opt.env_cube ? opt.env_cube : placeholder_cube();
    fu.env_ground[3] = opt.env_sphere ? 3.0f : opt.env_cube ? (opt.env_gt3_lookup ? 2.0f : 1.0f) : 0.0f;
    SDL_GPUTextureSamplerBinding sphere_sb{opt.env_sphere ? opt.env_sphere : ctx_.white_texture(),
                                           ctx_.sampler(true, true)};
    fu.output[0] = opt.quantize_rgb5 ? 1.0f : 0.0f;
    for (float& t : fu.tint) t = 1.0f;
    for (int i = 0; i < 4; ++i) fu.view_row0[i] = view.at(0, i), fu.view_row1[i] = view.at(1, i);

    // Visibility: one entry per mesh that passes LOD selection and culling.
    struct Visible {
        const DrawItem* item;
        const GpuModel::MeshInfo* mesh;
        Mat4 world;
        float dist;
    };
    std::vector<Visible> vis;
    stats_ = {};
    for (const DrawItem& item : items) {
        const GpuModel* model = item.model;
        if (!model || !model->valid()) continue;
        int lod = item.lod == -2 ? opt.lod : item.lod;
        float max_dist = item.draw_distance >= 0 ? item.draw_distance : opt.draw_distance;
        for (const GpuModel::MeshInfo& mesh : model->meshes()) {
            if (lod >= 0 && mesh.lod != lod) continue;
            Mat4 world = item.transform * mesh.transform;
            Vec3 c = world.transform_point(mesh.center);
            float sc = 0;
            for (int col = 0; col < 3; ++col)
                sc = std::max(sc, length(Vec3{world.at(0, col), world.at(1, col), world.at(2, col)}));
            float r = mesh.radius * sc;
            float dist = length(c - eye);
            if (!item.background && !item.helper) {
                bool culled = max_dist > 0 && dist - r > max_dist;
                if (opt.frustum_cull && !culled)
                    for (const auto& p : planes)
                        if (p[0] * c.x + p[1] * c.y + p[2] * c.z + p[3] < -r) {
                            culled = true;
                            break;
                        }
                if (culled) {
                    ++stats_.meshes_culled;
                    continue;
                }
            }
            ++stats_.meshes_drawn;
            vis.push_back({&item, &mesh, world, dist});
        }
    }

    SDL_GPUGraphicsPipeline* bound = nullptr;
    auto bind = [&](const PipelineKey& key) {
        SDL_GPUGraphicsPipeline* p = ctx_.pipeline(key);
        if (p && p != bound) SDL_BindGPUGraphicsPipeline(pass, p);
        if (p) bound = p;
        return p != nullptr;
    };
    const GpuModel* bound_model = nullptr;
    const Visible* pushed = nullptr;
    bool pushed_lines = false;
    auto use_mesh = [&](const Visible& v, bool lines) {
        const GpuModel* model = v.item->model;
        if (model != bound_model) {
            SDL_GPUBufferBinding vb{model->vertex_buffer(), 0};
            SDL_BindGPUVertexBuffers(pass, 0, &vb, 1);
            SDL_GPUBufferBinding ib{model->index_buffer(), 0};
            SDL_BindGPUIndexBuffer(pass, &ib, SDL_GPU_INDEXELEMENTSIZE_32BIT);
            bound_model = model;
        }
        if (pushed == &v && pushed_lines == lines) return;
        Mat4 mvp = (lines ? line_view_proj : view_proj) * v.world;
        Mat4 nm = transpose(inverse(v.world));
        VertexUniforms vu;
        std::copy(mvp.m.begin(), mvp.m.end(), vu.mvp);
        std::copy(v.world.m.begin(), v.world.m.end(), vu.model);
        std::copy(nm.m.begin(), nm.m.end(), vu.normal_mat);
        SDL_PushGPUVertexUniformData(cmd, 0, &vu, sizeof(vu));
        pushed = &v;
        pushed_lines = lines;
    };
    // half: kAlphaFailBlend primitives are drawn in two halves: 1 = fragments passing the
    // alpha test (with depth write, in the opaque phase), 2 = the failing ones (blended,
    // no depth write, in the blended phase). 0 = whole primitive.
    // Per-vertex lighting off until a primitive needs it (slot 1 of the vertex uniforms).
    {
        VertexLighting off{};
        SDL_PushGPUVertexUniformData(cmd, 1, &off, sizeof(off));
    }
    bool vl_on = false;
    auto draw_prim = [&](const Visible& v, const scene::Primitive& p, int half = 0) {
        bool background = v.item->background;
        PipelineKey key;
        key.blend = (p.flags & scene::kMultiplyDest) ? BlendMode::MultiplyDest
                    : (p.flags & scene::kAdditive)   ? BlendMode::Additive
                    : (p.flags & scene::kAlphaBlend) ? BlendMode::Alpha
                                                     : BlendMode::Opaque;
        key.cull = opt.cull ? *opt.cull : (p.flags & scene::kDoubleSided) ? CullMode::None : CullMode::Back;
        if (opt.mirrored && key.cull != CullMode::None)
            key.cull = key.cull == CullMode::Back ? CullMode::Front : CullMode::Back;
        key.depth_write = !(p.flags & scene::kNoDepthWrite) && !background && half != 2;
        key.coverage = opt.clear[3] < 1.0f;
        bool faded = v.item->alpha < 1.0f;
        if (faded && key.blend == BlendMode::Opaque) key.blend = BlendMode::Alpha;
        if (v.item->dest_alpha_blend && key.blend != BlendMode::Opaque)
            key.blend = v.item->dest_alpha_blend == 1 ? BlendMode::DestAlphaLerp : BlendMode::DestAlphaAdd;
        if (p.flags & scene::kAlphaOnly) {
            if (!v.item->frame_alpha) return;
            key.blend = BlendMode::Opaque, key.alpha = AlphaWrite::Only;
        } else if (v.item->frame_alpha && !key.coverage) {
            bool sheen_pass = (p.flags & scene::kAdditive) && (p.flags & scene::kEnvironmentMap) && p.texture >= 0 &&
                              !(p.flags & scene::kUnlit);
            if (sheen_pass) key.blend = BlendMode::DestAlphaAdd; // Cs * Ad + Cd, alpha kept
            else if (!(p.flags & scene::kKeepAlpha)) key.alpha = AlphaWrite::Write;
            frame_alpha_written_ = frame_alpha_written_ || key.alpha != AlphaWrite::Keep;
        }
        if (!bind(key)) return;
        use_mesh(v, false);
        const GpuModel* model = v.item->model;
        SDL_GPUTextureSamplerBinding sb{ctx_.white_texture(), ctx_.sampler(false, false)};
        bool textured = false;
        if (v.item->texture) {
            sb = {v.item->texture, ctx_.sampler(true, true)};
            textured = true;
        } else if (p.texture >= 0) {
            const GpuModel::TextureInfo& t = model->textures()[size_t(p.texture)];
            if (t.texture) sb = {t.texture, ctx_.sampler(t.clamp_u, t.clamp_v)};
            textured = t.texture != nullptr;
        }
        SDL_GPUTextureSamplerBinding sbs[3] = {sb, {env_tex, ctx_.sampler(true, true)}, sphere_sb};
        SDL_BindGPUFragmentSamplers(pass, 0, sbs, 3);
        bool blended_prim = key.blend != BlendMode::Opaque;
        // Transparent pictures (composited over the 2D frame): opaque primitives cover fully,
        // whatever their texel alpha (which is reflectivity there).
        fu.grade2[3] = (opt.clear[3] < 1.0f && !blended_prim) ? 1.0f : 0.0f;
        fu.params[0] = !(p.flags & scene::kAlphaTest) ? -1.0f
                       : p.alpha_ref >= 0                ? p.alpha_ref
                       : blended_prim                    ? opt.blend_alpha_ref
                                                         : opt.alpha_ref;
        if (half == 2 && fu.params[0] >= 0) fu.params[0] += 2.0f; // keep only the failing fragments
        fu.params[1] = (p.flags & scene::kUnlit) ? 1.0f : 0.0f;
        // kAdditive | kEnvironmentMap: car reflection pass when kUnlit is set or nothing is
        // bound (the PS2 binds the env map at run time; a bound texture only supplies the
        // per-pixel reflectivity in its alpha); otherwise the textured course road sheen.
        bool env_add = (p.flags & scene::kAdditive) && (p.flags & scene::kEnvironmentMap);
        bool reflection = env_add && ((p.flags & scene::kUnlit) || !textured);
        bool sheen = env_add && !reflection;
        fu.params[2] = reflection ? opt.reflection_strength
                       : sheen    ? opt.sheen_strength
                       : (p.flags & scene::kEnvironmentMap) ? opt.env_strength
                                                            : 0.0f;
        fu.params[3] = reflection ? 3.0f : sheen ? 2.0f : 0.0f;
        // Prize scenes (.prm, DrawItem::sphere_map): a textured kEnvironmentMap primitive
        // carries its own sphere map, sampled from the view-space normal.
        if (v.item->sphere_map && (p.flags & scene::kEnvironmentMap) && textured) {
            fu.params[3] = 4.0f;
            // Additive layers (GS ALPHA 0x58: Cs * Ad + Cd) are weighted by the frame
            // buffer alpha the base pass wrote (1.0 on these models), not by the texel alpha.
            fu.params[2] = (p.flags & scene::kAdditive) ? 1.0f : 0.0f;
        }
        if (const ItemLights* il = v.item->lights) {
            float* dirs[4] = {fu.light_dir, fu.light2_dir, fu.light3_dir, fu.light4_dir};
            float* difs[4] = {fu.light_diffuse, fu.light2_diffuse, fu.light3_diffuse, fu.light4_diffuse};
            float* spes[4] = {fu.light_specular, fu.light2_specular, fu.light3_specular, fu.light4_specular};
            for (int k = 0; k < 4; ++k) {
                bool on = k < il->count;
                for (int i = 0; i < 3; ++i) {
                    dirs[k][i] = on ? il->dir[k][i] : (k == 0 ? 0.0f : 0.0f);
                    difs[k][i] = on ? il->diffuse[k][i] * v.item->light_scale[1] : 0.0f;
                    spes[k][i] = on ? il->specular[k][i] * v.item->light_scale[2] : 0.0f;
                }
                if (k == 0 && !on) dirs[0][1] = 1; // keep a valid direction
                if (k > 0) dirs[k][3] = on ? 1.0f : 0.0f;
            }
            for (int i = 0; i < 3; ++i) fu.light_ambient[i] = il->ambient[i] * v.item->light_scale[0];
        } else {
            for (int i = 0; i < 3; ++i) {
                fu.light_ambient[i] = opt.light_ambient[i] * v.item->light_scale[0];
                fu.light_diffuse[i] = opt.light_diffuse[i] * v.item->light_scale[1];
                fu.light_specular[i] = opt.light_specular[i] * v.item->light_scale[2];
            }
            Vec3 ld = normalize(opt.light_dir);
            fu.light_dir[0] = ld.x, fu.light_dir[1] = ld.y, fu.light_dir[2] = ld.z;
            float* dirs[3] = {fu.light2_dir, fu.light3_dir, fu.light4_dir};
            float* difs[3] = {fu.light2_diffuse, fu.light3_diffuse, fu.light4_diffuse};
            float* spes[3] = {fu.light2_specular, fu.light3_specular, fu.light4_specular};
            for (int k = 0; k < 3; ++k) {
                bool on = k < opt.fill_count;
                for (int i = 0; i < 3; ++i) { // GT3 road shade scales every car light
                    dirs[k][i] = opt.fill_dir[k][i];
                    difs[k][i] = opt.fill_diffuse[k][i] * v.item->light_scale[1];
                    spes[k][i] = opt.fill_specular[k][i] * v.item->light_scale[2];
                }
                dirs[k][3] = on ? 1.0f : 0.0f;
            }
        }
        if (reflection) fu.params[2] *= v.item->reflection_scale;
        fu.env_c0[3] = p.env_rows ? 1.0f : 0.0f;
        for (int i = 0; i < 3; ++i) fu.env_c0[i] = p.env_c0[i], fu.env_c1[i] = p.env_c1[i];
        const scene::Material& mt = p.material;
        fu.mat_ambient[3] = mt.valid ? 1.0f : 0.0f;
        if (mt.valid) {
            for (int i = 0; i < 3; ++i) {
                fu.mat_ambient[i] = mt.ambient[i], fu.mat_diffuse[i] = mt.diffuse[i];
                fu.mat_specular[i] = mt.specular[i], fu.mat_emission[i] = mt.emission[i] + mt.lamp_emission[i] * v.item->lamp;
            }
            fu.mat_diffuse[3] = mt.scale;
            // The GS uses the lit colour as is without a texture (255 = 1.0) and as a MODULATE
            // factor with one (0x80 = 1.0): untextured packets show it at 128/255.
            fu.mat_emission[3] = textured ? 1.0f : 128.0f / 255.0f;
            // GT3's VU kernels quantise the exponent: shininess < 6 -> 4, < 12 -> 8, else 16.
            bool spec = mt.specular[0] > 0 || mt.specular[1] > 0 || mt.specular[2] > 0;
            fu.mat_specular[3] = !spec ? 0.0f : mt.shininess < 6 ? 4.0f : mt.shininess < 12 ? 8.0f : 16.0f;
        }
        fu.fog_color[3] = opt.fog && !background && !v.item->helper ? 1.0f : 0.0f;
        fu.tint[3] = std::max(v.item->alpha, 0.0f); // > 1 (blended items): the blend clamps
        // GT3's lighting runs per vertex (the VU), with the infinite viewer: the lights and
        // material go to the vertex shader.
        const bool vertex_lit = mt.valid && fu.params[1] < 0.5f && fu.params[3] < 0.5f;
        if (vertex_lit || vl_on) {
            VertexLighting vl{};
            if (vertex_lit) {
                for (int i = 0; i < 3; ++i) vl.eye_dir[i] = view.at(2, i);
                vl.eye_dir[3] = 1;
                const float* dirs[4] = {fu.light_dir, fu.light2_dir, fu.light3_dir, fu.light4_dir};
                const float* difs[4] = {fu.light_diffuse, fu.light2_diffuse, fu.light3_diffuse, fu.light4_diffuse};
                const float* spes[4] = {fu.light_specular, fu.light2_specular, fu.light3_specular, fu.light4_specular};
                for (int k = 0; k < 4; ++k) {
                    for (int i = 0; i < 3; ++i) {
                        vl.light_dir[k][i] = dirs[k][i];
                        vl.light_diffuse[k][i] = difs[k][i];
                        vl.light_specular[k][i] = spes[k][i];
                    }
                    vl.light_dir[k][3] = k == 0 ? 1.0f : dirs[k][3]; // light 0 is always on
                }
                std::copy(std::begin(fu.light_ambient), std::end(fu.light_ambient), vl.light_ambient);
                std::copy(std::begin(fu.mat_ambient), std::end(fu.mat_ambient), vl.mat_ambient);
                std::copy(std::begin(fu.mat_diffuse), std::end(fu.mat_diffuse), vl.mat_diffuse);
                std::copy(std::begin(fu.mat_specular), std::end(fu.mat_specular), vl.mat_specular);
                std::copy(std::begin(fu.mat_emission), std::end(fu.mat_emission), vl.mat_emission);
            }
            SDL_PushGPUVertexUniformData(cmd, 1, &vl, sizeof(vl));
            vl_on = vertex_lit;
        }
        SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
        SDL_DrawGPUIndexedPrimitives(pass, p.index_count, 1, v.mesh->index_offset + p.first_index,
                                     v.mesh->vertex_offset, 0);
        ++stats_.draw_calls;
        stats_.triangles += p.index_count / 3;
    };
    auto prim_blended = [](const scene::Primitive& p) {
        return (p.flags & (scene::kAlphaBlend | scene::kAdditive | scene::kMultiplyDest | scene::kAlphaOnly)) != 0;
    };
    auto split = [](const scene::Primitive& p) {
        return (p.flags & (scene::kAlphaFailBlend | scene::kAlphaTest)) == (scene::kAlphaFailBlend | scene::kAlphaTest);
    };
    bool solid = opt.wire != WireMode::Only;

    // Background (sky, backdrop): everything in model order, no depth writes.
    for (const Visible& v : vis)
        if (v.item->background && solid)
            for (const scene::Primitive& p : v.mesh->primitives) draw_prim(v, p);
    // Opaque and alpha-tested.
    for (const Visible& v : vis)
        if (!v.item->background && (solid || v.item->helper))
            for (const scene::Primitive& p : v.mesh->primitives) {
                if (v.item->alpha < 1.0f) continue;
                if (!prim_blended(p)) draw_prim(v, p);
                else if (split(p)) draw_prim(v, p, 1);
            }
    // Blended: model order, or back to front by mesh distance.
    std::vector<const Visible*> blended;
    for (const Visible& v : vis)
        if (!v.item->background && (solid || v.item->helper))
            for (const scene::Primitive& p : v.mesh->primitives)
                if (prim_blended(p) || v.item->alpha < 1.0f) {
                    blended.push_back(&v);
                    break;
                }
    if (opt.sort_blended) // in-order items first (in model order), then the rest back to front
        std::stable_sort(blended.begin(), blended.end(), [](const Visible* a, const Visible* b) {
            if (a->item->in_order != b->item->in_order) return a->item->in_order;
            return !a->item->in_order && a->dist > b->dist;
        });
    for (const Visible* v : blended)
        for (const scene::Primitive& p : v->mesh->primitives)
            if (prim_blended(p) || v->item->alpha < 1.0f)
                draw_prim(*v, p, split(p) && v->item->alpha >= 1.0f ? 2 : 0);
    // Wireframe.
    if (opt.wire != WireMode::Off)
        for (const Visible& v : vis) {
            if (v.item->helper || v.item->background || !v.mesh->line_count) continue;
            if (!bind({BlendMode::Opaque, CullMode::None, true, true})) continue;
            use_mesh(v, true);
            SDL_GPUTextureSamplerBinding sbs[3] = {{ctx_.white_texture(), ctx_.sampler(false, false)},
                                                   {env_tex, ctx_.sampler(true, true)}, sphere_sb};
            SDL_BindGPUFragmentSamplers(pass, 0, sbs, 3);
            fu.params[0] = -1, fu.params[1] = 1, fu.params[2] = 0, fu.params[3] = 1;
            fu.fog_color[3] = 0;
            SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_DrawGPUIndexedPrimitives(pass, v.mesh->line_count, 1, v.mesh->index_offset + v.mesh->line_first,
                                         v.mesh->vertex_offset, 0);
            ++stats_.draw_calls;
        }
}

void View3D::capture_from_env() {
    const char* e = SDL_getenv("GT3R_CAPTURE3D");
    if (!e || capture_.count > 0) return;
    std::string s = e;
    size_t a = s.find(','), b = a == std::string::npos ? a : s.find(',', a + 1);
    if (b == std::string::npos) return;
    capture_.first = std::atoi(s.substr(0, a).c_str());
    capture_.count = std::max(0, std::atoi(s.substr(a + 1, b - a - 1).c_str()));
    capture_.path = s.substr(b + 1);
}

void View3D::capture_frame(SDL_GPUCommandBuffer* cmd) {
    int k = capture_.frame++ - capture_.first;
    if (k < 0 || k >= capture_.count) return;
    if (k == 0) capture_.w = width_, capture_.h = height_;
    if (width_ != capture_.w || height_ != capture_.h) return;
    SDL_GPUTextureCreateInfo ti{};
    ti.type = SDL_GPU_TEXTURETYPE_2D;
    ti.format = GpuContext::kColorFormat;
    ti.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;
    ti.width = u32(width_);
    ti.height = u32(height_);
    ti.layer_count_or_depth = 1;
    ti.num_levels = 1;
    SDL_GPUTexture* t = SDL_CreateGPUTexture(ctx_.device(), &ti);
    capture_.frames.push_back(t);
    if (!t) return;
    SDL_GPUCopyPass* cp = SDL_BeginGPUCopyPass(cmd);
    SDL_GPUTextureLocation src{}, dst{};
    src.texture = color_;
    dst.texture = t;
    SDL_CopyGPUTextureToTexture(cp, &src, &dst, u32(width_), u32(height_), 1, false);
    SDL_EndGPUCopyPass(cp);
}

void View3D::write_capture() {
    SDL_GPUDevice* dev = ctx_.device();
    SDL_WaitForGPUIdle(dev);
    u32 w = u32(capture_.w), h = u32(capture_.h);
    for (size_t k = 0; k < capture_.frames.size(); ++k) {
        SDL_GPUTexture* t = capture_.frames[k];
        if (!t) continue;
        SDL_GPUTransferBufferCreateInfo bi{};
        bi.usage = SDL_GPU_TRANSFERBUFFERUSAGE_DOWNLOAD;
        bi.size = w * h * 4;
        SDL_GPUTransferBuffer* tb = SDL_CreateGPUTransferBuffer(dev, &bi);
        SDL_GPUCommandBuffer* cmd = tb ? SDL_AcquireGPUCommandBuffer(dev) : nullptr;
        if (cmd) {
            SDL_GPUCopyPass* cp = SDL_BeginGPUCopyPass(cmd);
            SDL_GPUTextureRegion src{};
            src.texture = t;
            src.w = w, src.h = h, src.d = 1;
            SDL_GPUTextureTransferInfo dst{};
            dst.transfer_buffer = tb;
            SDL_DownloadFromGPUTexture(cp, &src, &dst);
            SDL_EndGPUCopyPass(cp);
            SDL_GPUFence* f = SDL_SubmitGPUCommandBufferAndAcquireFence(cmd);
            if (f) {
                SDL_WaitForGPUFences(dev, true, &f, 1);
                SDL_ReleaseGPUFence(dev, f);
            }
            std::vector<u32> px(size_t(w) * h);
            if (void* p = SDL_MapGPUTransferBuffer(dev, tb, false)) {
                std::memcpy(px.data(), p, px.size() * 4);
                SDL_UnmapGPUTransferBuffer(dev, tb);
            }
            for (u32& c : px) c |= 0xFF000000u; // texel alpha is reflectivity
            std::string path = capture_.path;
            size_t dot = path.rfind('.');
            std::string suffix = "-" + std::to_string(k);
            path = dot == std::string::npos ? path + suffix : path.substr(0, dot) + suffix + path.substr(dot);
            if (write_host_file(path, encode_png(w, h, px))) LOG_INFO("3D capture saved: %s", path.c_str());
        }
        if (tb) SDL_ReleaseGPUTransferBuffer(dev, tb);
        SDL_ReleaseGPUTexture(dev, t);
    }
    capture_ = {};
}

SDL_GPUTexture* View3D::placeholder_cube() {
    if (placeholder_cube_) return placeholder_cube_;
    SDL_GPUDevice* dev = ctx_.device();
    SDL_GPUTextureCreateInfo ti{};
    ti.type = SDL_GPU_TEXTURETYPE_CUBE;
    ti.format = GpuContext::kColorFormat;
    ti.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    ti.width = ti.height = 1;
    ti.layer_count_or_depth = 6;
    ti.num_levels = 1;
    ti.sample_count = SDL_GPU_SAMPLECOUNT_1;
    placeholder_cube_ = SDL_CreateGPUTexture(dev, &ti);
    if (!placeholder_cube_) return nullptr;
    if (SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(dev)) {
        for (u32 f = 0; f < 6; ++f) {
            SDL_GPUColorTargetInfo ct{};
            ct.texture = placeholder_cube_;
            ct.layer_or_depth_plane = f;
            ct.clear_color = {0.5f, 0.5f, 0.5f, 1.0f};
            ct.load_op = SDL_GPU_LOADOP_CLEAR;
            ct.store_op = SDL_GPU_STOREOP_STORE;
            SDL_EndGPURenderPass(SDL_BeginGPURenderPass(cmd, &ct, 1, nullptr));
        }
        SDL_SubmitGPUCommandBuffer(cmd);
    }
    return placeholder_cube_;
}

void View3D::release_offscreen() {
    for (Offscreen& o : offscreen_) {
        for (SDL_GPUTexture** t : {&o.color, &o.msaa, &o.depth}) {
            if (*t) SDL_ReleaseGPUTexture(ctx_.device(), *t);
            *t = nullptr;
        }
        o.w = o.h = o.samples = 0;
    }
}

SDL_GPUTexture* View3D::render_offscreen(int slot, int w, int h, const Mat4& view, const Mat4& proj, const Vec3& eye,
                                         const std::vector<DrawItem>& items, const DrawOptions& opt) {
    if (slot < 0 || slot >= 4 || w <= 0 || h <= 0) return nullptr;
    Offscreen& o = offscreen_[slot];
    SDL_GPUDevice* dev = ctx_.device();
    if (!o.color || o.w != w || o.h != h || o.samples != ctx_.msaa()) {
        for (SDL_GPUTexture** t : {&o.color, &o.msaa, &o.depth}) {
            if (*t) SDL_ReleaseGPUTexture(dev, *t);
            *t = nullptr;
        }
        SDL_GPUTextureCreateInfo ti{};
        ti.type = SDL_GPU_TEXTURETYPE_2D;
        ti.format = GpuContext::kColorFormat;
        ti.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
        ti.width = u32(w), ti.height = u32(h);
        ti.layer_count_or_depth = 1;
        ti.num_levels = 1;
        ti.sample_count = SDL_GPU_SAMPLECOUNT_1;
        o.color = SDL_CreateGPUTexture(dev, &ti);
        // The scene pipelines are built for the context's sample count.
        if (ctx_.sample_count() != SDL_GPU_SAMPLECOUNT_1) {
            ti.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
            ti.sample_count = ctx_.sample_count();
            o.msaa = SDL_CreateGPUTexture(dev, &ti);
        }
        ti.format = ctx_.depth_format();
        ti.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
        ti.sample_count = ctx_.sample_count();
        o.depth = SDL_CreateGPUTexture(dev, &ti);
        o.w = w, o.h = h, o.samples = ctx_.msaa();
        if (!o.color || !o.depth || (ctx_.sample_count() != SDL_GPU_SAMPLECOUNT_1 && !o.msaa)) {
            LOG_WARN("Offscreen buffer %dx%d: %s", w, h, SDL_GetError());
            return nullptr;
        }
    }
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(dev);
    if (!cmd) return nullptr;
    SDL_GPUColorTargetInfo ct{};
    ct.clear_color = {opt.clear[0], opt.clear[1], opt.clear[2], opt.clear[3]};
    ct.load_op = SDL_GPU_LOADOP_CLEAR;
    // The previous frame's commands may still sample the buffer: cycle it.
    if (o.msaa) {
        ct.texture = o.msaa;
        ct.store_op = SDL_GPU_STOREOP_RESOLVE;
        ct.resolve_texture = o.color;
        ct.cycle = true;
        ct.cycle_resolve_texture = true;
    } else {
        ct.texture = o.color;
        ct.store_op = SDL_GPU_STOREOP_STORE;
        ct.cycle = true;
    }
    SDL_GPUDepthStencilTargetInfo dt{};
    dt.texture = o.depth;
    dt.clear_depth = 1.0f;
    dt.load_op = SDL_GPU_LOADOP_CLEAR;
    dt.store_op = SDL_GPU_STOREOP_DONT_CARE;
    dt.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    dt.cycle = true;
    DrawStats keep = stats_;
    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &ct, 1, &dt);
    encode(cmd, pass, view, proj, eye, items, opt);
    SDL_EndGPURenderPass(pass);
    SDL_SubmitGPUCommandBuffer(cmd);
    stats_ = keep;
    return o.color;
}

void View3D::release_cube() {
    SDL_GPUDevice* dev = ctx_.device();
    for (SDL_GPUTexture** t : {&cube_, &cube_depth_, &cube_msaa_[0], &cube_msaa_[1], &cube_msaa_[2], &cube_msaa_[3],
                               &cube_msaa_[4], &cube_msaa_[5]}) {
        if (*t) SDL_ReleaseGPUTexture(dev, *t);
        *t = nullptr;
    }
}

bool View3D::ensure_cube(int size) {
    if (cube_ && size == cube_size_ && cube_samples_ == ctx_.msaa()) return true;
    SDL_GPUDevice* dev = ctx_.device();
    release_cube();
    SDL_GPUTextureCreateInfo ti{};
    ti.type = SDL_GPU_TEXTURETYPE_CUBE;
    ti.format = GpuContext::kColorFormat;
    ti.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    ti.width = ti.height = u32(size);
    ti.layer_count_or_depth = 6;
    ti.num_levels = 1;
    ti.sample_count = SDL_GPU_SAMPLECOUNT_1;
    cube_ = SDL_CreateGPUTexture(dev, &ti);
    // The scene pipelines are built for the context's sample count: render each face at it.
    ti.type = SDL_GPU_TEXTURETYPE_2D;
    ti.layer_count_or_depth = 1;
    bool msaa_ok = true;
    if (ctx_.sample_count() != SDL_GPU_SAMPLECOUNT_1) {
        ti.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
        ti.sample_count = ctx_.sample_count();
        for (SDL_GPUTexture*& t : cube_msaa_) msaa_ok = (t = SDL_CreateGPUTexture(dev, &ti)) && msaa_ok;
    }
    ti.format = ctx_.depth_format();
    ti.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
    ti.sample_count = ctx_.sample_count();
    cube_depth_ = SDL_CreateGPUTexture(dev, &ti);
    if (!cube_ || !cube_depth_ || !msaa_ok) {
        LOG_WARN("Environment cube %d creation failed: %s", size, SDL_GetError());
        release_cube();
        return false;
    }
    cube_size_ = size;
    cube_samples_ = ctx_.msaa();
    return true;
}

SDL_GPUTexture* View3D::render_env_cube(const Vec3& pos, const std::vector<DrawItem>& items, const DrawOptions& opt,
                                        int size, unsigned faces) {
    if (!ensure_cube(size)) return nullptr;
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(ctx_.device());
    if (!cmd) return nullptr;
    // Standard cube-map face frames (direction, up), as the sampler addresses them.
    static const Vec3 kDir[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    static const Vec3 kUp[6] = {{0, -1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {0, -1, 0}, {0, -1, 0}};
    // Cube faces are seen from inside, left-handed: mirror x in clip space.
    Mat4 proj = perspective(1.5707963f, 1.0f, 0.1f, std::max(opt.draw_distance, 10.0f) * 1.5f);
    for (int c = 0; c < 4; ++c) proj.at(0, c) = -proj.at(0, c);
    DrawStats keep = stats_;
    DrawOptions o = opt;
    o.env_cube = nullptr; // no reflections of reflections
    o.mirrored = !opt.mirrored;
    // The previous frame's command buffer may still be sampling the cube: a full refresh cycles
    // it (SDL_GPU: overwriting a texture a pending command references is undefined), at its first
    // face only - cycling undefines the whole cube. A partial refresh keeps the other faces.
    bool cycle = (faces & 0x3Fu) == 0x3Fu;
    for (int f = 0; f < 6; ++f) {
        if (!(faces & (1u << f))) continue;
        SDL_GPUColorTargetInfo ct{};
        ct.clear_color = {opt.clear[0], opt.clear[1], opt.clear[2], 1.0f};
        ct.load_op = SDL_GPU_LOADOP_CLEAR;
        if (cube_msaa_[f]) {
            ct.texture = cube_msaa_[f];
            ct.store_op = SDL_GPU_STOREOP_RESOLVE;
            ct.resolve_texture = cube_;
            ct.resolve_layer = u32(f);
            ct.cycle = true;
            ct.cycle_resolve_texture = cycle;
        } else {
            ct.texture = cube_;
            ct.layer_or_depth_plane = u32(f);
            ct.store_op = SDL_GPU_STOREOP_STORE;
            ct.cycle = cycle;
        }
        cycle = false;
        SDL_GPUDepthStencilTargetInfo dt{};
        dt.texture = cube_depth_;
        dt.clear_depth = 1.0f;
        dt.load_op = SDL_GPU_LOADOP_CLEAR;
        dt.store_op = SDL_GPU_STOREOP_DONT_CARE;
        dt.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
        dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
        dt.cycle = true;
        SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &ct, 1, &dt);
        encode(cmd, pass, look_at(pos, pos + kDir[f], kUp[f]), proj, pos, items, o);
        SDL_EndGPURenderPass(pass);
    }
    SDL_SubmitGPUCommandBuffer(cmd);
    stats_ = keep;
    return cube_;
}

} // namespace gt3r::render
