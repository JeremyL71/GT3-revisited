#include "render/gpu_context.h"

#include "base/log.h"
#include "render/shaders/generated/model_frag.h"
#include "render/shaders/generated/model_vert.h"
#include "scene/model.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#define GT3R_HAVE_GPU_RENDERER SDL_VERSION_ATLEAST(3, 4, 0)

namespace gt3r::render {

bool gpu_renderer_supported() { return GT3R_HAVE_GPU_RENDERER; }

std::unique_ptr<GpuContext> GpuContext::create(SDL_Window* window, const GpuOptions& options) {
#if GT3R_HAVE_GPU_RENDERER
    std::unique_ptr<GpuContext> ctx(new GpuContext());
    if (!ctx->init(window, options)) return nullptr;
    return ctx;
#else
    (void)window;
    (void)options;
    LOG_WARN("3D renderer disabled: built against SDL %d.%d.%d, needs 3.4 or later", SDL_MAJOR_VERSION,
             SDL_MINOR_VERSION, SDL_MICRO_VERSION);
    return nullptr;
#endif
}

bool GpuContext::init(SDL_Window* window, const GpuOptions& options) {
#if GT3R_HAVE_GPU_RENDERER
    device_ = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_MSL, options.debug, nullptr);
    if (!device_) {
        LOG_WARN("No SDL_GPU device: %s", SDL_GetError());
        return false;
    }
    // The 2D renderer claims the window and owns the swapchain.
    renderer_ = SDL_CreateGPURenderer(device_, window);
    if (!renderer_) {
        LOG_WARN("SDL_CreateGPURenderer failed: %s", SDL_GetError());
        return false;
    }
    SDL_SetRenderVSync(renderer_, options.vsync ? 1 : 0);

    vs_ = make_shader(true);
    fs_ = make_shader(false);
    if (!vs_ || !fs_) return false;

    for (SDL_GPUTextureFormat f :
         {SDL_GPU_TEXTUREFORMAT_D32_FLOAT, SDL_GPU_TEXTUREFORMAT_D24_UNORM, SDL_GPU_TEXTUREFORMAT_D16_UNORM})
        if (SDL_GPUTextureSupportsFormat(device_, f, SDL_GPU_TEXTURETYPE_2D, SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET)) {
            depth_format_ = f;
            break;
        }
    set_msaa(options.msaa);
    set_render_scale(options.render_scale);

    for (int i = 0; i < 4; ++i) {
        SDL_GPUSamplerCreateInfo si{};
        si.min_filter = SDL_GPU_FILTER_LINEAR;
        si.mag_filter = SDL_GPU_FILTER_LINEAR;
        si.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
        si.address_mode_u = (i & 1) ? SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE : SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
        si.address_mode_v = (i & 2) ? SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE : SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
        si.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
        si.max_lod = 1000.0f;
        si.enable_anisotropy = true;
        si.max_anisotropy = 4;
        samplers_[i] = SDL_CreateGPUSampler(device_, &si);
        if (!samplers_[i]) {
            LOG_WARN("Sampler creation failed: %s", SDL_GetError());
            return false;
        }
    }

    Uploader up(device_);
    const u32 white = 0xFFFFFFFFu;
    white_ = up.texture_rgba(1, 1, &white, false);
    if (!white_ || !up.submit()) return false;

    LOG_INFO("3D renderer: SDL_GPU %s, MSAA %dx, depth format %d, render scale %.2f", driver_name(), msaa(),
             int(depth_format_), double(render_scale_));
    return true;
#else
    (void)window;
    (void)options;
    return false;
#endif
}

GpuContext::~GpuContext() {
    if (device_) {
        SDL_WaitForGPUIdle(device_);
        release_pipelines();
        for (SDL_GPUSampler* s : samplers_)
            if (s) SDL_ReleaseGPUSampler(device_, s);
        if (white_) SDL_ReleaseGPUTexture(device_, white_);
        if (vs_) SDL_ReleaseGPUShader(device_, vs_);
        if (fs_) SDL_ReleaseGPUShader(device_, fs_);
    }
    if (renderer_) SDL_DestroyRenderer(renderer_); // releases the window claim
    if (device_) SDL_DestroyGPUDevice(device_);
}

SDL_GPUShader* GpuContext::make_shader(bool vertex) {
    SDL_GPUShaderCreateInfo info{};
    SDL_GPUShaderFormat formats = SDL_GetGPUShaderFormats(device_);
    if (formats & SDL_GPU_SHADERFORMAT_SPIRV) {
        info.format = SDL_GPU_SHADERFORMAT_SPIRV;
        info.code = vertex ? shaders::model_vert_spv : shaders::model_frag_spv;
        info.code_size = vertex ? sizeof(shaders::model_vert_spv) : sizeof(shaders::model_frag_spv);
        info.entrypoint = "main";
    } else if (formats & SDL_GPU_SHADERFORMAT_MSL) {
        info.format = SDL_GPU_SHADERFORMAT_MSL;
        const char* src = vertex ? shaders::model_vert_msl : shaders::model_frag_msl;
        info.code = reinterpret_cast<const Uint8*>(src);
        info.code_size = std::strlen(src);
        info.entrypoint = "main0";
    } else {
        LOG_WARN("GPU driver %s accepts none of our shader formats", driver_name());
        return nullptr;
    }
    info.stage = vertex ? SDL_GPU_SHADERSTAGE_VERTEX : SDL_GPU_SHADERSTAGE_FRAGMENT;
    info.num_samplers = vertex ? 0 : 3; // fragment: the texture, the environment cube, the sphere map
    info.num_uniform_buffers = vertex ? 2 : 1; // vertex: transforms, GT3 per-vertex lighting
    SDL_GPUShader* s = SDL_CreateGPUShader(device_, &info);
    if (!s) LOG_WARN("Shader creation failed: %s", SDL_GetError());
    return s;
}

void GpuContext::set_render_scale(float s) {
    render_scale_ = std::isfinite(s) ? std::clamp(s, 0.5f, 1.0f) : 1.0f;
}

void GpuContext::set_msaa(int samples) {
    SDL_GPUSampleCount want = samples >= 8   ? SDL_GPU_SAMPLECOUNT_8
                              : samples >= 4 ? SDL_GPU_SAMPLECOUNT_4
                              : samples >= 2 ? SDL_GPU_SAMPLECOUNT_2
                                             : SDL_GPU_SAMPLECOUNT_1;
    while (want != SDL_GPU_SAMPLECOUNT_1 && !(SDL_GPUTextureSupportsSampleCount(device_, kColorFormat, want) &&
                                              SDL_GPUTextureSupportsSampleCount(device_, depth_format_, want)))
        want = SDL_GPUSampleCount(int(want) - 1);
    if (want != samples_) {
        SDL_WaitForGPUIdle(device_);
        release_pipelines();
        samples_ = want;
    }
}

void GpuContext::release_pipelines() {
    for (auto& [key, p] : pipelines_) SDL_ReleaseGPUGraphicsPipeline(device_, p);
    pipelines_.clear();
}

SDL_GPUGraphicsPipeline* GpuContext::pipeline(const PipelineKey& key) {
    auto it = pipelines_.find(key);
    if (it != pipelines_.end()) return it->second;

    SDL_GPUVertexBufferDescription vb{};
    vb.slot = 0;
    vb.pitch = sizeof(scene::Vertex);
    vb.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    SDL_GPUVertexAttribute attrs[4] = {
        {0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, u32(offsetof(scene::Vertex, pos))},
        {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, u32(offsetof(scene::Vertex, normal))},
        {2, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, u32(offsetof(scene::Vertex, uv))},
        {3, 0, SDL_GPU_VERTEXELEMENTFORMAT_UBYTE4_NORM, u32(offsetof(scene::Vertex, color))},
    };

    SDL_GPUColorTargetDescription color{};
    color.format = kColorFormat;
    SDL_GPUColorTargetBlendState& b = color.blend_state;
    // Never write alpha: the target is cleared to opaque and must stay opaque, otherwise
    // translucent texels (e.g. car paint layers, alpha-tested foliage) leak into the
    // composited view and into screenshots.
    b.enable_color_write_mask = true;
    b.color_write_mask = SDL_GPU_COLORCOMPONENT_R | SDL_GPU_COLORCOMPONENT_G | SDL_GPU_COLORCOMPONENT_B;
    if (key.coverage) b.color_write_mask |= SDL_GPU_COLORCOMPONENT_A;
    switch (key.blend) {
    case BlendMode::Opaque:
        break;
    case BlendMode::Alpha:
        b.enable_blend = true;
        b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
        b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        b.color_blend_op = SDL_GPU_BLENDOP_ADD;
        b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        b.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        break;
    case BlendMode::Additive:
        b.enable_blend = true;
        b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
        b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        b.color_blend_op = SDL_GPU_BLENDOP_ADD;
        b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
        b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        b.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        break;
    case BlendMode::MultiplyDest:
        b.enable_blend = true;
        b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
        b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
        b.color_blend_op = SDL_GPU_BLENDOP_ADD;
        b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
        b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        b.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        break;
    case BlendMode::DestAlphaAdd:
        b.enable_blend = true;
        b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_DST_ALPHA;
        b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        b.color_blend_op = SDL_GPU_BLENDOP_ADD;
        b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
        b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        b.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        break;
    case BlendMode::DestAlphaLerp:
        b.enable_blend = true;
        b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_DST_ALPHA;
        b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_DST_ALPHA;
        b.color_blend_op = SDL_GPU_BLENDOP_ADD;
        b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
        b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        b.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        break;
    }
    // GS-style frame alpha: the source alpha replaces the destination's (the GS never blends
    // the alpha channel), with the colour channels written or masked off.
    if (key.alpha != AlphaWrite::Keep) {
        if (!b.enable_blend) { // opaque: the colour as is
            b.enable_blend = true;
            b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
            b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
            b.color_blend_op = SDL_GPU_BLENDOP_ADD;
        }
        b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
        b.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        b.color_write_mask = key.alpha == AlphaWrite::Only
                                 ? SDL_GPU_COLORCOMPONENT_A
                                 : SDL_GPU_COLORCOMPONENT_R | SDL_GPU_COLORCOMPONENT_G | SDL_GPU_COLORCOMPONENT_B |
                                       SDL_GPU_COLORCOMPONENT_A;
    }

    SDL_GPUGraphicsPipelineCreateInfo info{};
    info.vertex_shader = vs_;
    info.fragment_shader = fs_;
    info.vertex_input_state.vertex_buffer_descriptions = &vb;
    info.vertex_input_state.num_vertex_buffers = 1;
    info.vertex_input_state.vertex_attributes = attrs;
    info.vertex_input_state.num_vertex_attributes = 4;
    info.primitive_type = key.lines ? SDL_GPU_PRIMITIVETYPE_LINELIST : SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    info.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    info.rasterizer_state.cull_mode = key.lines || key.cull == CullMode::None ? SDL_GPU_CULLMODE_NONE
                                      : key.cull == CullMode::Back          ? SDL_GPU_CULLMODE_BACK
                                                                            : SDL_GPU_CULLMODE_FRONT;
    info.rasterizer_state.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    info.rasterizer_state.enable_depth_clip = true;
    info.multisample_state.sample_count = samples_;
    info.depth_stencil_state.enable_depth_test = true;
    info.depth_stencil_state.enable_depth_write = key.depth_write;
    // Passes without depth writes (second passes over the same vertices, decals, sky) must
    // pass on equal depth too.
    info.depth_stencil_state.compare_op =
        key.lines || !key.depth_write ? SDL_GPU_COMPAREOP_LESS_OR_EQUAL : SDL_GPU_COMPAREOP_LESS;
    info.target_info.color_target_descriptions = &color;
    info.target_info.num_color_targets = 1;
    info.target_info.depth_stencil_format = depth_format_;
    info.target_info.has_depth_stencil_target = true;

    SDL_GPUGraphicsPipeline* p = SDL_CreateGPUGraphicsPipeline(device_, &info);
    if (!p) LOG_WARN("Pipeline creation failed: %s", SDL_GetError());
    pipelines_[key] = p; // cache failures too, so we don't retry every frame
    return p;
}

// ---------------------------------------------------------------------------

Uploader::Uploader(SDL_GPUDevice* device) : device_(device) {}

Uploader::~Uploader() {
    for (Pending& p : pending_) SDL_ReleaseGPUTransferBuffer(device_, p.transfer);
}

static SDL_GPUTransferBuffer* make_transfer(SDL_GPUDevice* device, const void* data, u32 size) {
    SDL_GPUTransferBufferCreateInfo ti{};
    ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    ti.size = size;
    SDL_GPUTransferBuffer* t = SDL_CreateGPUTransferBuffer(device, &ti);
    if (!t) return nullptr;
    void* dst = SDL_MapGPUTransferBuffer(device, t, false);
    if (!dst) {
        SDL_ReleaseGPUTransferBuffer(device, t);
        return nullptr;
    }
    std::memcpy(dst, data, size);
    SDL_UnmapGPUTransferBuffer(device, t);
    return t;
}

SDL_GPUBuffer* Uploader::buffer(SDL_GPUBufferUsageFlags usage, const void* data, u32 size) {
    if (size == 0) return nullptr;
    SDL_GPUBufferCreateInfo bi{};
    bi.usage = usage;
    bi.size = size;
    SDL_GPUBuffer* b = SDL_CreateGPUBuffer(device_, &bi);
    if (!b) return nullptr;
    SDL_GPUTransferBuffer* t = make_transfer(device_, data, size);
    if (!t) {
        SDL_ReleaseGPUBuffer(device_, b);
        return nullptr;
    }
    pending_.push_back({t, b, nullptr, size, 0, 0, false});
    return b;
}

SDL_GPUTexture* Uploader::texture_rgba(u32 w, u32 h, const u32* rgba, bool mipmaps) {
    if (w == 0 || h == 0) return nullptr;
    u32 levels = 1;
    if (mipmaps)
        while ((std::max(w, h) >> levels) > 0) ++levels;
    SDL_GPUTextureCreateInfo ti{};
    ti.type = SDL_GPU_TEXTURETYPE_2D;
    ti.format = GpuContext::kColorFormat;
    ti.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER | (levels > 1 ? SDL_GPU_TEXTUREUSAGE_COLOR_TARGET : 0);
    ti.width = w;
    ti.height = h;
    ti.layer_count_or_depth = 1;
    ti.num_levels = levels;
    SDL_GPUTexture* tex = SDL_CreateGPUTexture(device_, &ti);
    if (!tex) return nullptr;
    SDL_GPUTransferBuffer* t = make_transfer(device_, rgba, w * h * 4);
    if (!t) {
        SDL_ReleaseGPUTexture(device_, tex);
        return nullptr;
    }
    pending_.push_back({t, nullptr, tex, w * h * 4, w, h, levels > 1});
    return tex;
}

bool Uploader::submit() {
    if (pending_.empty()) return true;
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(device_);
    if (!cmd) return false;
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
    for (const Pending& p : pending_) {
        if (p.buffer) {
            SDL_GPUTransferBufferLocation src{p.transfer, 0};
            SDL_GPUBufferRegion dst{p.buffer, 0, p.size};
            SDL_UploadToGPUBuffer(copy, &src, &dst, false);
        } else {
            SDL_GPUTextureTransferInfo src{};
            src.transfer_buffer = p.transfer;
            src.pixels_per_row = p.width;
            src.rows_per_layer = p.height;
            SDL_GPUTextureRegion dst{};
            dst.texture = p.texture;
            dst.w = p.width;
            dst.h = p.height;
            dst.d = 1;
            SDL_UploadToGPUTexture(copy, &src, &dst, false);
        }
    }
    SDL_EndGPUCopyPass(copy);
    for (const Pending& p : pending_)
        if (p.texture && p.mipmaps) SDL_GenerateMipmapsForGPUTexture(cmd, p.texture);
    bool ok = SDL_SubmitGPUCommandBuffer(cmd);
    for (Pending& p : pending_) SDL_ReleaseGPUTransferBuffer(device_, p.transfer); // deferred until the GPU is done
    pending_.clear();
    return ok;
}

} // namespace gt3r::render
