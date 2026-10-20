#pragma once

// SDL_GPU device shared by the 3D renderer and the 2D UI renderer.
//
// Frame structure: the window's swapchain belongs to the 2D SDL_Renderer
// (SDL's "gpu" render driver, created on our device with
// SDL_CreateGPURenderer). 3D views render into offscreen GPU textures in
// their own command buffers, which are submitted before the 2D renderer's;
// the 2D renderer then composites each view as an ordinary SDL_Texture that
// wraps the GPU texture, and draws the UI on top. Screenshots via
// SDL_RenderReadPixels therefore include the 3D layer.
//
// Requires SDL >= 3.4 (SDL_CreateGPURenderer, GPU texture wrapping). With
// older SDL, or when no backend supports our shaders, create() returns
// nullptr and the app runs 2D-only.

#include "base/bytes.h"
#include "xr/xr_session.h" // VR builds (GT3R_XR)

#include <SDL3/SDL.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gt3r::render {

enum class BlendMode : u8 {
    Opaque,
    Alpha,
    Additive,
    MultiplyDest, // dst * src alpha
    DestAlphaAdd, // GS ALPHA 0x58: Cs * Ad + Cd (frame-buffer alpha as the weight)
    DestAlphaLerp, // GS ALPHA 0x54: (Cs - Cd) * Ad + Cd
};
// What a draw does to the frame-buffer alpha (GS semantics: a draw writes its source alpha,
// unblended, unless FBMSK keeps it).
enum class AlphaWrite : u8 {
    Keep,  // never written (the default: the target stays opaque)
    Write, // the fragment's alpha replaces it, colour as the blend mode says
    Only,  // only the alpha is written (GS FBMSK 0x00FFFFFF)
};
enum class CullMode : u8 { None, Back, Front };

struct PipelineKey {
    BlendMode blend = BlendMode::Opaque;
    CullMode cull = CullMode::Back;
    bool depth_write = true;
    bool lines = false; // line list (wireframe overlay), depth test LESS_OR_EQUAL
    // Write the alpha channel as coverage (pictures composited over the 2D frame with a
    // transparent clear); otherwise alpha is never written.
    bool coverage = false;
    AlphaWrite alpha = AlphaWrite::Keep;
    auto operator<=>(const PipelineKey&) const = default;
};

struct GpuOptions {
    int msaa = 4;       // 1, 2, 4 or 8; clamped to what the device supports
    bool vsync = true;
    bool debug = false; // backend validation layers
    float render_scale = 1.0f; // 3D resolution relative to the display (0.5..1); the UI stays native
    // VR builds (GT3R_XR): create the device through the headset's OpenXR runtime when there is
    // one; the 2D renderer then draws offscreen (canvases of xr::Session) instead of the window.
    bool xr = false;
};

class GpuContext {
public:
    // Creates the device, claims the window and creates the 2D renderer on it.
    static std::unique_ptr<GpuContext> create(SDL_Window* window, const GpuOptions& options);
    ~GpuContext();
    GpuContext(const GpuContext&) = delete;
    GpuContext& operator=(const GpuContext&) = delete;

    SDL_GPUDevice* device() const { return device_; }
    SDL_Renderer* renderer2d() const { return renderer_; }
#if GT3R_XR
    // Set when the device was created for a headset (see GpuOptions::xr).
    bool xr() const { return xr_; }
    const xr::Session::DeviceRequest& xr_request() const { return xr_request_; }
#endif
    const char* driver_name() const { return SDL_GetGPUDeviceDriver(device_); }

    // Offscreen 3D formats.
    static constexpr SDL_GPUTextureFormat kColorFormat = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
    SDL_GPUTextureFormat depth_format() const { return depth_format_; }
    SDL_GPUSampleCount sample_count() const { return samples_; }
    int msaa() const { return 1 << int(samples_); }
    // Changes the MSAA level (rounded down to a supported count); drops cached pipelines.
    void set_msaa(int samples);
    // 3D views render at this fraction of their on-screen pixel size and are
    // upscaled when composited (cheaper on high-resolution phones/tablets).
    float render_scale() const { return render_scale_; }
    void set_render_scale(float s);

    SDL_GPUGraphicsPipeline* pipeline(const PipelineKey& key);
    SDL_GPUSampler* sampler(bool clamp_u, bool clamp_v) const { return samplers_[(clamp_u ? 1 : 0) | (clamp_v ? 2 : 0)]; }
    SDL_GPUTexture* white_texture() const { return white_; }

private:
    GpuContext() = default;
    bool init(SDL_Window* window, const GpuOptions& options);
    SDL_GPUShader* make_shader(bool vertex);
    void release_pipelines();

    SDL_GPUDevice* device_ = nullptr;
    bool anisotropy_ = true; // the device was created with anisotropic filtering
#if GT3R_XR
    bool xr_ = false;
    xr::Session::DeviceRequest xr_request_;
#endif
    float render_scale_ = 1.0f;
    SDL_Renderer* renderer_ = nullptr;
    SDL_GPUShader* vs_ = nullptr;
    SDL_GPUShader* fs_ = nullptr;
    SDL_GPUSampler* samplers_[4] = {};
    SDL_GPUTexture* white_ = nullptr;
    SDL_GPUTextureFormat depth_format_ = SDL_GPU_TEXTUREFORMAT_D16_UNORM;
    SDL_GPUSampleCount samples_ = SDL_GPU_SAMPLECOUNT_1;
    std::map<PipelineKey, SDL_GPUGraphicsPipeline*> pipelines_;
};

// Batches buffer/texture uploads into one command buffer:
//   Uploader up(device); auto* vb = up.buffer(VERTEX, data, size); ...; up.submit();
// Created resources are owned by the caller. Textures with mipmaps get
// their chain generated on the GPU at submit().
class Uploader {
public:
    explicit Uploader(SDL_GPUDevice* device);
    ~Uploader();
    SDL_GPUBuffer* buffer(SDL_GPUBufferUsageFlags usage, const void* data, u32 size);
    SDL_GPUTexture* texture_rgba(u32 width, u32 height, const u32* rgba, bool mipmaps);
    bool submit();

private:
    struct Pending {
        SDL_GPUTransferBuffer* transfer;
        SDL_GPUBuffer* buffer;
        SDL_GPUTexture* texture;
        u32 size, width, height;
        bool mipmaps;
    };
    SDL_GPUDevice* device_;
    std::vector<Pending> pending_;
};

// True when this build's SDL supports the shared GPU device path.
bool gpu_renderer_supported();

} // namespace gt3r::render
