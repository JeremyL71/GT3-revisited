#pragma once

// A scene::Model uploaded to the GPU: one vertex and one index buffer for
// all meshes, one texture per scene::Texture (with mipmaps), and a line
// index list per mesh for wireframe display.

#include "render/gpu_context.h"
#include "render/math3d.h"
#include "scene/model.h"

#include <vector>

namespace gt3r::render {

struct ModelStats {
    size_t meshes = 0, primitives = 0, vertices = 0, triangles = 0, textures = 0;
    int max_lod = 0;
};
// CPU-side statistics; `lod` < 0 counts every mesh, otherwise only that LOD.
ModelStats model_stats(const scene::Model& model, int lod = -1);

class GpuModel {
public:
    GpuModel(GpuContext& ctx, const scene::Model& model);
    ~GpuModel();
    GpuModel(const GpuModel&) = delete;
    GpuModel& operator=(const GpuModel&) = delete;

    bool valid() const { return vertices_ != nullptr && indices_ != nullptr; }
    // Rewrites the vertex buffer (same layout and count as at construction, meshes in
    // order), e.g. camera-facing billboards. Returns false on a size mismatch.
    bool update_vertices(const std::vector<scene::Vertex>& all);
    size_t vertex_count() const { return vertex_count_; }

    struct MeshInfo {
        Mat4 transform;
        int lod = 0;
        s32 vertex_offset = 0; // added to every index
        u32 index_offset = 0;  // first triangle index in the shared buffer
        u32 line_first = 0, line_count = 0;
        std::vector<scene::Primitive> primitives; // first_index relative to index_offset
        // Bounding sphere of the mesh's vertices in mesh space (before `transform`),
        // used for frustum culling, draw distance and transparency sorting.
        Vec3 center;
        float radius = 0;
    };
    struct TextureInfo {
        SDL_GPUTexture* texture = nullptr;
        bool clamp_u = false, clamp_v = false;
    };

    const std::vector<MeshInfo>& meshes() const { return meshes_; }
    const std::vector<TextureInfo>& textures() const { return textures_; }
    SDL_GPUBuffer* vertex_buffer() const { return vertices_; }
    SDL_GPUBuffer* index_buffer() const { return indices_; }

private:
    SDL_GPUDevice* device_;
    SDL_GPUBuffer* vertices_ = nullptr;
    SDL_GPUBuffer* indices_ = nullptr;
    std::vector<MeshInfo> meshes_;
    std::vector<TextureInfo> textures_;
    size_t vertex_count_ = 0;
};

} // namespace gt3r::render
