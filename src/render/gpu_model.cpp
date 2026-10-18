#include "render/gpu_model.h"

#include "base/log.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_set>

namespace gt3r::render {

ModelStats model_stats(const scene::Model& model, int lod) {
    ModelStats s;
    s.textures = model.textures.size();
    for (const scene::Mesh& m : model.meshes) {
        s.max_lod = std::max(s.max_lod, m.lod);
        if (lod >= 0 && m.lod != lod) continue;
        ++s.meshes;
        s.vertices += m.vertices.size();
        s.primitives += m.primitives.size();
        for (const scene::Primitive& p : m.primitives) s.triangles += p.index_count / 3;
    }
    return s;
}

GpuModel::GpuModel(GpuContext& ctx, const scene::Model& model) : device_(ctx.device()) {
    std::vector<scene::Vertex> vertices;
    std::vector<u32> indices;
    for (const scene::Mesh& m : model.meshes) {
        MeshInfo info;
        info.transform = Mat4::from(m.transform);
        info.lod = m.lod;
        info.vertex_offset = s32(vertices.size());
        info.index_offset = u32(indices.size());
        vertices.insert(vertices.end(), m.vertices.begin(), m.vertices.end());
        if (!m.vertices.empty()) {
            float lo[3], hi[3];
            for (int k = 0; k < 3; ++k) lo[k] = hi[k] = m.vertices[0].pos[k];
            for (const scene::Vertex& v : m.vertices)
                for (int k = 0; k < 3; ++k) lo[k] = std::min(lo[k], v.pos[k]), hi[k] = std::max(hi[k], v.pos[k]);
            info.center = {(lo[0] + hi[0]) * 0.5f, (lo[1] + hi[1]) * 0.5f, (lo[2] + hi[2]) * 0.5f};
            float r2 = 0;
            for (const scene::Vertex& v : m.vertices) {
                Vec3 d = Vec3{v.pos[0], v.pos[1], v.pos[2]} - info.center;
                r2 = std::max(r2, dot(d, d));
            }
            info.radius = std::sqrt(r2);
        }
        u32 nv = u32(m.vertices.size());
        // Drop out-of-range indices instead of letting the GPU read garbage.
        for (const scene::Primitive& p : m.primitives) {
            scene::Primitive q = p;
            q.first_index = u32(indices.size()) - info.index_offset;
            u32 end = std::min<u32>(p.first_index + p.index_count, u32(m.indices.size()));
            u32 tri_end = p.first_index + (end > p.first_index ? (end - p.first_index) / 3 * 3 : 0);
            for (u32 i = p.first_index; i < tri_end; i += 3) {
                u32 a = m.indices[i], b = m.indices[i + 1], c = m.indices[i + 2];
                if (a >= nv || b >= nv || c >= nv) continue;
                indices.insert(indices.end(), {a, b, c});
            }
            q.index_count = u32(indices.size()) - info.index_offset - q.first_index;
            if (q.texture >= int(model.textures.size())) q.texture = -1;
            if (q.index_count) info.primitives.push_back(q);
        }
        // Unique edges for the wireframe view.
        info.line_first = u32(indices.size()) - info.index_offset;
        std::unordered_set<u64> edges;
        size_t tri_count = indices.size() - info.index_offset;
        for (size_t i = info.index_offset; i < info.index_offset + tri_count; i += 3)
            for (int e = 0; e < 3; ++e) {
                u32 a = indices[i + e], b = indices[i + (e + 1) % 3];
                if (a > b) std::swap(a, b);
                if (edges.insert((u64(a) << 32) | b).second) indices.insert(indices.end(), {a, b});
            }
        info.line_count = u32(indices.size()) - info.index_offset - info.line_first;
        meshes_.push_back(std::move(info));
    }

    vertex_count_ = vertices.size();
    Uploader up(device_);
    if (!vertices.empty() && !indices.empty()) {
        vertices_ = up.buffer(SDL_GPU_BUFFERUSAGE_VERTEX, vertices.data(), u32(vertices.size() * sizeof(scene::Vertex)));
        indices_ = up.buffer(SDL_GPU_BUFFERUSAGE_INDEX, indices.data(), u32(indices.size() * sizeof(u32)));
    }
    for (const scene::Texture& t : model.textures) {
        TextureInfo ti;
        ti.clamp_u = t.clamp_u;
        ti.clamp_v = t.clamp_v;
        if (t.width && t.height && t.rgba.size() >= size_t(t.width) * t.height)
            ti.texture = up.texture_rgba(t.width, t.height, t.rgba.data(), true);
        if (!ti.texture) LOG_WARN("Texture '%s' (%ux%u) not uploaded", t.name.c_str(), t.width, t.height);
        textures_.push_back(ti);
    }
    if (!up.submit()) LOG_WARN("Model upload failed: %s", SDL_GetError());
}

bool GpuModel::update_vertices(const std::vector<scene::Vertex>& all) {
    if (!vertices_ || all.size() != vertex_count_) return false;
    u32 size = u32(all.size() * sizeof(scene::Vertex));
    SDL_GPUTransferBufferCreateInfo ti{};
    ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    ti.size = size;
    SDL_GPUTransferBuffer* t = SDL_CreateGPUTransferBuffer(device_, &ti);
    if (!t) return false;
    if (void* p = SDL_MapGPUTransferBuffer(device_, t, false)) {
        std::memcpy(p, all.data(), size);
        SDL_UnmapGPUTransferBuffer(device_, t);
    }
    bool ok = false;
    if (SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(device_)) {
        SDL_GPUCopyPass* cp = SDL_BeginGPUCopyPass(cmd);
        SDL_GPUTransferBufferLocation src{t, 0};
        SDL_GPUBufferRegion dst{vertices_, 0, size};
        SDL_UploadToGPUBuffer(cp, &src, &dst, true); // cycle: frames still drawing keep the old data
        SDL_EndGPUCopyPass(cp);
        ok = SDL_SubmitGPUCommandBuffer(cmd);
    }
    SDL_ReleaseGPUTransferBuffer(device_, t);
    return ok;
}

GpuModel::~GpuModel() {
    if (vertices_) SDL_ReleaseGPUBuffer(device_, vertices_);
    if (indices_) SDL_ReleaseGPUBuffer(device_, indices_);
    for (TextureInfo& t : textures_)
        if (t.texture) SDL_ReleaseGPUTexture(device_, t.texture);
}

} // namespace gt3r::render
