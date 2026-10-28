#include "race/point_lights.h"

#include "render/gpu_model.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::race {

namespace {
float rdf(ByteSpan d, size_t o) {
    u32 v = rd32(d, o);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}
} // namespace

void CoursePointLights::load(ByteSpan g) {
    active_ = false;
    lamps_.clear();
    if (g.size() < 0x60 || std::memcmp(g.data(), "GTRW", 4) != 0) return;
    active_ = g[0x21] == 1;
    u32 n = rd32(g, 0x58), off = rd32(g, 0x5C);
    if (!active_ || n == 0 || n > 4096 || off + size_t(n) * 0x20 > g.size()) return;
    for (u32 i = 0; i < n; ++i) {
        size_t o = off + 0x20 * size_t(i);
        CourseLamp l;
        l.type = int(rd32(g, o) & 3);
        l.axis = {rdf(g, o + 4), rdf(g, o + 8), rdf(g, o + 12)};
        l.pos = {rdf(g, o + 0x10), rdf(g, o + 0x14), rdf(g, o + 0x18)};
        l.kind = rdf(g, o + 0x1C);
        lamps_.push_back(l);
    }
}

int first_candidate(int hint, int n) {
    if (n <= 0) return 0;
    int start = hint - 4;
    if (start < 0) start += n;
    if (start < 0 || start >= n) start = (start % n + n) % n; // hints below 4 - n or above n + 3 (none on the disc)
    return start;
}

std::vector<SelectedLamp> CoursePointLights::candidates(const SunParams& env, const render::Vec3& p, int hint) const {
    std::vector<SelectedLamp> out;
    int n = int(lamps_.size());
    if (n == 0) return out;
    int scan = std::min(n, 9);
    int start = first_candidate(hint, n);
    for (int k = 0; k < scan; ++k) {
        int idx = (start + k) % n;
        const CourseLamp& l = lamps_[size_t(idx)];
        const SunParams::LampType& t = env.lamps[size_t(l.type)];
        SelectedLamp s;
        s.type = l.type;
        s.lamp = idx;
        if (l.kind == 0) {
            s.dir = render::normalize(l.pos);
            s.w = 1;
        } else {
            render::Vec3 d = l.pos - p;
            float dist = render::length(d);
            s.dir = dist > 1e-4f ? d * (1.0f / dist) : render::Vec3{0, 1, 0};
            float den = t.c0 + t.c1 * dist + t.c2 * dist * dist;
            s.w = den > 1e-6f ? 1.0f / den : 0.0f;
            if (t.cutoff != 180.0f) {
                float c = std::max(0.0f, render::dot(s.dir, l.axis));
                if (std::cos(t.cutoff * 3.14159265f / 180.0f) <= c) s.w *= std::pow(c, t.spot_exp);
                else s.w = 0;
            }
        }
        out.push_back(s);
    }
    std::stable_sort(out.begin(), out.end(), [](const SelectedLamp& a, const SelectedLamp& b) { return a.w > b.w; });
    return out;
}

std::vector<SelectedLamp> CoursePointLights::select(const SunParams& env, const render::Vec3& p, int hint) const {
    std::vector<SelectedLamp> out = candidates(env, p, hint);
    if (out.size() > 4) out.resize(4);
    return out;
}

std::vector<ReflectionDisc> CoursePointLights::reflection_discs(const SunParams& env, const render::Vec3& p, int hint) const {
    std::vector<ReflectionDisc> out;
    for (const SelectedLamp& s : candidates(env, p, hint)) {
        const CourseLamp& l = lamps_[size_t(s.lamp)];
        if (l.kind == 0) continue;
        if (out.size() >= size_t(ReflectionDiscMesh::kMaxDiscs)) break;
        const SunParams::LampType& t = env.lamps[size_t(l.type)];
        ReflectionDisc d;
        d.pos = l.pos;
        d.rgba = t.disc_rgba;
        d.sqrt_w = std::sqrt(std::max(0.0f, s.w));
        d.size = t.disc_size;
        out.push_back(d);
    }
    return out;
}

ReflectionDiscMesh::ReflectionDiscMesh() = default;
ReflectionDiscMesh::~ReflectionDiscMesh() = default;

void ReflectionDiscMesh::update(render::GpuContext& ctx, const std::vector<ReflectionDisc>& discs, const render::Vec3& eye,
                                const render::Vec3 basis[3]) {
    constexpr int kPerDisc = 1 + 2 * kWedges; // centre, a middle ring, the rim
    eye_ = eye;
    any_ = false;
    verts_.assign(size_t(kMaxDiscs * kPerDisc), scene::Vertex{});
    // Unused slots: transparent, spread over the unit sphere (the mesh's bounds for culling).
    for (size_t k = 0; k < verts_.size(); ++k) verts_[k].color = 0, verts_[k].pos[k % 3] = (k / 3) % 2 ? -1.0f : 1.0f;
    // Map point (u, v) -> world direction: radius rho = sin(phi / 2), phi from the map's axis.
    auto dir = [&](float u, float v) {
        float rho = std::min(std::sqrt(u * u + v * v), 0.9999f);
        float phi = 2.0f * std::asin(rho);
        float k = rho > 1e-6f ? std::sin(phi) / rho : 2.0f;
        return render::normalize(basis[0] * (u * k) + basis[1] * (v * k) + basis[2] * std::cos(phi));
    };
    for (size_t i = 0; i < discs.size() && i < size_t(kMaxDiscs); ++i) {
        const ReflectionDisc& d = discs[i];
        render::Vec3 r = d.pos - eye;
        float dist = render::length(r);
        if (dist < 1e-3f || d.size <= 0) continue;
        r = r * (1.0f / dist);
        render::Vec3 m{render::dot(basis[0], r), render::dot(basis[1], r), render::dot(basis[2], r)};
        float mm = std::sqrt(m.x * m.x + m.y * m.y + (m.z + 1) * (m.z + 1));
        if (mm < 1e-4f) continue; // straight behind the map: not in it
        float cu = m.x / mm, cv = m.y / mm, R = d.size / dist;
        auto byte = [](float x) { return u32(std::clamp(x * 255.0f + 0.5f, 0.0f, 255.0f)); };
        u32 rgb = byte(d.rgba[0]) | byte(d.rgba[1]) << 8 | byte(d.rgba[2]) << 16;
        // Alpha x 128 on the GS (0x80 = 1.0): our 0..255 alpha.
        float a = std::clamp(d.rgba[3] * d.sqrt_w, 0.0f, 1.0f);
        scene::Vertex* v = &verts_[i * kPerDisc];
        auto put = [&](scene::Vertex& o, float u, float w, float alpha) {
            render::Vec3 p = dir(u, w);
            o.pos[0] = p.x, o.pos[1] = p.y, o.pos[2] = p.z;
            o.normal[0] = -p.x, o.normal[1] = -p.y, o.normal[2] = -p.z;
            o.color = rgb | byte(alpha) << 24;
        };
        put(v[0], cu, cv, a);
        for (int k = 0; k < kWedges; ++k) {
            // GT3's rim starts at (0, R) and steps by -36 degrees.
            float ang = 1.5707963f - float(k) * 0.62831853f;
            float cx = std::cos(ang), sy = std::sin(ang);
            put(v[1 + k], cu + 0.5f * R * cx, cv + 0.5f * R * sy, 0.5f * a); // Gouraud: half way
            put(v[1 + kWedges + k], cu + R * cx, cv + R * sy, 0.0f);
        }
        any_ = any_ || a > 0;
    }
    if (!gpu_) {
        scene::Model model;
        scene::Mesh mesh;
        mesh.name = "reflection_discs";
        mesh.vertices = verts_;
        for (int i = 0; i < kMaxDiscs; ++i) {
            u32 b = u32(i * kPerDisc);
            for (int k = 0; k < kWedges; ++k) {
                u32 m0 = b + 1 + u32(k), m1 = b + 1 + u32((k + 1) % kWedges);
                u32 r0 = m0 + kWedges, r1 = m1 + kWedges;
                for (u32 x : {b, m0, m1, m0, r0, r1, m0, r1, m1}) mesh.indices.push_back(x);
            }
        }
        scene::Primitive prim;
        prim.first_index = 0;
        prim.index_count = u32(mesh.indices.size());
        prim.texture = -1;
        prim.flags = scene::kAlphaBlend | scene::kUnlit | scene::kNoDepthWrite | scene::kDoubleSided;
        mesh.primitives.push_back(prim);
        model.meshes.push_back(std::move(mesh));
        gpu_ = std::make_unique<render::GpuModel>(ctx, model);
    } else {
        gpu_->update_vertices(verts_);
    }
}

void ReflectionDiscMesh::add_item(std::vector<render::DrawItem>& items) const {
    if (!gpu_ || !any_) return;
    render::DrawItem it;
    it.model = gpu_.get();
    it.transform = render::translate(eye_);
    it.lod = 0;
    items.push_back(it);
}

} // namespace gt3r::race
