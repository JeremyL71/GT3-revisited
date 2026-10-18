#include "render/test_models.h"

#include <algorithm>
#include <cmath>

namespace gt3r::render {

namespace {

constexpr float kPi = 3.14159265358979f;

struct Vec3f {
    float x, y, z;
};

constexpr u32 rgba(u32 r, u32 g, u32 b, u32 a = 255) { return r | (g << 8) | (b << 16) | (a << 24); }

scene::Texture checker(const char* name, u32 size, u32 cells, u32 c0, u32 c1, bool clamp) {
    scene::Texture t;
    t.name = name;
    t.width = t.height = size;
    t.rgba.resize(size_t(size) * size);
    for (u32 y = 0; y < size; ++y)
        for (u32 x = 0; x < size; ++x) {
            bool odd = ((x * cells / size) + (y * cells / size)) & 1;
            t.rgba[size_t(y) * size + x] = odd ? c1 : c0;
        }
    // A one-pixel red border makes wrap-vs-clamp visible.
    for (u32 i = 0; i < size; ++i) {
        t.rgba[i] = t.rgba[size_t(size - 1) * size + i] = rgba(220, 40, 40);
        t.rgba[size_t(i) * size] = t.rgba[size_t(i) * size + size - 1] = rgba(220, 40, 40);
    }
    t.clamp_u = t.clamp_v = clamp;
    return t;
}

// Soft round sprite, opaque centre fading to transparent edges.
scene::Texture glow(const char* name, u32 size, u32 rgb) {
    scene::Texture t;
    t.name = name;
    t.width = t.height = size;
    t.rgba.resize(size_t(size) * size);
    for (u32 y = 0; y < size; ++y)
        for (u32 x = 0; x < size; ++x) {
            float dx = (x + 0.5f) / size * 2 - 1, dy = (y + 0.5f) / size * 2 - 1;
            float a = std::clamp(1.0f - std::sqrt(dx * dx + dy * dy), 0.0f, 1.0f);
            t.rgba[size_t(y) * size + x] = (rgb & 0xFFFFFFu) | (u32(a * 255) << 24);
        }
    t.clamp_u = t.clamp_v = true;
    return t;
}

// Cut-out pattern: opaque stripes with fully transparent gaps (alpha test).
scene::Texture stripes(const char* name, u32 size) {
    scene::Texture t;
    t.name = name;
    t.width = t.height = size;
    t.rgba.resize(size_t(size) * size);
    for (u32 y = 0; y < size; ++y)
        for (u32 x = 0; x < size; ++x) {
            bool on = ((x + y) / (size / 8)) & 1;
            t.rgba[size_t(y) * size + x] = on ? rgba(250, 200, 40) : rgba(0, 0, 0, 0);
        }
    return t;
}

void add_vertex(scene::Mesh& m, float x, float y, float z, float nx, float ny, float nz, float u, float v, u32 c) {
    scene::Vertex vx;
    vx.pos[0] = x, vx.pos[1] = y, vx.pos[2] = z;
    vx.normal[0] = nx, vx.normal[1] = ny, vx.normal[2] = nz;
    vx.uv[0] = u, vx.uv[1] = v;
    vx.color = c;
    m.vertices.push_back(vx);
}

// Quad facing +Z in its local frame, placed at (cx,cy,cz), size w x h.
void append_quad(scene::Mesh& m, float cx, float cy, float cz, float w, float h, float uv_scale, u32 c) {
    u32 b = u32(m.vertices.size());
    add_vertex(m, cx - w / 2, cy - h / 2, cz, 0, 0, 1, 0, uv_scale, c);
    add_vertex(m, cx + w / 2, cy - h / 2, cz, 0, 0, 1, uv_scale, uv_scale, c);
    add_vertex(m, cx + w / 2, cy + h / 2, cz, 0, 0, 1, uv_scale, 0, c);
    add_vertex(m, cx - w / 2, cy + h / 2, cz, 0, 0, 1, 0, 0, c);
    for (u32 i : {0u, 1u, 2u, 0u, 2u, 3u}) m.indices.push_back(b + i);
}

void add_primitive(scene::Mesh& m, u32 first, int texture, u32 flags) {
    scene::Primitive p;
    p.first_index = first;
    p.index_count = u32(m.indices.size()) - first;
    p.texture = texture;
    p.flags = flags;
    if (p.index_count) m.primitives.push_back(p);
}

} // namespace

void append_sphere(scene::Mesh& m, float cx, float cy, float cz, float r, int slices, int stacks, u32 color) {
    u32 base = u32(m.vertices.size());
    for (int i = 0; i <= stacks; ++i) {
        float v = float(i) / stacks, phi = v * kPi;
        for (int j = 0; j <= slices; ++j) {
            float u = float(j) / slices, theta = u * 2 * kPi;
            float nx = std::sin(phi) * std::sin(theta), ny = std::cos(phi), nz = std::sin(phi) * std::cos(theta);
            // Vertex colour varies with height so unlit/lit differences are visible.
            u32 c = color;
            if (color == 0) c = rgba(u32(127 + 127 * nx), u32(127 + 127 * ny), u32(127 + 127 * nz));
            add_vertex(m, cx + r * nx, cy + r * ny, cz + r * nz, nx, ny, nz, u * 4, v * 2, c);
        }
    }
    for (int i = 0; i < stacks; ++i)
        for (int j = 0; j < slices; ++j) {
            u32 a = base + u32(i * (slices + 1) + j), b = a + u32(slices + 1);
            // Counter-clockwise when seen from outside.
            for (u32 k : {a, b, a + 1, a + 1, b, b + 1}) m.indices.push_back(k);
        }
}

void append_box(scene::Mesh& m, float cx, float cy, float cz, float hx, float hy, float hz, u32 color) {
    const Vec3f normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    const float corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    for (const Vec3f& n : normals) {
        // v = "up" on the face, u = v x n, so u x v = n and the corner order is counter-clockwise from outside.
        Vec3f v = n.y != 0 ? Vec3f{0, 0, -n.y} : Vec3f{0, 1, 0};
        Vec3f u{v.y * n.z - v.z * n.y, v.z * n.x - v.x * n.z, v.x * n.y - v.y * n.x};
        u32 b = u32(m.vertices.size());
        for (const auto& c : corners) {
            float px = cx + hx * (n.x + c[0] * u.x + c[1] * v.x);
            float py = cy + hy * (n.y + c[0] * u.y + c[1] * v.y);
            float pz = cz + hz * (n.z + c[0] * u.z + c[1] * v.z);
            add_vertex(m, px, py, pz, n.x, n.y, n.z, (c[0] + 1) / 2, (1 - c[1]) / 2, color);
        }
        for (u32 i : {0u, 1u, 2u, 0u, 2u, 3u}) m.indices.push_back(b + i);
    }
}

const char* test_model_name(int variant) {
    switch (((variant % kTestModelVariants) + kTestModelVariants) % kTestModelVariants) {
    case 0: return "Materials (cube, sphere, blend modes)";
    case 1: return "Environment map + unlit pyramid";
    default: return "LOD spheres";
    }
}

scene::Model make_test_model(int variant) {
    variant = ((variant % kTestModelVariants) + kTestModelVariants) % kTestModelVariants;
    scene::Model model;
    model.name = test_model_name(variant);

    if (variant == 0) {
        model.textures.push_back(checker("checker_clamp", 64, 8, rgba(235, 235, 235), rgba(40, 90, 200), true));
        model.textures.push_back(glow("glow", 64, rgba(255, 140, 40)));
        model.textures.push_back(stripes("stripes", 64));
        model.textures.push_back(checker("checker_wrap", 32, 4, rgba(200, 200, 200), rgba(60, 160, 80), false));

        scene::Mesh cube;
        cube.name = "cube";
        append_box(cube, -1.3f, 0.75f, 0, 0.75f, 0.75f, 0.75f, rgba(255, 255, 255));
        add_primitive(cube, 0, 0, 0);
        model.meshes.push_back(std::move(cube));

        scene::Mesh sphere;
        sphere.name = "sphere";
        append_sphere(sphere, 1.3f, 0.8f, 0, 0.8f, 32, 16, 0);
        add_primitive(sphere, 0, -1, 0);
        model.meshes.push_back(std::move(sphere));

        scene::Mesh fx;
        fx.name = "effects";
        u32 first = 0;
        append_quad(fx, 0, 2.3f, -0.6f, 1.2f, 1.2f, 1, rgba(255, 255, 255, 160)); // alpha blend (vertex alpha)
        add_primitive(fx, first, 3, scene::kAlphaBlend | scene::kDoubleSided);
        first = u32(fx.indices.size());
        append_quad(fx, 1.3f, 0.8f, 1.0f, 1.4f, 1.4f, 1, rgba(255, 255, 255)); // additive glow in front of sphere
        add_primitive(fx, first, 1, scene::kAdditive | scene::kDoubleSided | scene::kNoDepthWrite);
        first = u32(fx.indices.size());
        append_quad(fx, -1.3f, 0.75f, 0.9f, 1.4f, 1.4f, 1, rgba(255, 255, 255)); // alpha-tested fence
        add_primitive(fx, first, 2, scene::kAlphaTest | scene::kDoubleSided);
        first = u32(fx.indices.size());
        append_quad(fx, 0, 0.6f, -1.6f, 2.0f, 1.2f, 3, rgba(255, 255, 255)); // wrapped texture, single-sided
        add_primitive(fx, first, 3, 0);
        model.meshes.push_back(std::move(fx));
    } else if (variant == 1) {
        scene::Mesh chrome;
        chrome.name = "chrome sphere";
        append_sphere(chrome, -1.1f, 1.0f, 0, 1.0f, 48, 24, rgba(200, 40, 40));
        add_primitive(chrome, 0, -1, scene::kEnvironmentMap);
        model.meshes.push_back(std::move(chrome));

        scene::Mesh pyr;
        pyr.name = "unlit pyramid";
        const float apex[3] = {1.2f, 2.0f, 0};
        const float base[4][2] = {{0.2f, -1}, {2.2f, -1}, {2.2f, 1}, {0.2f, 1}};
        const u32 cols[4] = {rgba(255, 0, 0), rgba(0, 255, 0), rgba(0, 0, 255), rgba(255, 255, 0)};
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            u32 b = u32(pyr.vertices.size());
            add_vertex(pyr, base[i][0], 0, base[i][1], 0, 1, 0, 0, 1, cols[i]);
            add_vertex(pyr, apex[0], apex[1], apex[2], 0, 1, 0, 0.5f, 0, rgba(255, 255, 255));
            add_vertex(pyr, base[j][0], 0, base[j][1], 0, 1, 0, 1, 1, cols[j]);
            for (u32 k : {0u, 1u, 2u}) pyr.indices.push_back(b + k);
        }
        add_primitive(pyr, 0, -1, scene::kUnlit | scene::kDoubleSided);
        model.meshes.push_back(std::move(pyr));
    } else {
        const int detail[3][2] = {{32, 16}, {12, 6}, {6, 3}};
        for (int lod = 0; lod < 3; ++lod) {
            scene::Mesh m;
            m.name = "sphere lod" + std::to_string(lod);
            m.lod = lod;
            append_sphere(m, 0, 1, 0, 1, detail[lod][0], detail[lod][1], 0);
            add_primitive(m, 0, -1, 0);
            // Place LODs side by side via the mesh transform (column-major translation).
            m.transform[12] = (lod - 1) * 2.4f;
            model.meshes.push_back(std::move(m));
        }
    }
    model.compute_bounds();
    if (variant == 2) { // compute_bounds ignores mesh transforms; widen manually.
        model.bounds_min[0] = -3.4f;
        model.bounds_max[0] = 3.4f;
    }
    return model;
}

scene::Model make_ground_grid(const float bmin[3], const float bmax[3]) {
    scene::Model g;
    g.name = "grid";
    scene::Texture t;
    t.name = "grid";
    t.width = t.height = 64;
    t.rgba.assign(64 * 64, rgba(0, 0, 0, 0));
    for (u32 i = 0; i < 64; ++i)
        for (u32 k : {0u, 1u, 63u}) {
            t.rgba[k * 64 + i] = rgba(150, 160, 180, 200);
            t.rgba[i * 64 + k] = rgba(150, 160, 180, 200);
        }
    g.textures.push_back(std::move(t));

    float cx = (bmin[0] + bmax[0]) / 2, cz = (bmin[2] + bmax[2]) / 2;
    float ext = std::max({bmax[0] - bmin[0], bmax[2] - bmin[2], bmax[1] - bmin[1], 1e-3f});
    // Cell size: a "nice" power of ten fraction of the model size.
    float cell = std::pow(10.0f, std::floor(std::log10(ext / 2)));
    float half = std::ceil(ext * 1.5f / cell) * cell;
    float y = bmin[1];
    scene::Mesh m;
    m.name = "grid";
    float tiles = half * 2 / cell;
    u32 c = rgba(255, 255, 255);
    add_vertex(m, cx - half, y, cz + half, 0, 1, 0, 0, tiles, c);
    add_vertex(m, cx + half, y, cz + half, 0, 1, 0, tiles, tiles, c);
    add_vertex(m, cx + half, y, cz - half, 0, 1, 0, tiles, 0, c);
    add_vertex(m, cx - half, y, cz - half, 0, 1, 0, 0, 0, c);
    for (u32 i : {0u, 1u, 2u, 0u, 2u, 3u}) m.indices.push_back(i);
    add_primitive(m, 0, 0, scene::kAlphaBlend | scene::kDoubleSided | scene::kUnlit | scene::kNoDepthWrite);
    g.meshes.push_back(std::move(m));
    g.compute_bounds();
    return g;
}

} // namespace gt3r::render
