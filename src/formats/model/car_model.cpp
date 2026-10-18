#include "formats/model/car_model.h"
#include "formats/model/gltf.h"

#include "formats/model/wheel_model.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r {

namespace {

const char* const kSectionMagic[4] = {"GTCI", "GTM1", "GTTR", "GTTW"};

// Procedural tyre (the game does not store tyre geometry): a tread band and
// two sidewalls around the X axis, centred at the origin. The GTTR texture
// has the tread in its top half and the sidewall lettering in the bottom.
scene::Mesh make_tyre(float radius, float rim_radius, float width, int texture, float repeat) {
    constexpr int kSegments = 32;
    constexpr float kPi = 3.14159265f;
    scene::Mesh m;
    m.name = "tyre";
    auto add = [&](float x, float ang, float r, float nx, float ny_scale, float u, float v) {
        scene::Vertex vt;
        float c = std::cos(ang), s = std::sin(ang);
        vt.pos[0] = x, vt.pos[1] = r * c, vt.pos[2] = r * s;
        float n[3] = {nx, ny_scale * c, ny_scale * s};
        float l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        for (int k = 0; k < 3; ++k) vt.normal[k] = n[k] / l;
        vt.uv[0] = u, vt.uv[1] = v;
        m.vertices.push_back(vt);
        return u32(m.vertices.size() - 1);
    };
    float hw = width * 0.5f;
    for (int i = 0; i < kSegments; ++i) {
        float a0 = 2 * kPi * i / kSegments, a1 = 2 * kPi * (i + 1) / kSegments;
        float u0 = repeat * i / kSegments, u1 = repeat * (i + 1) / kSegments;
        // tread (normal outwards)
        u32 a = add(-hw, a0, radius, 0, 1, u0, 0.0f), b = add(hw, a0, radius, 0, 1, u0, 0.5f);
        u32 c = add(-hw, a1, radius, 0, 1, u1, 0.0f), d = add(hw, a1, radius, 0, 1, u1, 0.5f);
        m.indices.insert(m.indices.end(), {a, c, b, b, c, d});
        // sidewalls (normals -X / +X)
        for (int side = -1; side <= 1; side += 2) {
            float x = side * hw;
            u32 o0 = add(x, a0, radius, float(side), 0, u0, 0.55f), i0 = add(x, a0, rim_radius, float(side), 0, u0, 0.95f);
            u32 o1 = add(x, a1, radius, float(side), 0, u1, 0.55f), i1 = add(x, a1, rim_radius, float(side), 0, u1, 0.95f);
            if (side < 0) m.indices.insert(m.indices.end(), {o0, i0, o1, o1, i0, i1});
            else m.indices.insert(m.indices.end(), {o0, o1, i0, o1, i1, i0});
        }
    }
    scene::Primitive p;
    p.index_count = u32(m.indices.size());
    p.texture = texture;
    m.primitives.push_back(p);
    return m;
}

// Body-based guess used when no database spec is available.
WheelSpec estimate_wheels(const scene::Model& car) {
    WheelSpec w;
    float len = car.bounds_max[2] - car.bounds_min[2];
    float half_w = std::max(std::fabs(car.bounds_min[0]), std::fabs(car.bounds_max[0]));
    w.wheelbase = 0.6f * len;
    for (int a = 0; a < 2; ++a) {
        w.width[a] = 0.22f;
        w.sidewall[a] = 0.1f;
        w.radius[a] = 0.32f;
        w.track[a] = 2 * (half_w - 0.04f) - w.width[a];
    }
    return w;
}

// Wheel i: axle i >> 1 (front, rear), side i & 1 (left = -X, right = +X).
// The wheel GTM1 is unit-sized with its outer face at x = 0 pointing -X (a
// left wheel). The game scales it by R - 0.75 * sidewall and puts the face at
// the tyre's outer sidewall; right wheels are rotated 180 degrees about Y.
void add_wheels(scene::Model& car, const scene::Model& wheel, int tyre_texture, float tyre_repeat,
                const WheelSpec& ws) {
    static const char* names[4] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
    int tex_base = int(car.textures.size());
    car.textures.insert(car.textures.end(), wheel.textures.begin(), wheel.textures.end());
    for (int i = 0; i < 4; ++i) {
        int axle = i >> 1;
        bool right = i & 1;
        float r = ws.radius[axle], w = ws.width[axle], sw = ws.sidewall[axle];
        float rim = r - 0.75f * sw;
        float cx = (right ? 0.5f : -0.5f) * ws.track[axle];
        float cy = r;
        float cz = (axle == 0 ? -0.5f : 0.5f) * ws.wheelbase;
        auto place = [&](scene::Vertex& v, float scale, float dx) {
            float px = v.pos[0] * scale + dx, py = v.pos[1] * scale, pz = v.pos[2] * scale;
            if (right) px = -px, pz = -pz, v.normal[0] = -v.normal[0], v.normal[2] = -v.normal[2];
            v.pos[0] = px + cx, v.pos[1] = py + cy, v.pos[2] = pz + cz;
        };
        for (const scene::Mesh& src : wheel.meshes) {
            scene::Mesh m = src;
            m.name = names[i];
            for (scene::Vertex& v : m.vertices) place(v, rim, -0.5f * w);
            for (scene::Primitive& p : m.primitives)
                if (p.texture >= 0) p.texture += tex_base;
            car.meshes.push_back(std::move(m));
        }
        scene::Mesh t = make_tyre(r, rim, w, tyre_texture, tyre_repeat);
        t.name = std::string(names[i]) + "_tyre";
        for (scene::Vertex& v : t.vertices) place(v, 1.0f, 0.0f);
        car.meshes.push_back(std::move(t));
    }
}

} // namespace

bool CarFile::is_car(ByteSpan data) {
    if (data.size() < 0x40 || rd32(data, 0) != 0) return false;
    for (int i = 0; i < 4; ++i) {
        u32 o = rd32(data, 4 + 4 * i);
        if (o + 4 > data.size() || std::memcmp(data.data() + o, kSectionMagic[i], 4) != 0) return false;
    }
    return true;
}

CarFile CarFile::parse(ByteSpan data) {
    if (!is_car(data)) throw FormatError("not a car model file");
    u32 off[6];
    for (int i = 0; i < 6; ++i) off[i] = rd32(data, 4 * i);
    if (off[5] > data.size()) throw FormatError("car model truncated");
    for (int i = 1; i < 5; ++i)
        if (off[i + 1] < off[i]) throw FormatError("car model sections out of order");
    CarFile c;
    c.info = data.subspan(off[1], off[2] - off[1]);
    c.body = data.subspan(off[2], off[3] - off[2]);
    c.tire = data.subspan(off[3], off[4] - off[3]);
    c.wheel = data.subspan(off[4], off[5] - off[4]);
    return c;
}

namespace {

// Appends `src` to `dst`, remapping texture indices.
void merge(scene::Model& dst, scene::Model&& src) {
    int base = int(dst.textures.size());
    for (scene::Texture& t : src.textures) dst.textures.push_back(std::move(t));
    for (scene::Mesh& m : src.meshes) {
        for (scene::Primitive& p : m.primitives)
            if (p.texture >= 0) p.texture += base;
        dst.meshes.push_back(std::move(m));
    }
}

scene::Model car_lod(const CarFile& f, const gtm1::Gtm1& g, const CarModelOptions& opt, int lod,
                     const std::string& name) {
    gtm1::Options o;
    o.run.lod = lod;
    o.run.brake = opt.brake;
    o.reflection = opt.reflection;
    o.node = opt.effects ? -1 : 0;
    o.paint = opt.paint;
    o.program_alpha_test = true;  // wheel-arch / grille cut-outs
    o.weighted_reflection = true; // reflection passes weighted by the body's texel alpha
    scene::Model m = gtm1::build_model(g, o, name);
    for (scene::Mesh& mesh : m.meshes)
        if (mesh.name == "node0") mesh.name = "body";
    m.compute_bounds();
    if (opt.wheels) {
        gtm1::Options wo;
        wo.reflection = opt.reflection;
        wo.run.lod = lod;
        wo.program_alpha_test = true;
        wo.weighted_reflection = true;
        scene::Model wheel = load_wheel_model(opt.wheel_override.empty() ? f.wheel : opt.wheel_override, wo, "wheel");
        int tyre_tex = -1;
        float repeat = 2;
        if (f.tire.size() > 0x20 && rd32(f.tire, 0x1C) < f.tire.size()) {
            tyre_tex = gtm1::decode_texture_set(f.tire.subspan(rd32(f.tire, 0x1C)), m.textures, "tyre_");
            repeat = float(std::clamp<u32>(rd32(f.tire, 0x10), 1, 8)); // texture repeats around the tread
        }
        add_wheels(m, wheel, tyre_tex, repeat, opt.wheel_spec ? *opt.wheel_spec : estimate_wheels(m));
    }
    for (scene::Mesh& mesh : m.meshes) mesh.lod = lod;
    return m;
}

} // namespace

int gtm1_lod_count(const gtm1::Gtm1& g, u32 node) {
    std::vector<gtm1::Draw> prev = g.run(node, {0, 0, 0});
    for (int lod = 1; lod < 8; ++lod) {
        std::vector<gtm1::Draw> cur = g.run(node, {lod, 0, 0});
        bool same = cur.size() == prev.size();
        for (size_t i = 0; same && i < cur.size(); ++i) same = cur[i].chunk == prev[i].chunk;
        if (same) return lod;
        prev = std::move(cur);
    }
    return 8;
}

void fit_body(scene::Model& m, const BodyFit& f) {
    if (f.from_wheelbase <= 0 || f.to_wheelbase <= 0 || f.from_width <= 0 || f.from_height <= 0) return;
    const float hw0 = f.from_wheelbase * 0.0005f, hw1 = f.to_wheelbase * 0.0005f; // half wheelbase, m
    const float oh0 = f.from_length - f.from_wheelbase, oh1 = f.to_length - f.to_wheelbase;
    const float kz = hw1 / hw0, ko = oh0 > 1 && oh1 > 1 ? oh1 / oh0 : kz;
    const float kx = f.to_width / f.from_width, ky = f.to_height / f.from_height;
    for (scene::Mesh& mesh : m.meshes) {
        if (mesh.name.rfind("wheel_", 0) == 0) continue;
        for (scene::Vertex& v : mesh.vertices) {
            float z = v.pos[2], a = std::fabs(z);
            float dz = a <= hw0 ? kz : ko; // slope of the map at z (for the normals)
            v.pos[2] = a <= hw0 ? z * kz : std::copysign(hw1 + (a - hw0) * ko, z);
            v.pos[0] *= kx;
            v.pos[1] *= ky;
            float n[3] = {v.normal[0] / kx, v.normal[1] / ky, v.normal[2] / dz};
            float l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (l > 0)
                for (int i = 0; i < 3; ++i) v.normal[i] = n[i] / l;
        }
    }
    m.compute_bounds();
}

namespace {

// Mod cars: stretch the borrowed body, then (with a glTF body) put the mod's
// body in place of the car file's, at every LOD the file has.
void apply_body_mod(scene::Model& m, const CarBodyMod& mod, const std::string& name) {
    if (mod.fit) fit_body(m, *mod.fit);
    if (!mod.glb) return;
    GlbOptions go;
    go.paint = mod.paint;
    scene::Model own = load_glb(*mod.glb, go, name);
    std::vector<int> lods;
    std::vector<scene::Mesh> kept;
    for (scene::Mesh& mesh : m.meshes) {
        bool body = mesh.name.rfind("wheel_", 0) != 0 && mesh.name.rfind("node", 0) != 0;
        if (body) {
            if (std::find(lods.begin(), lods.end(), mesh.lod) == lods.end()) lods.push_back(mesh.lod);
        } else {
            kept.push_back(std::move(mesh));
        }
    }
    if (lods.empty()) lods.push_back(0);
    const int tex0 = int(m.textures.size());
    for (scene::Texture& t : own.textures) m.textures.push_back(t);
    m.meshes = std::move(kept);
    for (scene::Mesh& mesh : own.meshes)
        for (scene::Vertex& v : mesh.vertices) v.pos[1] -= mod.glb_ground;
    for (int lod : lods)
        for (const scene::Mesh& src : own.meshes) {
            scene::Mesh mesh = src;
            mesh.lod = lod;
            for (scene::Primitive& p : mesh.primitives)
                if (p.texture >= 0) p.texture += tex0;
            m.meshes.push_back(std::move(mesh));
        }
}

} // namespace

scene::Model load_car_model(ByteSpan data, const CarModelOptions& opt, const std::string& name) {
    CarFile f = CarFile::parse(data);
    gtm1::Gtm1 g = gtm1::Gtm1::parse(f.body);
    if (opt.lod >= 0) {
        scene::Model m = car_lod(f, g, opt, opt.lod, name);
        if (opt.body_mod.active()) apply_body_mod(m, opt.body_mod, name);
        m.compute_bounds();
        return m;
    }
    scene::Model all;
    all.name = name;
    int n = g.nodes.empty() ? 0 : gtm1_lod_count(g, 0);
    for (int lod = 0; lod < n; ++lod) merge(all, car_lod(f, g, opt, lod, name));
    if (opt.body_mod.active()) apply_body_mod(all, opt.body_mod, name);
    all.compute_bounds();
    return all;
}

} // namespace gt3r
