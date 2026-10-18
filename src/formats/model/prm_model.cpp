#include "formats/model/prm_model.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::prm {

namespace {

float rdf(ByteSpan d, size_t o) {
    u32 v = rd32(d, o);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}

Matrix read_matrix(ByteSpan d, size_t o) {
    Matrix m;
    for (int k = 0; k < 16; ++k) m[k] = rdf(d, o + 4 * k);
    return m;
}

std::array<float, 4> read_vec4(ByteSpan d, size_t o) {
    return {rdf(d, o), rdf(d, o + 4), rdf(d, o + 8), rdf(d, o + 12)};
}

constexpr Matrix kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

struct Quat {
    float w, x, y, z;
};

// Rotation part of a row-vector matrix (rows = basis vectors) to a quaternion.
// `r` is a normalized 3x3, r[i][j] = row i, column j.
Quat to_quat(const float r[3][3]) {
    // Column-vector rotation R = transpose(r).
    auto R = [&](int i, int j) { return r[j][i]; };
    float tr = R(0, 0) + R(1, 1) + R(2, 2);
    Quat q;
    if (tr > 0) {
        float s = std::sqrt(tr + 1.0f) * 2;
        q = {0.25f * s, (R(2, 1) - R(1, 2)) / s, (R(0, 2) - R(2, 0)) / s, (R(1, 0) - R(0, 1)) / s};
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        float s = std::sqrt(1.0f + R(0, 0) - R(1, 1) - R(2, 2)) * 2;
        q = {(R(2, 1) - R(1, 2)) / s, 0.25f * s, (R(0, 1) + R(1, 0)) / s, (R(0, 2) + R(2, 0)) / s};
    } else if (R(1, 1) > R(2, 2)) {
        float s = std::sqrt(1.0f + R(1, 1) - R(0, 0) - R(2, 2)) * 2;
        q = {(R(0, 2) - R(2, 0)) / s, (R(0, 1) + R(1, 0)) / s, 0.25f * s, (R(1, 2) + R(2, 1)) / s};
    } else {
        float s = std::sqrt(1.0f + R(2, 2) - R(0, 0) - R(1, 1)) * 2;
        q = {(R(1, 0) - R(0, 1)) / s, (R(0, 2) + R(2, 0)) / s, (R(1, 2) + R(2, 1)) / s, 0.25f * s};
    }
    return q;
}

void from_quat(const Quat& q, float r[3][3]) {
    float w = q.w, x = q.x, y = q.y, z = q.z;
    float R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                     {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                     {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i][j] = R[j][i];
}

Quat slerp(Quat a, Quat b, float t) {
    float d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (d < 0) b = {-b.w, -b.x, -b.y, -b.z}, d = -d;
    float wa = 1 - t, wb = t;
    if (d < 0.9995f) {
        float th = std::acos(std::min(d, 1.0f)), s = std::sin(th);
        wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
    }
    Quat q{wa * a.w + wb * b.w, wa * a.x + wb * b.x, wa * a.y + wb * b.y, wa * a.z + wb * b.z};
    float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

Matrix interpolate(const Matrix& a, const Matrix& b, float t) {
    float ra[3][3], rb[3][3], sa[3], sb[3];
    bool ok = true;
    for (int i = 0; i < 3; ++i) {
        sa[i] = std::sqrt(a[4 * i] * a[4 * i] + a[4 * i + 1] * a[4 * i + 1] + a[4 * i + 2] * a[4 * i + 2]);
        sb[i] = std::sqrt(b[4 * i] * b[4 * i] + b[4 * i + 1] * b[4 * i + 1] + b[4 * i + 2] * b[4 * i + 2]);
        ok &= sa[i] > 1e-6f && sb[i] > 1e-6f;
        for (int j = 0; j < 3 && ok; ++j) ra[i][j] = a[4 * i + j] / sa[i], rb[i][j] = b[4 * i + j] / sb[i];
    }
    Matrix m;
    if (!ok) { // degenerate: plain linear blend
        for (int k = 0; k < 16; ++k) m[k] = a[k] + (b[k] - a[k]) * t;
        return m;
    }
    float r[3][3];
    from_quat(slerp(to_quat(ra), to_quat(rb), t), r);
    for (int i = 0; i < 3; ++i) {
        float s = sa[i] + (sb[i] - sa[i]) * t;
        for (int j = 0; j < 3; ++j) m[4 * i + j] = r[i][j] * s;
        m[4 * i + 3] = a[4 * i + 3] + (b[4 * i + 3] - a[4 * i + 3]) * t;
    }
    for (int j = 0; j < 4; ++j) m[12 + j] = a[12 + j] + (b[12 + j] - a[12 + j]) * t;
    return m;
}

} // namespace

bool PrmFile::is_prm(ByteSpan data) {
    return data.size() >= 0x40 && (has_magic(data, "PRZ0") || has_magic(data, "PRM0"));
}

PrmFile PrmFile::parse(ByteSpan data) {
    if (!is_prm(data)) throw FormatError("not a PRZ0/PRM0 prize scene");
    PrmFile p;
    p.prm0 = has_magic(data, "PRM0");
    p.start_frame = rdf(data, 0x10);
    p.last_frame = rdf(data, 0x14);
    p.loop_frame = rdf(data, 0x18);
    p.fps = rdf(data, 0x1C);
    u16 nlights = rd16(data, 0x20), nkeys = rd16(data, 0x22);
    u32 view = rd32(data, 0x24), model = rd32(data, 0x28), gtm = rd32(data, 0x2C), lights = rd32(data, 0x30);
    p.flags = rd16(data, 0x34);
    p.display[0] = rdf(data, 0x38);
    p.display[1] = rdf(data, 0x3C);
    if (nlights > 8) throw FormatError("PRM: too many lights");
    auto track = [&](u32 off, bool single, std::vector<Matrix>& out) {
        if (!off) return;
        size_t n = single ? 1 : nkeys;
        if (size_t(off) + n * 64 > data.size()) throw FormatError("PRM: key track out of range");
        for (size_t i = 0; i < n; ++i) out.push_back(read_matrix(data, off + 64 * i));
    };
    track(view, p.flags & 1, p.view_keys);
    track(model, p.flags & 2, p.model_keys);
    if (lights) {
        if (size_t(lights) + size_t(nlights) * 64 > data.size()) throw FormatError("PRM: lights out of range");
        for (u32 i = 0; i < nlights; ++i) {
            size_t o = lights + 64 * i;
            p.lights.push_back({read_vec4(data, o), read_vec4(data, o + 0x10), read_vec4(data, o + 0x20),
                                read_vec4(data, o + 0x30)});
        }
    }
    if (!gtm || gtm >= data.size()) throw FormatError("PRM: GTM1 offset out of range");
    p.gtm1 = data.subspan(gtm);
    if (!gtm1::Gtm1::is_gtm1(p.gtm1)) throw FormatError("PRM: no GTM1 at the model offset");
    return p;
}

float PrmFile::advance(float t, float ticks) const {
    t += ticks * fps / 60.0f;
    for (int i = 0; i < 21 && t >= last_frame + 1.0f; ++i) t = loop_frame + (t - (last_frame + 1.0f));
    return t;
}

static Matrix sample(const std::vector<Matrix>& keys, const PrmFile& p, float t) {
    if (keys.empty()) return kIdentity;
    if (keys.size() == 1) return keys[0];
    float ft = std::floor(t);
    int i = int(ft);
    int j = (i == int(p.last_frame)) ? int(p.loop_frame) : i + 1;
    int n = int(keys.size());
    i = std::clamp(i, 0, n - 1), j = std::clamp(j, 0, n - 1);
    return interpolate(keys[size_t(i)], keys[size_t(j)], t - ft);
}

Matrix PrmFile::view_at(float t) const { return sample(view_keys, *this, t); }
Matrix PrmFile::model_at(float t) const { return sample(model_keys, *this, t); }

bool shadow_matrix(const Light& l, int index, Matrix& out) {
    float vy = l.direction[1];
    if (vy == 0) return false;
    out = kIdentity;
    out[5] = 0;
    if (std::fabs(vy) >= 1e-4f) { // nearly horizontal light: flatten straight down
        out[4] = -l.direction[0] / vy;
        out[6] = -l.direction[2] / vy;
    }
    out[13] = -0.1f * float(index + 1);
    return true;
}

scene::Model load_prm_model(ByteSpan data, const gtm1::Options& opt, const std::string& name) {
    gtm1::Options o = opt;
    o.static_env_maps = true;
    if (o.node < 0) o.node = 0; // node 1 (when present) is the shadow silhouette
    return gtm1::decode_gtm1(PrmFile::parse(data).gtm1, o, name);
}

scene::Model load_prm_shadow(ByteSpan data, const std::string& name) {
    PrmFile p = PrmFile::parse(data);
    gtm1::Gtm1 g = gtm1::Gtm1::parse(p.gtm1);
    if (g.nodes.size() < 2) {
        scene::Model m;
        m.name = name;
        return m;
    }
    gtm1::Options o;
    o.node = 1;
    o.static_env_maps = true;
    return gtm1::build_model(g, o, name);
}

Matrix multiply(const Matrix& a, const Matrix& b) {
    Matrix m{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k) m[4 * i + j] += a[4 * i + k] * b[4 * k + j];
    return m;
}

void apply_sphere_map(scene::Model& model, const Matrix& model_view) {
    for (scene::Mesh& mesh : model.meshes)
        for (const scene::Primitive& p : mesh.primitives) {
            if (!(p.flags & scene::kEnvironmentMap) || p.texture < 0) continue;
            for (u32 i = p.first_index; i < p.first_index + p.index_count; ++i) {
                scene::Vertex& v = mesh.vertices[mesh.indices[i]];
                const Matrix& m = model_view;
                float n[3];
                for (int k = 0; k < 3; ++k) n[k] = v.normal[0] * m[k] + v.normal[1] * m[4 + k] + v.normal[2] * m[8 + k];
                float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                if (len > 0) n[0] /= len, n[1] /= len;
                v.uv[0] = 0.5f + 0.5f * n[0];
                v.uv[1] = 0.5f - 0.5f * n[1];
            }
        }
}

} // namespace gt3r::prm
