#include "race/pylons.h"

#include "base/log.h"
#include "formats/course/course_file.h"
#include "formats/model/gtm1.h"
#include "render/gpu_context.h"
#include "render/gpu_model.h"
#include "track/track.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::race {

namespace {

using sim::Vec3;

float rdf(ByteSpan d, size_t o) {
    u32 v = rd32(d, o);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}

// Row-major 3x3 helpers (m = body -> world).
Vec3 mul(const float* m, const Vec3& v) {
    return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z, m[6] * v.x + m[7] * v.y + m[8] * v.z};
}
Vec3 mul_t(const float* m, const Vec3& v) { // transpose: world -> body
    return {m[0] * v.x + m[3] * v.y + m[6] * v.z, m[1] * v.x + m[4] * v.y + m[7] * v.z, m[2] * v.x + m[5] * v.y + m[8] * v.z};
}
void matmul(const float* a, const float* b, float* out) {
    float r[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    std::memcpy(out, r, sizeof r);
}
// Rotation by angle |w| about w (Rodrigues).
void axis_angle(const Vec3& w, float* out) {
    float a = w.length();
    if (a < 1e-9f) {
        const float id[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::memcpy(out, id, sizeof id);
        return;
    }
    Vec3 k = w * (1.0f / a);
    float c = std::cos(a), s = std::sin(a), t = 1 - c;
    const float r[9] = {t * k.x * k.x + c,       t * k.x * k.y - s * k.z, t * k.x * k.z + s * k.y,
                        t * k.x * k.y + s * k.z, t * k.y * k.y + c,       t * k.y * k.z - s * k.x,
                        t * k.x * k.z - s * k.y, t * k.y * k.z + s * k.x, t * k.z * k.z + c};
    std::memcpy(out, r, sizeof r);
}
void orthonormalize(float* m) {
    Vec3 x{m[0], m[3], m[6]}, y{m[1], m[4], m[7]};
    x = x * (1.0f / std::max(1e-9f, x.length()));
    y = y - x * x.dot(y);
    y = y * (1.0f / std::max(1e-9f, y.length()));
    Vec3 z = x.cross(y);
    m[0] = x.x, m[3] = x.y, m[6] = x.z, m[1] = y.x, m[4] = y.y, m[7] = y.z, m[2] = z.x, m[5] = z.y, m[8] = z.z;
}

// Ground plane under (x, z): normal n and d with n.p + d = 0 on the surface.
bool ground_plane(const sim::TrackQuery& g, const Vec3& c, Vec3& n, float& d) {
    sim::SurfaceHit h = g.probe(c.x, c.z, c.y + 3.0f);
    if (!h.hit) return false;
    n = h.normal;
    float l = n.length();
    if (l < 1e-6f) n = {0, 1, 0};
    else n = n * (1.0f / l);
    d = -(n.x * c.x + n.y * h.height + n.z * c.z);
    return true;
}

// Inverse inertia (world) times v, with body-frame principal moments scaled by k.
Vec3 inv_inertia(const float* R, const Vec3& I, float k, const Vec3& v) {
    Vec3 b = mul_t(R, v);
    b = {b.x / (I.x * k), b.y / (I.y * k), b.z / (I.z * k)};
    return mul(R, b);
}

// Contact impulse at world offset r from the centre with unit normal n against a static
// (or infinitely heavy) partner moving with velocity `other` at that point.
// Returns the normal impulse (0 when separating).
float impulse(Pylons::Body& b, const Pylons::Type& t, const Vec3& r, const Vec3& n, const Vec3& other, float e,
              float inertia_scale, float mu) {
    Vec3 w = mul(b.rot, b.omega);
    Vec3 vp = b.vel + w.cross(r) - other;
    float vn = vp.dot(n);
    if (e > 0 && vp.dot(vp) < 0.81f) e = 0; // under 0.9 m/s: no bounce
    float K = 1.0f / t.mass + n.dot(inv_inertia(b.rot, t.inertia, inertia_scale, r.cross(n)).cross(r));
    float j = -(1 + e) * vn / std::max(K, 1e-6f);
    if (j < 0) return 0;
    Vec3 J = n * j;
    if (mu > 0) { // Coulomb friction: may stop the slide, never reverse it
        Vec3 vt = vp - n * vn;
        float lt = vt.length();
        if (lt > 1e-6f) {
            float f = std::min(mu * j / t.mass / lt, 1.0f);
            J += vt * (-f * t.mass);
        }
    }
    b.vel += J * (1.0f / t.mass);
    Vec3 dw = inv_inertia(b.rot, t.inertia, inertia_scale, r.cross(J));
    b.omega += mul_t(b.rot, dw);
    return j;
}

} // namespace

Pylons::Pylons() = default;
Pylons::~Pylons() = default;

bool Pylons::load(render::GpuContext* ctx, ByteSpan data) {
    types_.clear();
    records_.clear();
    bodies_.clear();
    ctx_ = ctx;
    try {
        course::CourseFile f = course::CourseFile::parse(data);
        ByteSpan tb = f.section(data, 36), road = f.section(data, course::kSlotRoad);
        if (tb.size() < 0x44 || road.size() < 0x58) return false;
        for (size_t k = 0; k < 15; ++k) {
            u32 off = rd32(tb, 4 + 4 * k);
            if (off == 0 || off + 0xB0 > tb.size()) break;
            ByteSpan d = tb.subspan(off);
            Type t;
            u32 n = std::min<u32>(rd32(d, 0), 8);
            t.inertia = {rdf(d, 4), rdf(d, 8), rdf(d, 0xC)};
            for (u32 i = 0; i < n; ++i) t.hull.push_back({rdf(d, 0x10 + 16 * i), rdf(d, 0x14 + 16 * i), rdf(d, 0x18 + 16 * i)});
            t.radius = rdf(d, 0x90), t.mass = std::max(0.1f, rdf(d, 0x94));
            u32 gm = rd32(d, 0xAC);
            if (ctx && gm > 0 && off + gm < tb.size()) {
                ByteSpan g = d.subspan(gm);
                if (gtm1::Gtm1::is_gtm1(g)) {
                    gtm1::Options o;
                    o.node = 0; // stream 0: the body (stream 1 is its planar shadow)
                    o.alpha_test = true;
                    gtm1::Gtm1 gm1 = gtm1::Gtm1::parse(g);
                    scene::Model m = gtm1::build_model(gm1, o, "pylon" + std::to_string(k));
                    if (!m.meshes.empty()) t.model = std::make_unique<render::GpuModel>(*ctx, m);
                    o.node = 1; // the planar shadow
                    o.alpha_test = false;
                    t.shadow_src = gtm1::build_model(gm1, o, "pylon_shadow" + std::to_string(k));
                    for (scene::Mesh& sm : t.shadow_src.meshes)
                        for (scene::Primitive& p : sm.primitives) {
                            p.material = {};
                            p.flags = scene::kAlphaBlend | scene::kUnlit | scene::kNoDepthWrite | scene::kDoubleSided;
                        }
                }
            }
            types_.push_back(std::move(t));
        }
        u32 count = rd32(road, 0x50), recs = rd32(road, 0x54);
        for (u32 i = 0; i < count && records_.size() < kPool; ++i) {
            size_t o = size_t(recs) + 32 * size_t(i);
            if (o + 32 > road.size()) break;
            Record r;
            r.type = int(rd32(road, o) & 0xFFFF);
            r.pos = {rdf(road, o + 8), rdf(road, o + 12), rdf(road, o + 16)};
            for (int a = 0; a < 3; ++a) r.angles[a] = rdf(road, o + 20 + 4 * size_t(a));
            if (r.type < int(types_.size())) records_.push_back(r);
        }
    } catch (const std::exception& e) {
        LOG_WARN("Pylons: %s", e.what());
        return false;
    }
    if (!records_.empty()) LOG_INFO("Pylons: %zu bodies, %zu types", records_.size(), types_.size());
    return !records_.empty();
}

void Pylons::reset(const sim::TrackQuery& ground) {
    bodies_.clear();
    for (const Record& r : records_) {
        Body b;
        b.type = r.type;
        b.pos = r.pos;
        const Type& t = types_[size_t(r.type)];
        // Euler matrix (a, b, c); the PS2 multiplies row vectors, so ours is its transpose.
        float sa = std::sin(r.angles[0]), ca = std::cos(r.angles[0]), sb = std::sin(r.angles[1]), cb = std::cos(r.angles[1]);
        float sc = std::sin(r.angles[2]), cc = std::cos(r.angles[2]);
        const float e[9] = {cb * cc - sa * sb * sc, -ca * sc, sb * cc + sa * sc * cb, cb * sc + sa * cc * sb, ca * cc,
                            sb * sc - sa * cc * cb, -ca * sb, sa, ca * cb};
        float rot[9] = {e[0], e[3], e[6], e[1], e[4], e[7], e[2], e[5], e[8]};
        // Tilted onto the ground under the centre; hull point 0 on the ground.
        Vec3 n{0, 1, 0};
        float d = -b.pos.y;
        bool on_ground = ground_plane(ground, b.pos, n, d);
        b.plane_n = n, b.plane_d = d;
        if (on_ground) {
            float tilt[9];
            Vec3 axis = Vec3{0, 1, 0}.cross(n);
            float s = axis.length(), c = n.y;
            axis_angle(s > 1e-6f ? axis * (std::atan2(s, c) / s) : Vec3{}, tilt);
            matmul(tilt, rot, rot);
            std::memcpy(b.rot, rot, sizeof rot);
            if (!t.hull.empty()) {
                Vec3 p0 = b.pos + mul(b.rot, t.hull[0]);
                float off = n.dot(p0) + d; // signed distance of point 0 from the plane
                if (std::fabs(n.y) > 1e-3f) b.pos.y -= off / n.y;
            }
        } else {
            std::memcpy(b.rot, rot, sizeof rot);
        }
        b.contact[0] = b.contact[1] = b.contact[2] = 2; // asleep
        bodies_.push_back(b);
    }
}

void Pylons::ground_contact(Body& b, const Type& t, const sim::TrackQuery& ground) {
    Vec3 n;
    float d;
    if (!ground_plane(ground, b.pos, n, d)) {
        b.active = false; // off the course mesh: removed for the rest of the race
        return;
    }
    b.plane_n = n, b.plane_d = d;
    float deepest = 0;
    int k = -1;
    for (size_t i = 0; i < t.hull.size() && i < 8; ++i) {
        Vec3 r = mul(b.rot, t.hull[i]);
        Vec3 p = b.pos + r;
        float h = std::fabs(n.y) > 1e-3f ? -(d + n.x * p.x + n.z * p.z) / n.y : p.y;
        float depth = h - p.y; // > 0: below the ground
        if (depth <= 0) {
            b.contact[i] = 0;
        } else if (depth > deepest) {
            deepest = depth, k = int(i);
            b.contact[i] = static_cast<unsigned char>(std::min(2, b.contact[i] + 1));
        }
    }
    if (k < 0) return;
    b.pos.y += deepest;
    impulse(b, t, mul(b.rot, t.hull[size_t(k)]), n, {}, 0.25f, 1.0f, 0.5f);
}

namespace {

// The wall edges within `reach` of `c` in x / z and in the height band [-5 m, +8 m] around it (bridges); an edge starting
// at a kWallSensor point has no wall.
struct WallEdge {
    double ax, az, bx, bz;
};
void wall_edges(std::span<const track::WallLine> walls, const Vec3& c, float reach, std::vector<WallEdge>& out) {
    for (const track::WallLine& line : walls) {
        const size_t n = line.points.size();
        const size_t edges = line.closed ? n : (n ? n - 1 : 0);
        for (size_t i = 0; i < edges; ++i) {
            const auto& pa = line.points[i];
            const auto& pb = line.points[(i + 1) % n];
            if (pa.flags & track::kWallSensor) continue;
            const Vec3 &p = pa.pos, &q = pb.pos;
            if (c.y < std::min(p.y, q.y) - 8.0f || c.y > std::max(p.y, q.y) + 5.0f) continue;
            if (c.x < std::min(p.x, q.x) - reach || c.x > std::max(p.x, q.x) + reach) continue;
            if (c.z < std::min(p.z, q.z) - reach || c.z > std::max(p.z, q.z) + reach) continue;
            out.push_back({p.x, p.z, q.x, q.z});
        }
    }
}

} // namespace

void Pylons::wall_contact(Body& b, const Type& t, const Vec3& old_pos, std::span<const track::WallLine> walls) {
    // Wall edges near the motion (x/z span old -> new, grown by 8 m).
    Vec3 mid = (old_pos + b.pos) * 0.5f;
    float reach = 0.5f * std::hypot(b.pos.x - old_pos.x, b.pos.z - old_pos.z) + 8.0f;
    std::vector<WallEdge> segs;
    wall_edges(walls, mid, reach, segs);
    for (const WallEdge& sg : segs) {
        float sx = float(sg.bx - sg.ax), sz = float(sg.bz - sg.az);
        float len2 = sx * sx + sz * sz;
        if (len2 < 1e-8f) continue;
        float len = std::sqrt(len2);
        Vec3 n{sz / len, 0, -sx / len}; // the drivable side (the track lies left of each edge)
        auto side = [&](const Vec3& p) { return sx * (p.z - float(sg.az)) - sz * (p.x - float(sg.ax)); }; // > 0: behind
        float dx = b.pos.x - float(sg.ax), dz = b.pos.z - float(sg.az);
        float u = (dx * sx + dz * sz) / len2;
        float c_new = side(b.pos), c_old = side(old_pos);
        bool moved = false;
        if (u > 0 && u < 1 && std::fabs(c_new) / len < t.radius) {
            // Overlapping: the centre goes to the radius on the drivable side.
            float fx = float(sg.ax) + u * sx, fz = float(sg.az) + u * sz;
            b.pos.x = fx + t.radius * n.x, b.pos.z = fz + t.radius * n.z;
            moved = true;
        } else if (c_new >= 0 && c_old <= 0 && c_new - c_old > 1e-9f) {
            // Crossed this frame within the edge: back along the motion onto the wall line.
            float f = c_new / (c_new - c_old);
            Vec3 p = b.pos + (old_pos - b.pos) * f;
            float pu = ((p.x - float(sg.ax)) * sx + (p.z - float(sg.az)) * sz) / len2;
            if (pu >= 0 && pu <= 1) b.pos = p, moved = true;
        }
        if (!moved) continue;
        // The hull point deepest towards the wall, the ground contact's impulse.
        float best = 3.4e38f;
        Vec3 r{};
        for (size_t i = 0; i < t.hull.size(); ++i) {
            Vec3 ri = mul(b.rot, t.hull[i]);
            float k = n.dot(b.pos + ri);
            if (k < best) best = k, r = ri;
        }
        impulse(b, t, r, n, {}, 0.25f, 1.0f, 0.5f);
    }
}

unsigned Pylons::step(const sim::TrackQuery& ground, const std::vector<Car>& cars, double dt_d,
                      std::span<const track::WallLine> walls) {
    const float dt = float(dt_d);
    unsigned hits = 0;
    for (Body& b : bodies_) {
        if (!b.active) continue;
        const Type& t = types_[size_t(b.type)];
        // Car test (asleep or not): the first car whose box holds a hull point.
        for (size_t c = 0; c < cars.size() && c < 32; ++c) {
            const Car& car = cars[c];
            Vec3 dc = b.pos - car.pos;
            if (dc.dot(dc) > (t.radius + car.front + car.rear + car.half_width + 2) * (t.radius + car.front + car.rear + car.half_width + 2))
                continue;
            const sim::Quat& q = car.orientation;
            auto rot = [&](const Vec3& v) { // car frame -> world
                Vec3 u{q.x, q.y, q.z};
                Vec3 tt = u.cross(v) * 2.0f;
                return v + tt * q.w + u.cross(tt);
            };
            Vec3 fwd = rot({0, 0, -1});
            fwd.y = 0;
            float fl = fwd.length();
            if (fl < 1e-6f) continue;
            fwd = fwd * (1.0f / fl);
            Vec3 right{-fwd.z, 0, fwd.x};
            // Five faces in (lateral, up, forward) car coordinates: outward normal, offset.
            struct Face { float nx, ny, nu, d; };
            const float s = 1.0f / std::sqrt(1.0f + 0.05f * 0.05f);
            const Face faces[5] = {{0, 0.05f * s, s, -(car.front + 0.05f) * s},   // front: u <= a - 0.05 (y - 1)
                                   {0, 0.05f * s, -s, -(car.rear + 0.05f) * s},   // rear
                                   {s, 0.05f * s, 0, -(car.half_width + 0.05f) * s}, // right side
                                   {-s, 0.05f * s, 0, -(car.half_width + 0.05f) * s},
                                   {0, 1, 0, -1}};                                // top, 1 m up
            int hit_face = -1;
            float hit_depth = 0;
            Vec3 hit_r;
            for (size_t i = 0; i < t.hull.size(); ++i) {
                Vec3 r = mul(b.rot, t.hull[i]);
                Vec3 p = b.pos + r - car.pos;
                float x = p.dot(right), y = p.y, u = p.dot(fwd);
                float best = 1e9f;
                int bf = -1;
                bool inside = true;
                for (int f = 0; f < 5 && inside; ++f) {
                    float sd = faces[f].nx * x + faces[f].ny * y + faces[f].nu * u + faces[f].d;
                    if (sd > 0) inside = false;
                    else if (-sd < best) best = -sd, bf = f;
                }
                if (inside) {
                    hit_face = bf, hit_depth = best, hit_r = r;
                    break;
                }
            }
            if (hit_face < 0) continue;
            hits |= 1u << c;
            const Face& f = faces[hit_face];
            Vec3 n = right * f.nx + Vec3{0, f.ny, 0} + fwd * f.nu;
            b.pos += n * hit_depth; // out through the nearest face
            Vec3 lever = b.pos + hit_r - car.pos;
            Vec3 car_v = car.vel + Vec3{0, car.yaw_rate, 0}.cross(Vec3{lever.x, 0, lever.z});
            impulse(b, t, hit_r, n, car_v, 0.01f, 1.5f, 0);
            std::memset(b.contact, 0, sizeof b.contact); // wake
            break;
        }
        int resting = 0;
        for (size_t i = 0; i < t.hull.size(); ++i) resting += b.contact[i] == 2;
        if (resting >= 3) continue; // asleep
        // Torque-free rotation (Euler's equations, body frame), damped and clamped.
        Vec3 w = b.omega, I = t.inertia;
        const int sub = 4;
        for (int s = 0; s < sub; ++s) {
            float h = dt / sub;
            w.x += h * (I.y - I.z) * w.y * w.z / I.x;
            w.y += h * (I.z - I.x) * w.z * w.x / I.y;
            w.z += h * (I.x - I.y) * w.x * w.y / I.z;
        }
        float dr[9];
        axis_angle(w * dt, dr);
        matmul(b.rot, dr, b.rot); // body-frame increment
        orthonormalize(b.rot);
        w = w * 0.999f;
        w = {std::clamp(w.x, -20.0f, 20.0f), std::clamp(w.y, -20.0f, 20.0f), std::clamp(w.z, -20.0f, 20.0f)};
        b.omega = w;
        Vec3 old_pos = b.pos;
        b.pos += b.vel * dt;
        b.vel.y -= 9.81f * dt;
        ground_contact(b, t, ground);
        if (!walls.empty() && b.active) wall_contact(b, t, old_pos, walls);
    }
    return hits;
}

void Pylons::set_shadow(const std::array<float, 3>& rgb, float alpha) {
    if (!ctx_) return;
    auto byte = [](float x) { return u32(std::clamp(x, 0.0f, 1.0f) * 255.0f + 0.5f); };
    for (Type& t : types_) {
        t.shadow.reset();
        if (t.shadow_src.meshes.empty()) continue;
        scene::Model m = t.shadow_src;
        for (scene::Mesh& mesh : m.meshes)
            for (scene::Vertex& v : mesh.vertices) {
                float a = std::min(1.0f, float(v.color >> 24) / 255.0f * alpha);
                v.color = byte(rgb[0]) | byte(rgb[1]) << 8 | byte(rgb[2]) << 16 | byte(a) << 24;
            }
        t.shadow = std::make_unique<render::GpuModel>(*ctx_, m);
    }
}

bool Pylons::shadow_matrix(const Body& b, const render::Vec3& light, render::Mat4& out) {
    // p' = p - L (n.p + d) / (n.L): every point slid along L onto the plane; a 1e-3 share of the
    // plane distance kept so the matrix stays invertible (normal matrix).
    const Vec3& n = b.plane_n;
    float nl = n.x * light.x + n.y * light.y + n.z * light.z;
    if (std::fabs(nl) < 0.05f) return false;
    const float L[3] = {light.x, light.y, light.z}, P[4] = {n.x, n.y, n.z, b.plane_d};
    const float N[3] = {n.x, n.y, n.z};
    render::Mat4 proj;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            proj.at(r, c) = (r == c ? 1.0f : 0.0f) - L[r] * P[c] / nl + 1e-3f * N[r] * P[c];
    render::Mat4 body;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) body.at(r, c) = b.rot[r * 3 + c];
    body.at(0, 3) = b.pos.x, body.at(1, 3) = b.pos.y, body.at(2, 3) = b.pos.z;
    out = proj * body;
    return true;
}

void Pylons::add_items(std::vector<render::DrawItem>& items, const render::Vec3& eye, const render::Vec3& light) const {
    // GT3 moves the shadow's near / far planes by 0.005: about 5 mm nearer the camera.
    for (const Body& b : bodies_) {
        if (!b.active) continue;
        const Type& t = types_[size_t(b.type)];
        render::Mat4 sm;
        if (!t.shadow || !shadow_matrix(b, light, sm)) continue;
        render::Vec3 c{b.pos.x, b.pos.y, b.pos.z};
        float dist = std::max(render::length(c - eye), 0.1f);
        float k = 1.0f - 0.005f / dist;
        render::Mat4 nudge = render::translate(eye) * render::scale({k, k, k}) * render::translate(render::Vec3{0, 0, 0} - eye);
        items.push_back({t.shadow.get(), nudge * sm, 0});
    }
    for (const Body& b : bodies_) {
        if (!b.active) continue;
        const Type& t = types_[size_t(b.type)];
        if (!t.model) continue;
        render::Mat4 m;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) m.at(r, c) = b.rot[r * 3 + c];
        m.at(0, 3) = b.pos.x, m.at(1, 3) = b.pos.y, m.at(2, 3) = b.pos.z;
        items.push_back({t.model.get(), m, 0});
    }
}

} // namespace gt3r::race
