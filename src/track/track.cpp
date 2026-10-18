#include "track/track.h"

#include "formats/course/course_file.h"
#include "vfs/vfs.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <numbers>

namespace gt3r::track {

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kCell = 8.0f; // own grid cell size (the game's grids use 8 m too)

// Raw SurfaceHit::surface packing: material (triangle +0x08) in bits 0-7,
// flags (triangle +0x0C) in bits 8-15.
u32 pack_surface(u16 material, u16 flags) { return u32(material & 0xFF) | u32(flags & 0xFF) << 8; }

sim::Vec3 to_sim(const course::Vec3& v) { return {v.x, v.y, v.z}; }

// Flattened triangle with precomputed barycentric and plane coefficients:
//   l1 = a1 x + b1 z + c1,  l2 = a2 x + b2 z + c2,  l0 = 1 - l1 - l2
//   y  = hx x + hz z + hc
struct FlatTri {
    float a1, b1, c1, a2, b2, c2;
    float hx, hz, hc;
    sim::Vec3 normal;
    u32 surface;
    s32 segment;
    float min_x, min_z, max_x, max_z;
};

struct WallEdge {
    float x0, z0, x1, z1;
    float ymin, ymax;
    float nx, nz;       // unit normal pointing to the drivable side
    bool sensor;        // start point flagged as sensor: no contact
    int wall, edge;
};

// A uniform 2-D grid of index lists (CSR).
struct UniformGrid {
    float x0 = 0, z0 = 0, inv = 1.0f / kCell, cell = kCell;
    int nx = 0, nz = 0;
    std::vector<u32> begin; // nx*nz + 1
    std::vector<u32> items;

    bool empty() const { return nx == 0 || nz == 0; }
    // Returns -1 when outside.
    int cell_of(float x, float z) const {
        float fx = (x - x0) * inv, fz = (z - z0) * inv;
        if (!(fx >= 0 && fz >= 0)) return -1;
        int ix = int(fx), iz = int(fz);
        if (ix >= nx || iz >= nz) return -1;
        return iz * nx + ix;
    }
    void clamp_range(float lo_x, float lo_z, float hi_x, float hi_z, int& ix0, int& iz0, int& ix1, int& iz1) const {
        ix0 = std::max(0, int(std::floor((lo_x - x0) * inv)));
        iz0 = std::max(0, int(std::floor((lo_z - z0) * inv)));
        ix1 = std::min(nx - 1, int(std::floor((hi_x - x0) * inv)));
        iz1 = std::min(nz - 1, int(std::floor((hi_z - z0) * inv)));
    }

    // Builds from axis-aligned boxes (one per item).
    template <typename BoxFn>
    void build(size_t count, float bx0, float bz0, float bx1, float bz1, BoxFn box) {
        x0 = bx0 - 1, z0 = bz0 - 1;
        nx = std::max(1, int(std::ceil((bx1 + 1 - x0) * inv)));
        nz = std::max(1, int(std::ceil((bz1 + 1 - z0) * inv)));
        std::vector<u32> counts(size_t(nx) * nz + 1, 0);
        auto each = [&](auto&& fn) {
            for (size_t i = 0; i < count; ++i) {
                float lx, lz, hx, hz;
                box(i, lx, lz, hx, hz);
                int ix0, iz0, ix1, iz1;
                clamp_range(lx, lz, hx, hz, ix0, iz0, ix1, iz1);
                for (int iz = iz0; iz <= iz1; ++iz)
                    for (int ix = ix0; ix <= ix1; ++ix) fn(size_t(iz) * nx + ix, u32(i));
            }
        };
        each([&](size_t c, u32) { ++counts[c]; });
        begin.assign(counts.size(), 0);
        for (size_t c = 0; c + 1 < counts.size(); ++c) begin[c + 1] = begin[c] + counts[c];
        items.assign(begin.back(), 0);
        std::vector<u32> fill(begin.begin(), begin.end() - 1);
        each([&](size_t c, u32 i) { items[fill[c]++] = i; });
    }
};

float wrap(float s, float len) {
    if (len <= 0) return s;
    s = std::fmod(s, len);
    return s < 0 ? s + len : s;
}

sim::Quat yaw_quat(float yaw) { return {std::cos(yaw * 0.5f), 0, std::sin(yaw * 0.5f), 0}; }

// Yaw (about +Y) that turns the car's nose (-Z) towards horizontal `f`.
float yaw_of(float fx, float fz) { return std::atan2(-fx, -fz); }

} // namespace

const char* surface_name(Surface s) {
    switch (s) {
    case Surface::Tarmac: return "tarmac";
    case Surface::Kerb: return "kerb";
    case Surface::Grass: return "grass";
    case Surface::Gravel: return "gravel";
    case Surface::Sand: return "sand";
    case Surface::Dirt: return "dirt";
    case Surface::Paving: return "paving";
    case Surface::Loose: return "loose";
    case Surface::Rough: return "rough";
    case Surface::OffRoad: return "off-road";
    case Surface::Unknown: break;
    }
    return "unknown";
}

Surface surface_type(u32 raw) {
    switch (surface_material(raw)) {
    case 0: return Surface::Tarmac;
    case 1: return Surface::Kerb;
    case 2: return Surface::Grass;
    case 3: return Surface::Gravel;
    case 4: return Surface::Sand;
    case 5: return Surface::Dirt;
    case 7: return Surface::Paving;
    case 8: return Surface::Loose;
    case 9: return Surface::Rough;
    default: return Surface::Unknown;
    }
}

int surface_physics_index(u32 raw) {
    int m = surface_material(raw);
    if (!(surface_flags(raw) & kSurfaceWet)) return m;
    switch (m) {
    case 0:
    case 7: return 12;
    case 1: return 13;
    case 9: return 14;
    default: return m;
    }
}

// --------------------------------------------------------------------------

struct Track::Impl {
    std::vector<FlatTri> tris;
    std::vector<u32> tri_base; // first flat index of each segment

    // Triangle index: the game's grid (non-uniform breakpoints allowed) or ours.
    bool game_grid = false;
    std::vector<float> gxs, gzs;
    bool g_uniform = false;
    float g_x0 = 0, g_z0 = 0, g_inv = 0;
    std::vector<u32> g_begin, g_items;
    UniformGrid own;

    // Vertices for the off-road ground estimate.
    std::vector<sim::Vec3> verts;
    UniformGrid vgrid;

    // Walls.
    std::vector<WallEdge> edges;
    UniformGrid egrid;

    // Centre line ring (from sorted checkpoints).
    std::vector<sim::Vec3> cpts;
    std::vector<float> cs; // track distance at each point, ascending
    // Checkpoint gates: travel direction (= -stored dir) and half width.
    std::vector<sim::Vec3> gdir;
    std::vector<float> ghalf;
    std::vector<float> gwidth; // racing-surface width at each gate (0 = unknown)

    int game_cell(float x, float z) const {
        int ix, iz;
        if (g_uniform) {
            float fx = (x - g_x0) * g_inv, fz = (z - g_z0) * g_inv;
            if (!(fx >= 0 && fz >= 0)) return -1;
            ix = int(fx), iz = int(fz);
        } else {
            auto itx = std::upper_bound(gxs.begin(), gxs.end(), x);
            auto itz = std::upper_bound(gzs.begin(), gzs.end(), z);
            if (itx == gxs.begin() || itz == gzs.begin()) return -1;
            ix = int(itx - gxs.begin()) - 1, iz = int(itz - gzs.begin()) - 1;
        }
        int ncx = int(gxs.size()) - 1, ncz = int(gzs.size()) - 1;
        if (ix >= ncx || iz >= ncz) return -1;
        return iz * ncx + ix;
    }

    // Candidate triangle list for (x, z).
    std::pair<const u32*, const u32*> candidates(float x, float z) const {
        if (game_grid) {
            int c = game_cell(x, z);
            if (c < 0) return {nullptr, nullptr};
            return {g_items.data() + g_begin[c], g_items.data() + g_begin[c + 1]};
        }
        int c = own.cell_of(x, z);
        if (c < 0) return {nullptr, nullptr};
        return {own.items.data() + own.begin[c], own.items.data() + own.begin[c + 1]};
    }

    static bool inside(const FlatTri& t, float x, float z) {
        // Shared edges: the barycentric coefficients are precomputed in absolute coordinates, so at
        // a few hundred metres from the origin rounding opens hairline cracks between neighbours
        // (a probe at the exact float position misses, 1 cm away hits). A slightly wider
        // tolerance (about 1-2 mm on a 10-20 m triangle) closes them; overlaps pick the highest.
        constexpr float eps = -1e-4f;
        float l1 = t.a1 * x + t.b1 * z + t.c1;
        float l2 = t.a2 * x + t.b2 * z + t.c2;
        return l1 >= eps && l2 >= eps && 1.0f - l1 - l2 >= eps;
    }

    int probe_index(float x, float z, float y_max, float& y_out) const {
        auto [b, e] = candidates(x, z);
        int best = -1;
        float best_y = -kInf;
        for (const u32* p = b; p != e; ++p) {
            const FlatTri& t = tris[*p];
            if (!inside(t, x, z)) continue;
            float y = t.hx * x + t.hz * z + t.hc;
            if (y > y_max || y <= best_y) continue;
            best_y = y;
            best = int(*p);
        }
        y_out = best_y;
        return best;
    }
};

Track::~Track() = default;

std::unique_ptr<Track> Track::load(const Vfs& vfs, std::string_view label, std::string_view region) {
    std::string path = "courses/" + std::string(region) + "/" + std::string(label);
    std::optional<Bytes> data = vfs.read(path);
    if (!data) throw FormatError("track: missing course file " + path);
    return from_course_file(*data, std::string(label));
}

std::unique_ptr<Track> Track::from_course_file(ByteSpan data, std::string label) {
    course::CourseFile file = course::CourseFile::parse(data);
    ByteSpan road = file.section(data, course::kSlotRoad);
    if (road.empty()) throw FormatError("track: course has no GTRW road");
    course::AiLines ai;
    ByteSpan lines = file.section(data, course::kSlotAiLines);
    if (!lines.empty()) ai = course::AiLines::parse(lines);
    return std::make_unique<Track>(course::Road::parse(road), std::move(ai), std::move(label));
}

Track::Track(course::Road road, course::AiLines ai, std::string label)
    : label_(std::move(label)), road_(std::move(road)), ai_(std::move(ai)), impl_(std::make_unique<Impl>()) {
    Impl& m = *impl_;
    grid_ = road_.grid;

    // --- flatten triangles ---------------------------------------------------
    float bx0 = kInf, bz0 = kInf, bx1 = -kInf, bz1 = -kInf;
    for (size_t si = 0; si < road_.segments.size(); ++si) {
        const course::RoadSegment& seg = road_.segments[si];
        m.tri_base.push_back(u32(m.tris.size()));
        for (const course::Vec3& v : seg.vertices) {
            m.verts.push_back(to_sim(v));
            bx0 = std::min(bx0, v.x), bz0 = std::min(bz0, v.z);
            bx1 = std::max(bx1, v.x), bz1 = std::max(bz1, v.z);
        }
        for (const course::RoadTriangle& rt : seg.triangles) {
            const course::Vec3 &a = seg.vertices[rt.a], &b = seg.vertices[rt.b], &c = seg.vertices[rt.c];
            FlatTri t{};
            t.surface = pack_surface(rt.material, rt.flags);
            t.segment = s32(si);
            t.min_x = std::min({a.x, b.x, c.x}), t.max_x = std::max({a.x, b.x, c.x});
            t.min_z = std::min({a.z, b.z, c.z}), t.max_z = std::max({a.z, b.z, c.z});
            // Barycentric weights of a (l0), b (l1), c (l2) as affine functions of (x, z).
            double d = double(b.z - c.z) * (a.x - c.x) + double(c.x - b.x) * (a.z - c.z);
            if (std::fabs(d) < 1e-9) {
                // Degenerate in XZ (vertical): never contains a point.
                t.a1 = t.b1 = t.a2 = t.b2 = 0;
                t.c1 = t.c2 = -1;
            } else {
                // l0 = ((b.z-c.z)(x-c.x) + (c.x-b.x)(z-c.z)) / d
                // l1 = ((c.z-a.z)(x-c.x) + (a.x-c.x)(z-c.z)) / d
                double l0x = (b.z - c.z) / d, l0z = (c.x - b.x) / d, l0c = -(l0x * c.x + l0z * c.z);
                double l1x = (c.z - a.z) / d, l1z = (a.x - c.x) / d, l1c = -(l1x * c.x + l1z * c.z);
                // Store l1 := weight of a (l0 above) and l2 := weight of b; the third is c.
                t.a1 = float(l0x), t.b1 = float(l0z), t.c1 = float(l0c);
                t.a2 = float(l1x), t.b2 = float(l1z), t.c2 = float(l1c);
                // y = a.y l0 + b.y l1 + c.y (1 - l0 - l1)
                t.hx = float((a.y - c.y) * l0x + (b.y - c.y) * l1x);
                t.hz = float((a.y - c.y) * l0z + (b.y - c.y) * l1z);
                t.hc = float(c.y + (a.y - c.y) * l0c + (b.y - c.y) * l1c);
            }
            sim::Vec3 e1{b.x - a.x, b.y - a.y, b.z - a.z}, e2{c.x - a.x, c.y - a.y, c.z - a.z};
            sim::Vec3 n = e1.cross(e2);
            if (n.y < 0) n = n * -1.0f;
            float nl = n.length();
            t.normal = nl > 0 ? n * (1.0f / nl) : sim::Vec3{0, 1, 0};
            m.tris.push_back(t);
        }
    }

    // --- triangle index ------------------------------------------------------
    const course::RoadGrid& gg = road_.grid_index;
    if (!gg.empty() && gg.cell_begin.size() == (gg.xs.size() - 1) * (gg.zs.size() - 1) + 1) {
        m.gxs = gg.xs, m.gzs = gg.zs;
        m.g_begin = gg.cell_begin;
        m.g_items.reserve(gg.refs.size());
        for (const course::RoadGridRef& r : gg.refs) m.g_items.push_back(m.tri_base[r.segment] + r.triangle);
        float step = m.gxs[1] - m.gxs[0];
        bool uniform = step > 0;
        for (size_t i = 1; uniform && i < m.gxs.size(); ++i)
            uniform = std::fabs(m.gxs[i] - m.gxs[0] - step * float(i)) < 1e-3f;
        for (size_t i = 1; uniform && i < m.gzs.size(); ++i)
            uniform = std::fabs(m.gzs[i] - m.gzs[0] - step * float(i)) < 1e-3f;
        m.g_uniform = uniform;
        if (uniform) m.g_x0 = m.gxs[0], m.g_z0 = m.gzs[0], m.g_inv = 1.0f / step;
        // Validate: every triangle must be listed in the cells under its
        // centroid and under points just inside each corner.
        m.game_grid = true;
        for (size_t ti = 0; ti < m.tris.size() && m.game_grid; ++ti) {
            const FlatTri& t = m.tris[ti];
            if (t.c1 == -1 && t.a1 == 0) continue; // degenerate
            const course::RoadSegment& seg = road_.segments[size_t(t.segment)];
            const course::RoadTriangle& rt = seg.triangles[ti - m.tri_base[size_t(t.segment)]];
            const course::Vec3* v[3] = {&seg.vertices[rt.a], &seg.vertices[rt.b], &seg.vertices[rt.c]};
            float cx = (v[0]->x + v[1]->x + v[2]->x) / 3, cz = (v[0]->z + v[1]->z + v[2]->z) / 3;
            for (int k = -1; k < 3 && m.game_grid; ++k) {
                float px = k < 0 ? cx : v[k]->x + (cx - v[k]->x) * 0.02f;
                float pz = k < 0 ? cz : v[k]->z + (cz - v[k]->z) * 0.02f;
                auto [b, e] = m.candidates(px, pz);
                // (r_route11: one triangle corner lies outside the grid's
                // breakpoint range, so the game could not find it there.)
                if (std::find(b, e, u32(ti)) == e) m.game_grid = false;
            }
        }
    }
    if (!m.game_grid && !m.tris.empty()) {
        m.own.build(m.tris.size(), bx0, bz0, bx1, bz1, [&](size_t i, float& lx, float& lz, float& hx, float& hz) {
            const FlatTri& t = m.tris[i];
            lx = t.min_x, lz = t.min_z, hx = t.max_x, hz = t.max_z;
        });
    }
    if (!m.verts.empty())
        m.vgrid.build(m.verts.size(), bx0, bz0, bx1, bz1, [&](size_t i, float& lx, float& lz, float& hx, float& hz) {
            lx = hx = m.verts[i].x, lz = hz = m.verts[i].z;
        });

    // --- centre line -----------------------------------------------------------
    cps_ = road_.checkpoints;
    std::stable_sort(cps_.begin(), cps_.end(), [](const auto& a, const auto& b) { return a.distance < b.distance; });
    length_ = road_.length;
    float prev_y = kInf;
    for (const course::Checkpoint& c : cps_) {
        sim::Vec3 p{(c.x0 + c.x1) * 0.5f, 0, (c.z0 + c.z1) * 0.5f};
        // Follow the road continuously where it passes over itself.
        float y = 0.0f;
        int ti = prev_y < kInf ? m.probe_index(p.x, p.z, prev_y + 6.0f, y) : -1;
        if (ti < 0) ti = m.probe_index(p.x, p.z, kInf, y);
        p.y = ti >= 0 ? y : (prev_y < kInf ? prev_y : 0.0f);
        prev_y = p.y;
        m.cpts.push_back(p);
        m.cs.push_back(c.distance);
        // The stored direction points against travel (checked on all courses).
        float dl = std::hypot(c.dir_x, c.dir_z);
        m.gdir.push_back(dl > 1e-6f ? sim::Vec3{-c.dir_x / dl, 0, -c.dir_z / dl} : sim::Vec3{0, 0, 0});
        m.ghalf.push_back(0.5f * std::hypot(c.x1 - c.x0, c.z1 - c.z0));
    }
    // Gates without a usable direction take the centre-line direction.
    for (size_t i = 0; i < m.cpts.size(); ++i)
        if (m.gdir[i].x == 0 && m.gdir[i].z == 0 && m.cpts.size() > 1) {
            const sim::Vec3 &a = m.cpts[i], &b = m.cpts[(i + 1) % m.cpts.size()];
            float l = std::max(1e-6f, std::hypot(b.x - a.x, b.z - a.z));
            m.gdir[i] = {(b.x - a.x) / l, 0, (b.z - a.z) / l};
        }
    if (length_ <= 0 && !m.cs.empty()) length_ = m.cs.back() + 15.0f;

    // The gate lines span the whole drivable width including run-off and the
    // pit lane, so their midpoints are not the middle of the road. Move each
    // gate's centre along its line to the middle of the racing surface
    // (tarmac, kerb, dirt, paving...; not grass, gravel, sand or pit lane)
    // around it. Moving along the line keeps the gate itself unchanged.
    float ref_y = kInf; // height of the previous gate's centre (follows bridges)
    for (size_t i = 0; i < m.cpts.size(); ++i) {
        const sim::Vec3 c = m.cpts[i], t = m.gdir[i];
        float lx = t.z, lz = -t.x, half = m.ghalf[i];
        constexpr float kStep = 0.5f;
        int nsteps = int(half / kStep);
        float best_lo = 0, best_hi = 0, best_y = 0, best_score = kInf;
        // Pass 0 stays within 4 m of the previous centre's height; pass 1 (or
        // the first gate) takes the highest surface.
        for (int pass = ref_y < kInf ? 0 : 1; pass < 2 && best_score == kInf; ++pass) {
            float ymax = pass == 0 ? ref_y + 4.0f : kInf;
            float run_lo = 0, run_y = 0;
            bool in_run = false;
            for (int k = -nsteps; k <= nsteps + 1; ++k) {
                float d = float(k) * kStep, y = 0;
                bool ok = false;
                if (k <= nsteps) {
                    int ti = m.probe_index(c.x + lx * d, c.z + lz * d, ymax, y);
                    if (ti >= 0 && (pass == 1 || std::fabs(y - ref_y) < 4.0f)) {
                        u32 raw = m.tris[size_t(ti)].surface;
                        Surface st = surface_type(raw);
                        ok = !(surface_flags(raw) & kSurfacePitLane) && st != Surface::Grass &&
                             st != Surface::Gravel && st != Surface::Sand && st != Surface::Unknown;
                    }
                }
                if (ok && !in_run) run_lo = d, run_y = y, in_run = true;
                if (!ok && in_run) {
                    in_run = false;
                    float hi = d - kStep;
                    // Prefer the run containing the gate midpoint, else the nearest, then the widest.
                    float dist = (run_lo <= 0 && hi >= 0) ? 0.f : std::min(std::fabs(run_lo), std::fabs(hi));
                    float score = dist * 4 - (hi - run_lo);
                    if (hi - run_lo >= 3.0f && score < best_score)
                        best_score = score, best_lo = run_lo, best_hi = hi, best_y = run_y;
                }
            }
        }
        if (best_score < kInf) {
            float dc = 0.5f * (best_lo + best_hi), y;
            float nx = c.x + lx * dc, nz = c.z + lz * dc;
            int ti = m.probe_index(nx, nz, best_y + 4.0f, y);
            m.cpts[i] = {nx, ti >= 0 ? y : best_y, nz};
            m.gwidth.push_back(best_hi - best_lo);
        } else {
            m.gwidth.push_back(0);
        }
        ref_y = m.cpts[i].y;
    }

    // --- walls -----------------------------------------------------------------
    // Polylines from the raw wall points: the first point's tag holds the
    // count, the last one -count; the game treats the last point as linking
    // back to the first (closed loop). Upper tag half = point flags.
    for (size_t i = 0; i < road_.wall_points.size();) {
        u32 n = road_.wall_points[i].tag & 0xFFFF;
        if (n == 0 || i + n > road_.wall_points.size()) n = u32(road_.wall_points.size() - i);
        WallLine line;
        for (size_t k = i; k < i + n; ++k)
            line.points.push_back({to_sim(road_.wall_points[k].pos), u16(road_.wall_points[k].tag >> 16)});
        line.closed = n >= 3 && (road_.wall_points[i + n - 1].tag & 0xFFFF) == ((0x10000 - n) & 0xFFFF);
        wall_lines_.push_back(std::move(line));
        i += n;
    }
    if (road_.wall_points.empty()) // a Road built in code with polylines only
        for (const auto& poly : road_.walls) {
            WallLine line;
            for (const course::Vec3& v : poly) line.points.push_back({to_sim(v), 0});
            wall_lines_.push_back(std::move(line));
        }
    for (size_t w = 0; w < wall_lines_.size(); ++w) {
        const WallLine& line = wall_lines_[w];
        size_t np = line.points.size();
        size_t nedges = line.closed ? np : (np ? np - 1 : 0);
        for (size_t i = 0; i < nedges; ++i) {
            const WallPoint &pa = line.points[i], &pb = line.points[(i + 1) % np];
            const sim::Vec3 &a = pa.pos, &b = pb.pos;
            float dx = b.x - a.x, dz = b.z - a.z, l = std::hypot(dx, dz);
            if (l < 1e-4f) continue;
            WallEdge e{a.x, a.z, b.x, b.z, std::min(a.y, b.y), std::max(a.y, b.y), dz / l, -dx / l,
                       (pa.flags & kWallSensor) != 0, int(w), int(i)};
            // Like the game, every edge is one-sided with the drivable side on
            // the left of the polyline direction (left = up x direction). Over
            // the 80 US courses, probing 2 m to each side finds road only on
            // the left for 102,473 edges, only on the right for 45, on both
            // sides for 51,868 (mesh continues behind the wall) and on
            // neither for 276.
            m.edges.push_back(e);
        }
    }
    if (!m.edges.empty()) {
        float ex0 = kInf, ez0 = kInf, ex1 = -kInf, ez1 = -kInf;
        for (const WallEdge& e : m.edges) {
            ex0 = std::min({ex0, e.x0, e.x1}), ez0 = std::min({ez0, e.z0, e.z1});
            ex1 = std::max({ex1, e.x0, e.x1}), ez1 = std::max({ez1, e.z0, e.z1});
        }
        m.egrid.build(m.edges.size(), ex0, ez0, ex1, ez1, [&](size_t i, float& lx, float& lz, float& hx, float& hz) {
            const WallEdge& e = m.edges[i];
            lx = std::min(e.x0, e.x1), hx = std::max(e.x0, e.x1);
            lz = std::min(e.z0, e.z1), hz = std::max(e.z0, e.z1);
        });
    }
}

const char* Track::index_source() const { return impl_->game_grid ? "game" : "built"; }

// --- surface ------------------------------------------------------------------

sim::SurfaceHit Track::probe(float x, float z, float y_max) const {
    sim::SurfaceHit h;
    float y;
    int ti = impl_->probe_index(x, z, y_max, y);
    if (ti < 0) return h;
    const FlatTri& t = impl_->tris[size_t(ti)];
    h.hit = true;
    h.height = y;
    h.normal = t.normal;
    h.surface = t.surface;
    h.segment = t.segment;
    return h;
}

sim::SurfaceHit Track::probe_or_ground(float x, float z, float y_max) const {
    sim::SurfaceHit h = probe(x, z, y_max);
    if (h.hit) return h;
    const Impl& m = *impl_;
    h.normal = {0, 1, 0};
    h.surface = 0;
    h.segment = -1;
    // Nearest road vertex within ~24 m (preferring those below y_max), else
    // the centre line.
    float best = kInf, best_y = 0;
    if (!m.vgrid.empty()) {
        int ix0, iz0, ix1, iz1;
        m.vgrid.clamp_range(x - 24, z - 24, x + 24, z + 24, ix0, iz0, ix1, iz1);
        for (int iz = iz0; iz <= iz1; ++iz)
            for (int ix = ix0; ix <= ix1; ++ix) {
                size_t c = size_t(iz) * m.vgrid.nx + ix;
                for (u32 k = m.vgrid.begin[c]; k < m.vgrid.begin[c + 1]; ++k) {
                    const sim::Vec3& v = m.verts[m.vgrid.items[k]];
                    float d = (v.x - x) * (v.x - x) + (v.z - z) * (v.z - z);
                    if (v.y > y_max) d += 1e6f;
                    if (d < best) best = d, best_y = v.y;
                }
            }
    }
    if (best == kInf) best_y = locate({x, y_max, z}).centre.y;
    h.height = best_y;
    return h;
}

// --- grid / poses ---------------------------------------------------------------

Pose Track::grid_pose(int slot) const {
    Pose p;
    if (grid_.empty()) return respawn_pose(0);
    slot = std::clamp(slot, 0, int(grid_.size()) - 1);
    const course::GridSlot& g = grid_[size_t(slot)];
    sim::SurfaceHit h = probe(g.pos.x, g.pos.z, g.pos.y + 50.0f);
    if (!h.hit) h = probe_or_ground(g.pos.x, g.pos.z, kInf);
    p.position = {g.pos.x, h.height, g.pos.z};
    // Heading: GTRW yaw + pi (see TRACK.md); verified against the lap direction.
    float heading = g.yaw + kPi;
    p.forward = {-std::sin(heading), 0, -std::cos(heading)};
    p.yaw = heading;
    p.orientation = yaw_quat(heading);
    return p;
}

Pose Track::respawn_pose(float s) const {
    Pose p;
    sim::Vec3 c = point_at(s, 0);
    sim::Vec3 f = direction_at(s);
    sim::SurfaceHit h = probe(c.x, c.z, c.y + 3.0f);
    if (h.hit) c.y = h.height;
    p.position = c;
    p.forward = f;
    p.yaw = yaw_of(f.x, f.z);
    p.orientation = yaw_quat(p.yaw);
    return p;
}

// --- lap geometry ---------------------------------------------------------------

TrackPosition Track::locate(const sim::Vec3& p, float hint, float hint_window) const {
    const Impl& m = *impl_;
    TrackPosition out;
    size_t n = m.cpts.size();
    if (n < 2) return out;
    float best = kInf;
    bool use_hint = hint >= 0 && length_ > 0;
    auto outside_window = [&](float s0, float s1) {
        float mid = (s0 + s1) * 0.5f, half = (s1 - s0) * 0.5f;
        float d = std::fabs(wrap(mid - hint + length_ * 0.5f, length_) - length_ * 0.5f);
        return d > hint_window + half;
    };
    // 1. Gates: a point between checkpoint lines i and i+1 gets the distance
    //    interpolated by its signed distances to the two lines. This is
    //    continuous and monotonic along any forward path between the lines.
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            float s0 = m.cs[i], s1 = j ? m.cs[j] : m.cs[0] + length_;
            if (use_hint && outside_window(s0, s1)) continue;
            const sim::Vec3 &a = m.cpts[i], &b = m.cpts[j], &ta = m.gdir[i], &tb = m.gdir[j];
            float gi = (p.x - a.x) * ta.x + (p.z - a.z) * ta.z;
            if (gi < 0) continue;
            float gj = (p.x - b.x) * tb.x + (p.z - b.z) * tb.z;
            if (gj >= 0) continue;
            float f = gi / (gi - gj);
            sim::Vec3 c{a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.z + (b.z - a.z) * f};
            float dx = ta.x + (tb.x - ta.x) * f, dz = ta.z + (tb.z - ta.z) * f, l = std::hypot(dx, dz);
            if (l < 1e-6f) continue;
            // Lateral in the bilinear patch spanned by the two gate lines (the
            // inverse of point_at): p = c + lat * lerp(left_i, left_j, f), with
            // left = (t.z, -t.x). Unnormalised blend keeps gate lines exact.
            float lx = dz, lz = -dx; // = lerp of the gate left vectors
            float lat = ((p.x - c.x) * lx + (p.z - c.z) * lz) / (lx * lx + lz * lz);
            dx /= l, dz /= l;
            float limit = std::max(m.ghalf[i], m.ghalf[j]) + 10.0f;
            if (std::fabs(lat) > limit) continue;
            float dy = p.y - c.y;
            float score = lat * lat + dy * dy * 0.25f;
            if (score < best) {
                best = score;
                out.distance = wrap(s0 + (s1 - s0) * f, length_);
                out.centre = c;
                out.direction = {dx, 0, dz};
                out.lateral = lat;
            }
        }
        if (best < kInf || !use_hint) break;
        use_hint = false;
    }
    if (best < kInf) return out;
    // 2. Fallback (off the gates, e.g. far off the track or where gate lines
    //    cross inside a hairpin): nearest point of the centre polyline.
    use_hint = hint >= 0 && length_ > 0;
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            float s0 = m.cs[i], s1 = j ? m.cs[j] : m.cs[0] + length_;
            if (use_hint && outside_window(s0, s1)) continue;
            const sim::Vec3 &a = m.cpts[i], &b = m.cpts[j];
            float dx = b.x - a.x, dz = b.z - a.z, l2 = dx * dx + dz * dz;
            float t = l2 > 1e-9f ? std::clamp(((p.x - a.x) * dx + (p.z - a.z) * dz) / l2, 0.f, 1.f) : 0.f;
            float cx = a.x + dx * t, cz = a.z + dz * t, cy = a.y + (b.y - a.y) * t;
            float dy = p.y - cy;
            float d2 = (p.x - cx) * (p.x - cx) + (p.z - cz) * (p.z - cz) + dy * dy * 0.25f;
            if (d2 < best) {
                best = d2;
                float l = std::sqrt(std::max(l2, 1e-12f));
                out.distance = wrap(s0 + (s1 - s0) * t, length_);
                out.centre = {cx, cy, cz};
                out.direction = {dx / l, 0, dz / l};
                // Left of travel = up x forward = (fz, 0, -fx).
                out.lateral = (p.x - cx) * (dz / l) + (p.z - cz) * (-dx / l);
            }
        }
        // A hinted search that ends far from the line is retried globally.
        if (!use_hint || best < 60.0f * 60.0f) break;
        use_hint = false;
        best = kInf;
    }
    return out;
}

sim::Vec3 Track::point_at(float s, float lateral) const {
    const Impl& m = *impl_;
    size_t n = m.cpts.size();
    if (n == 0) return {};
    if (n == 1) return m.cpts[0];
    s = wrap(s, length_);
    size_t i;
    if (s < m.cs[0]) {
        i = n - 1;
        s += length_;
    } else {
        i = size_t(std::upper_bound(m.cs.begin(), m.cs.end(), s) - m.cs.begin()) - 1;
    }
    size_t j = (i + 1) % n;
    float s0 = m.cs[i], s1 = j ? m.cs[j] : m.cs[0] + length_;
    float f = s1 > s0 ? std::clamp((s - s0) / (s1 - s0), 0.f, 1.f) : 0.f;
    // Same interpolation as the gate search in locate(): linear blend
    // between the points at `lateral` on the two gate lines (bilinear patch).
    const sim::Vec3 &a = m.cpts[i], &b = m.cpts[j], &ta = m.gdir[i], &tb = m.gdir[j];
    float dx = ta.x + (tb.x - ta.x) * f, dz = ta.z + (tb.z - ta.z) * f;
    return {a.x + (b.x - a.x) * f + dz * lateral, a.y + (b.y - a.y) * f, a.z + (b.z - a.z) * f - dx * lateral};
}

std::vector<sim::Vec3> Track::racing_line(int index, float step) const {
    std::vector<sim::Vec3> out;
    if (index < 0) {
        for (int k : {0, 1, 2, 5, 6})
            if (size_t(k) < ai_.lines.size() && ai_.lines[size_t(k)].points.size() >= 2) {
                index = k;
                break;
            }
    }
    if (index < 0 || size_t(index) >= ai_.lines.size() || length_ <= 0) return out;
    const course::AiLine& line = ai_.lines[size_t(index)];
    size_t n = line.points.size();
    if (n < 2) return out;
    // The line is a chain of straights and circular arcs (AiLine::sample_arcs).
    // A line counts as a lap when its points cover most of the lap (the
    // first and last point can be far apart on a long straight).
    bool loop = line.closed;
    if (!loop && n >= 3) {
        float covered = 0, prev = -1, first = 0;
        for (size_t i = 0; i < n; ++i) {
            const course::AiPoint& p = line.points[i];
            float s = locate({p.pos.x, probe_or_ground(p.pos.x, p.pos.z, kInf).height, p.pos.z}, prev, 400.0f).distance;
            if (i) covered += wrap(s - prev, length_);
            else first = s;
            prev = s;
        }
        loop = covered > 0.6f * length_ && wrap(first - prev, length_) < 0.4f * length_;
    }
    float prev_y = kInf;
    for (const course::Vec3& p : line.sample_arcs(step, loop)) {
        float y = 0.0f;
        int ti = prev_y < kInf ? impl_->probe_index(p.x, p.z, prev_y + 3.0f, y) : -1;
        if (ti < 0) ti = impl_->probe_index(p.x, p.z, kInf, y);
        if (ti < 0) y = prev_y < kInf ? prev_y : 0.0f;
        prev_y = y;
        out.push_back({p.x, y, p.z});
    }
    return out;
}

sim::Vec3 Track::direction_at(float s) const {
    sim::Vec3 a = point_at(s - 2.0f), b = point_at(s + 2.0f);
    float dx = b.x - a.x, dz = b.z - a.z, l = std::hypot(dx, dz);
    if (l < 1e-6f) return {0, 0, -1};
    return {dx / l, 0, dz / l};
}

// --- walls ----------------------------------------------------------------------

void Track::wall_contacts(const sim::Vec3& c, float radius, std::vector<WallContact>& out) const {
    const Impl& m = *impl_;
    if (m.egrid.empty()) return;
    constexpr float kBehind = 2.0f; // how far behind a one-sided wall still pushes back
    float reach = radius + kBehind;
    int ix0, iz0, ix1, iz1;
    m.egrid.clamp_range(c.x - reach, c.z - reach, c.x + reach, c.z + reach, ix0, iz0, ix1, iz1);
    u32 seen[64];
    int nseen = 0;
    for (int iz = iz0; iz <= iz1; ++iz)
        for (int ix = ix0; ix <= ix1; ++ix) {
            size_t cell = size_t(iz) * m.egrid.nx + ix;
            for (u32 k = m.egrid.begin[cell]; k < m.egrid.begin[cell + 1]; ++k) {
                u32 id = m.egrid.items[k];
                if (std::find(seen, seen + nseen, id) != seen + nseen) continue;
                if (nseen < 64) seen[nseen++] = id;
                const WallEdge& e = m.edges[id];
                if (e.sensor) continue;
                if (c.y < e.ymin - 5.0f || c.y > e.ymax + 8.0f) continue;
                float dx = e.x1 - e.x0, dz = e.z1 - e.z0, l2 = dx * dx + dz * dz;
                float t = std::clamp(((c.x - e.x0) * dx + (c.z - e.z0) * dz) / l2, 0.f, 1.f);
                float qx = e.x0 + dx * t, qz = e.z0 + dz * t;
                float vx = c.x - qx, vz = c.z - qz, dist = std::hypot(vx, vz);
                WallContact wc;
                float side = (c.x - e.x0) * e.nx + (c.z - e.z0) * e.nz;
                if (side >= 0) {
                    if (dist >= radius) continue;
                    wc.normal = dist > 1e-5f ? sim::Vec3{vx / dist, 0, vz / dist} : sim::Vec3{e.nx, 0, e.nz};
                    wc.penetration = radius - dist;
                } else {
                    // Behind the wall: only within its span and a short depth.
                    if (t <= 0 || t >= 1 || -side > kBehind) continue;
                    wc.normal = {e.nx, 0, e.nz};
                    wc.penetration = radius - side;
                }
                wc.hit = true;
                wc.point = {qx, c.y, qz};
                wc.wall = e.wall;
                wc.edge = e.edge;
                out.push_back(wc);
            }
        }
}

WallContact Track::wall_contact(const sim::Vec3& centre, float radius) const {
    std::vector<WallContact> all;
    wall_contacts(centre, radius, all);
    WallContact best;
    for (const WallContact& c : all)
        if (!best.hit || c.penetration > best.penetration) best = c;
    return best;
}

// --- lap timing -------------------------------------------------------------------

LapTimer::LapTimer(const Track& track, int sector_count) : track_(track), sector_count_(std::max(1, sector_count)) {
    float L = track_.length();
    for (int i = 1; i <= sector_count_; ++i) sector_ends_.push_back(L * float(i) / float(sector_count_));
    reset();
}

void LapTimer::set_sector_distances(std::vector<float> ends) {
    if (ends.empty()) return;
    std::sort(ends.begin(), ends.end());
    ends.back() = track_.length();
    sector_ends_ = std::move(ends);
    sector_count_ = int(sector_ends_.size());
}

void LapTimer::reset(double time) {
    located_ = false;
    started_ = false;
    laps_ = 0;
    lap_start_ = sector_start_ = time;
    last_lap_ = best_lap_ = -1;
    sector_ = 0;
    sector_times_.clear();
    last_sectors_.clear();
    progress_ = 0;
    distance_ = 0;
    backwards_ = 0;
    wrong_way_ = false;
}

void LapTimer::teleport(const sim::Vec3& position) {
    float L = track_.length();
    TrackPosition tp = track_.locate(position);
    distance_ = tp.distance;
    // Keep the lap count; pick the unwrapped value nearest the old progress.
    double base = std::floor(progress_ / L) * L;
    double cand = base + tp.distance;
    if (cand - progress_ > L * 0.5) cand -= L;
    if (progress_ - cand > L * 0.5) cand += L;
    progress_ = cand;
    backwards_ = 0;
    wrong_way_ = false;
}

bool LapTimer::update(const sim::Vec3& position, double time) {
    float L = track_.length();
    if (L <= 0) return false;
    if (!located_) {
        TrackPosition tp = track_.locate(position);
        distance_ = tp.distance;
        // Cars on the grid sit just behind the line (distance near L).
        progress_ = tp.distance > L * 0.5f ? double(tp.distance) - L : double(tp.distance);
        located_ = true;
        if (progress_ >= 0) { // already past the line: the lap runs from now
            started_ = true;
            lap_start_ = sector_start_ = time;
        }
        return false;
    }
    TrackPosition tp = track_.locate(position, distance_);
    float delta = tp.distance - distance_;
    if (delta > L * 0.5f) delta -= L;
    if (delta < -L * 0.5f) delta += L;
    distance_ = tp.distance;
    double before = progress_;
    progress_ += delta;

    // Wrong way: sustained backwards travel along the lap.
    // Wrong way after 15 m net backwards; cleared after ~25 m forwards again.
    if (delta < 0) backwards_ = std::min(30.0f, backwards_ - delta);
    else backwards_ = std::max(0.f, backwards_ - delta);
    if (backwards_ > 15.0f) wrong_way_ = true;
    if (backwards_ < 5.0f) wrong_way_ = false;

    if (!started_) {
        if (before < 0 && progress_ >= 0) {
            started_ = true;
            lap_start_ = sector_start_ = time;
            sector_ = 0;
            sector_times_.clear();
        }
        return false;
    }
    // Sectors (except the last, which ends with the lap).
    double lap_pos = progress_ - double(laps_) * L;
    while (sector_ + 1 < sector_count_ && lap_pos >= sector_ends_[size_t(sector_)]) {
        sector_times_.push_back(time - sector_start_);
        sector_start_ = time;
        ++sector_;
    }
    if (progress_ >= double(laps_ + 1) * L) {
        sector_times_.push_back(time - sector_start_);
        last_sectors_ = sector_times_;
        sector_times_.clear();
        last_lap_ = time - lap_start_;
        if (best_lap_ < 0 || last_lap_ < best_lap_) best_lap_ = last_lap_;
        ++laps_;
        lap_start_ = sector_start_ = time;
        sector_ = 0;
        return true;
    }
    return false;
}

float LapTimer::lap_progress() const {
    float L = track_.length();
    if (L <= 0 || !started_) return 0;
    return float(std::clamp((progress_ - double(laps_) * L) / L, 0.0, 1.0));
}

double LapTimer::race_distance() const { return progress_; }

} // namespace gt3r::track
