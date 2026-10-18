#include "formats/course/road.h"

#include <algorithm>
#include <cmath>

namespace gt3r::course {

namespace {

float rdf(ByteSpan d, size_t o) {
    u32 v = rd32(d, o);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}

Vec3 rdv(ByteSpan d, size_t o) { return {rdf(d, o), rdf(d, o + 4), rdf(d, o + 8)}; }

void check_table(ByteSpan d, u32 off, u64 count, u32 stride, const char* what) {
    if (off > d.size() || count * stride > d.size() - off)
        throw FormatError(std::string("GTRW: ") + what + " table out of range");
}

// Lookup grid: u32 cells_off, s16 nx, s16 nz, f32 xs[nx], f32 zs[nz]; at
// cells_off (nx-1)*(nz-1) u32 offsets, each to groups {u16 n, u16 segment,
// u16 triangle[n]} ending with n = 0.
RoadGrid parse_grid(ByteSpan d, u32 off, const std::vector<RoadSegment>& segs) {
    RoadGrid g;
    if (!off || off + 8 > d.size()) return g;
    u32 cells_off = rd32(d, off);
    int nx = s16(rd16(d, off + 4)), nz = s16(rd16(d, off + 6));
    if (nx < 2 || nz < 2) return g;
    check_table(d, off + 8, u64(nx) + nz, 4, "grid breakpoint");
    for (int i = 0; i < nx; ++i) g.xs.push_back(rdf(d, off + 8 + 4 * size_t(i)));
    for (int i = 0; i < nz; ++i) g.zs.push_back(rdf(d, off + 8 + 4 * size_t(nx + i)));
    u64 ncell = u64(nx - 1) * u64(nz - 1);
    check_table(d, cells_off, ncell, 4, "grid cell");
    g.cell_begin.reserve(ncell + 1);
    for (u64 c = 0; c < ncell; ++c) {
        g.cell_begin.push_back(u32(g.refs.size()));
        size_t p = rd32(d, cells_off + 4 * c);
        for (int guard = 0; guard < 4096; ++guard) {
            u32 n = rd16(d, p);
            if (!n) break;
            u16 seg = rd16(d, p + 2);
            if (seg >= segs.size()) throw FormatError("GTRW: grid segment out of range");
            for (u32 k = 0; k < n; ++k) {
                u16 t = rd16(d, p + 4 + 2 * k);
                if (t >= segs[seg].triangles.size()) throw FormatError("GTRW: grid triangle out of range");
                g.refs.push_back({seg, t});
            }
            p += 4 + 2 * size_t(n);
        }
    }
    g.cell_begin.push_back(u32(g.refs.size()));
    return g;
}

} // namespace

Road Road::parse(ByteSpan d) {
    if (!has_magic(d, "GTRW") || d.size() < 0x60) throw FormatError("GTRW: bad header");
    Road r;
    r.course_id = rd32(d, 0x14);
    r.length = rdf(d, 0x18);

    u32 nmark = rd16(d, 0x22), mark_off = rd32(d, 0x24);
    check_table(d, mark_off, nmark, 8, "marker");
    for (u32 i = 0; i < nmark; ++i) r.markers.push_back({rd32(d, mark_off + 8 * i), rdf(d, mark_off + 8 * i + 4)});

    u32 ngrid = rd32(d, 0x28), grid_off = rd32(d, 0x2C);
    check_table(d, grid_off, ngrid, 16, "grid");
    for (u32 i = 0; i < ngrid; ++i) r.grid.push_back({rdv(d, grid_off + 16 * i), rdf(d, grid_off + 16 * i + 12)});

    u32 nseg = rd32(d, 0x30), seg_off = rd32(d, 0x34);
    check_table(d, seg_off, nseg, 0x40, "segment");
    u32 nwall = rd32(d, 0x38), wall_off = rd32(d, 0x3C);
    check_table(d, wall_off, nwall, 16, "wall point");

    for (u32 i = 0; i < nseg; ++i) {
        size_t s = seg_off + size_t(i) * 0x40;
        RoadSegment seg;
        u32 nv = rd16(d, s), nt = rd16(d, s + 2);
        u32 voff = rd32(d, s + 4), toff = rd32(d, s + 8), woff = rd32(d, s + 0x10);
        seg.unknown_a = rd16(d, s + 0x14);
        seg.unknown_b = rd16(d, s + 0x16);
        seg.prev = rd16(d, s + 0x18);
        seg.next = rd16(d, s + 0x1A);
        check_table(d, voff, nv, 16, "vertex");
        check_table(d, toff, nt, 16, "triangle");
        for (u32 k = 0; k < nv; ++k) {
            seg.vertices.push_back(rdv(d, voff + 16 * k));
            seg.vertex_attrs.push_back(rd32(d, voff + 16 * k + 12));
        }
        // Neighbour list: u16 count at +0x16, ids from +0x18 (room for 20).
        u32 nn = std::min<u32>(rd16(d, s + 0x16), 20);
        for (u32 k = 0; k < nn; ++k) {
            u16 id = rd16(d, s + 0x18 + 2 * k);
            if (id < nseg) seg.neighbours.push_back(id);
        }
        for (u32 k = 0; k < nt; ++k) {
            size_t t = toff + 16 * k;
            RoadTriangle tri{rd16(d, t), rd16(d, t + 2), rd16(d, t + 4), rd32(d, t + 8), rd32(d, t + 12),
                             rd16(d, t + 8), rd16(d, t + 12)};
            if (tri.a >= nv || tri.b >= nv || tri.c >= nv) throw FormatError("GTRW: triangle index out of range");
            seg.triangles.push_back(tri);
        }
        // Wall references: 16-byte rows of u16 values (meaning only partly
        // known), up to the next segment's vertex block. Kept raw.
        size_t wend = i + 1 < nseg ? rd32(d, seg_off + (i + 1) * 0x40 + 4) : woff + 16;
        if (woff && woff < wend && wend <= d.size())
            for (size_t p = woff; p + 2 <= wend; p += 2) seg.wall_refs.push_back(rd16(d, p));
        if (!seg.vertices.empty()) {
            seg.bounds_min = seg.bounds_max = seg.vertices[0];
            for (const Vec3& v : seg.vertices) {
                seg.bounds_min = {std::min(seg.bounds_min.x, v.x), std::min(seg.bounds_min.y, v.y),
                                  std::min(seg.bounds_min.z, v.z)};
                seg.bounds_max = {std::max(seg.bounds_max.x, v.x), std::max(seg.bounds_max.y, v.y),
                                  std::max(seg.bounds_max.z, v.z)};
            }
        }
        r.segments.push_back(std::move(seg));
    }

    for (u32 i = 0; i < nwall; ++i) r.wall_points.push_back({rdv(d, wall_off + 16 * i), rd32(d, wall_off + 16 * i + 12)});
    for (size_t i = 0; i < r.wall_points.size();) {
        u32 n = r.wall_points[i].tag & 0xFFFF;
        if (n == 0 || i + n > r.wall_points.size()) n = u32(r.wall_points.size() - i); // malformed: take the rest
        std::vector<Vec3> line;
        for (size_t k = i; k < i + n; ++k) line.push_back(r.wall_points[k].pos);
        r.walls.push_back(std::move(line));
        i += n;
    }

    r.start_distance = rdf(d, 0x1C);
    r.header_flag = rd16(d, 0x20);
    try {
        r.grid_index = parse_grid(d, rd32(d, 0x48), r.segments);
    } catch (const FormatError&) {
        r.grid_index = {}; // optional acceleration data; callers fall back to scanning
    }

    u32 ncp = rd32(d, 0x40), cp_off = rd32(d, 0x44);
    check_table(d, cp_off, ncp, 32, "checkpoint");
    for (u32 i = 0; i < ncp; ++i) {
        size_t c = cp_off + 32 * i;
        r.checkpoints.push_back({rdf(d, c), rdf(d, c + 4), rdf(d, c + 8), rdf(d, c + 12), rdf(d, c + 16),
                                 rdf(d, c + 20), rdf(d, c + 24), rdf(d, c + 28)});
    }
    return r;
}

size_t Road::triangle_count() const {
    size_t n = 0;
    for (const RoadSegment& s : segments) n += s.triangles.size();
    return n;
}

std::optional<float> Road::height_at(float x, float z, float y_max, u32* surface) const {
    std::optional<float> best;
    for (const RoadSegment& s : segments) {
        if (x < s.bounds_min.x || x > s.bounds_max.x || z < s.bounds_min.z || z > s.bounds_max.z) continue;
        for (const RoadTriangle& t : s.triangles) {
            const Vec3 &a = s.vertices[t.a], &b = s.vertices[t.b], &c = s.vertices[t.c];
            float d = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
            if (std::fabs(d) < 1e-12f) continue;
            float l0 = ((b.z - c.z) * (x - c.x) + (c.x - b.x) * (z - c.z)) / d;
            float l1 = ((c.z - a.z) * (x - c.x) + (a.x - c.x) * (z - c.z)) / d;
            float l2 = 1 - l0 - l1;
            const float eps = -1e-5f;
            if (l0 < eps || l1 < eps || l2 < eps) continue;
            float y = l0 * a.y + l1 * b.y + l2 * c.y;
            if (y > y_max) continue;
            if (!best || y > *best) {
                best = y;
                if (surface) *surface = t.surface;
            }
        }
    }
    return best;
}

std::vector<Vec3> Road::centre_line() const {
    std::vector<const Checkpoint*> cps;
    for (const Checkpoint& c : checkpoints) cps.push_back(&c);
    std::sort(cps.begin(), cps.end(), [](auto* a, auto* b) { return a->distance < b->distance; });
    std::vector<Vec3> out;
    for (const Checkpoint* c : cps) {
        Vec3 p{(c->x0 + c->x1) * 0.5f, 0, (c->z0 + c->z1) * 0.5f};
        if (auto y = height_at(p.x, p.z)) p.y = *y;
        out.push_back(p);
    }
    return out;
}

} // namespace gt3r::course
