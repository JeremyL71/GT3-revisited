#include "race/road_shade.h"

#include <algorithm>
#include <cmath>

namespace gt3r::race {

namespace {

// Tests one triangle; updates `best` when it contains (x, z) below y_max and is higher.
void test(const course::RoadSegment& s, const course::RoadTriangle& t, float x, float z, float y_max, float& best_y,
          RoadShade& out) {
    if (t.a >= s.vertices.size() || t.b >= s.vertices.size() || t.c >= s.vertices.size()) return;
    const course::Vec3 &a = s.vertices[t.a], &b = s.vertices[t.b], &c = s.vertices[t.c];
    float d = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
    if (std::fabs(d) < 1e-12f) return;
    float l0 = ((b.z - c.z) * (x - c.x) + (c.x - b.x) * (z - c.z)) / d;
    float l1 = ((c.z - a.z) * (x - c.x) + (a.x - c.x) * (z - c.z)) / d;
    float l2 = 1 - l0 - l1;
    const float eps = -1e-5f;
    if (l0 < eps || l1 < eps || l2 < eps) return;
    float y = l0 * a.y + l1 * b.y + l2 * c.y;
    if (y > y_max || (out.hit && y <= best_y)) return;
    best_y = y;
    auto attr = [&](u16 i, int hi) -> float {
        if (i >= s.vertex_attrs.size()) return 1.0f;
        return float((s.vertex_attrs[i] >> (hi ? 16 : 0)) & 0xFFFF) / 65535.0f;
    };
    out.hit = true;
    out.a = l0 * attr(t.a, 0) + l1 * attr(t.b, 0) + l2 * attr(t.c, 0);
    out.b = l0 * attr(t.a, 1) + l1 * attr(t.b, 1) + l2 * attr(t.c, 1);
    out.raw_a = t.a < s.vertex_attrs.size() ? u16(s.vertex_attrs[t.a] & 0xFFFF) : 0;
}

} // namespace

RoadShade road_shade(const course::Road& road, float x, float z, float y_max, bool use_grid) {
    RoadShade out;
    float best_y = 0;
    const course::RoadGrid& g = road.grid_index;
    if (use_grid && !g.empty()) {
        auto cell = [](const std::vector<float>& v, float p) -> int {
            auto it = std::upper_bound(v.begin(), v.end(), p);
            return int(it - v.begin()) - 1;
        };
        int cx = cell(g.xs, x), cz = cell(g.zs, z);
        int nx = int(g.xs.size()) - 1, nz = int(g.zs.size()) - 1;
        if (cx < 0 || cz < 0 || cx >= nx || cz >= nz) return out;
        size_t ci = size_t(cz) * size_t(nx) + size_t(cx);
        if (ci + 1 >= g.cell_begin.size()) return out;
        for (u32 k = g.cell_begin[ci]; k < g.cell_begin[ci + 1] && k < g.refs.size(); ++k) {
            const course::RoadGridRef& r = g.refs[k];
            if (r.segment >= road.segments.size()) continue;
            const course::RoadSegment& s = road.segments[r.segment];
            if (r.triangle < s.triangles.size()) test(s, s.triangles[r.triangle], x, z, y_max, best_y, out);
        }
        return out;
    }
    for (const course::RoadSegment& s : road.segments) {
        if (x < s.bounds_min.x || x > s.bounds_max.x || z < s.bounds_min.z || z > s.bounds_max.z) continue;
        for (const course::RoadTriangle& t : s.triangles) test(s, t, x, z, y_max, best_y, out);
    }
    return out;
}

} // namespace gt3r::race
