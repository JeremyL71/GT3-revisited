// AI lines prepared for driving.

#include "ai/ai_driver.h"

#include "sim/track_walls.h"
#include "track/track.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace gt3r::ai {

namespace {

constexpr float kPi = std::numbers::pi_v<float>;

float wrap_pi(float a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

// The arc of radius |r| from p to q that leaves p closest to direction (dx, dz)
// (the same choice as course::AiLine::sample_arcs).
bool fit_arc(const sim::Vec3& p, const sim::Vec3& q, float r, float dx, float dz, AiCourse::Point& out) {
    float cx = q.x - p.x, cz = q.z - p.z, c = std::hypot(cx, cz);
    if (c < 1e-3f || std::fabs(r) <= 0.5f * c) return false;
    float h = std::sqrt(r * r - 0.25f * c * c);
    float lx = cz / c, lz = -cx / c; // left of the chord (Y up)
    float side = r > 0 ? 1.f : -1.f, rr = std::fabs(r);
    float best = -1e30f;
    for (int major = 0; major < 2; ++major) {
        float sd = major ? -side : side;
        float ox = p.x + cx * 0.5f + lx * h * sd, oz = p.z + cz * 0.5f + lz * h * sd;
        float a0 = std::atan2(p.z - oz, p.x - ox), a1 = std::atan2(q.z - oz, q.x - ox);
        float da = wrap_pi(a1 - a0);
        if (major) da += da > 0 ? -2 * kPi : 2 * kPi;
        float g = da > 0 ? 1.f : -1.f;
        float tx = -std::sin(a0) * g, tz = std::cos(a0) * g;
        float dot = tx * dx + tz * dz;
        if (dot > best) {
            best = dot;
            out.cx = ox, out.cz = oz, out.a0 = a0, out.da = da, out.rr = rr;
        }
    }
    out.arc = true;
    return true;
}

} // namespace

AiCourse::AiCourse(const track::Track& track) : track_(track), length_(track.length()) {
    sim::TrackWorld world(track);
    const auto& file_lines = track.ai_lines().lines;
    lines_.resize(8);
    for (size_t fl = 0; fl < file_lines.size() && fl < 7; ++fl) {
        const course::AiLine& src = file_lines[fl];
        Line& line = lines_[fl + 1];
        float prev = -1, covered = 0;
        for (const course::AiPoint& sp : src.points) {
            Point p;
            sim::SurfaceHit top = track.probe_or_ground(sp.pos.x, sp.pos.z, 1e30f);
            p.pos = {sp.pos.x, top.height, sp.pos.z};
            // Follow the previous point's height where the road passes over itself.
            if (!line.points.empty()) {
                sim::SurfaceHit h = track.probe(sp.pos.x, sp.pos.z, line.points.back().pos.y + 6.0f);
                if (h.hit) top = h, p.pos.y = h.height;
            }
            p.raw_kind = u8(sp.flags & 0xFF);
            p.heading = sp.heading;
            p.kind = p.raw_kind ? 1 : 0;
            p.brake_adjust = s8(sp.flags >> 8);
            p.speed_adjust = s8(sp.flags >> 16);
            p.radius = sp.radius();
            if (p.radius == 0) p.kind = 0;
            track::TrackPosition tp = track.locate(p.pos, prev);
            p.progress = tp.distance;
            if (prev >= 0) {
                float d = std::fmod(p.progress - prev + length_, length_);
                if (d < 0.5f * length_) covered += d;
            }
            prev = p.progress;
            p.grip = top.hit ? world.surface_grip(top.surface).mu : 1.0f;
            // [placeholder] slope and bank from the road normal (the game fills
            // point +0x1C/+0x20 at load; its derivation was not read).
            sim::Vec3 fwd = tp.direction, left{fwd.z, 0, -fwd.x};
            p.slope = std::asin(std::clamp(-top.normal.dot(fwd), -1.0f, 1.0f));
            float lean_left = std::asin(std::clamp(top.normal.dot(left), -1.0f, 1.0f));
            p.bank = p.radius > 0 ? lean_left : p.radius < 0 ? -lean_left : 0;
            line.points.push_back(p);
        }
        if (line.points.size() < 2) continue;
        // A lap line: covered most of the lap and no gap (including the
        // closing one from the last point back to the first) over half a lap.
        float closing = std::fmod(line.points.front().progress - line.points.back().progress + 2 * length_, length_);
        float max_gap = closing;
        for (size_t i = 1; i < line.points.size(); ++i)
            max_gap = std::max(max_gap, std::fmod(line.points[i].progress - line.points[i - 1].progress + 2 * length_, length_));
        line.lap = covered > 0.5f * length_ && max_gap < 0.5f * length_;
        size_t n = line.points.size();
        for (size_t i = 0; i < n; ++i) {
            Point& p = line.points[i];
            if (!p.kind) continue;
            if (i + 1 == n && !line.lap) continue;
            const Point& q = line.points[(i + 1) % n];
            sim::Vec3 d = track.direction_at(p.progress);
            if (!fit_arc(p.pos, q.pos, p.radius, d.x, d.z, p)) p.arc = false;
            // The game's own construction (spec §2.3): centre = P_i + r x the
            // left normal of the stored heading, angles from that centre to
            // P_i and P_j, interpolated, at distance |r|. Used when the
            // heading is consistent with the radius (the next point lies
            // within 25 % of |r| from that centre); the chord fit above
            // differs from it where the radius is about half the chord.
            {
                float nx = std::cos(p.heading), nz = std::sin(p.heading); // left normal of (-sin h, cos h)
                float cx = p.pos.x + nx * p.radius, cz = p.pos.z + nz * p.radius, rr = std::fabs(p.radius);
                float dq = std::hypot(q.pos.x - cx, q.pos.z - cz);
                if (rr > 1 && std::fabs(dq - rr) < 0.25f * rr && std::hypot(p.pos.x - cx, p.pos.z - cz) > 0.5f * rr) {
                    float a0 = std::atan2(p.pos.z - cz, p.pos.x - cx), a1 = std::atan2(q.pos.z - cz, q.pos.x - cx);
                    float da = a1 - a0;
                    float dir = p.arc ? (p.da >= 0 ? 1.f : -1.f) : 1.f;
                    if (!p.arc) {
                        // Direction from the stored heading: the travel direction
                        // (-sin h, cos h) as an angle change around the centre.
                        float tx = -std::sin(p.heading), tz = std::cos(p.heading);
                        float rx = p.pos.x - cx, rz = p.pos.z - cz;
                        dir = (rx * tz - rz * tx) > 0 ? 1.f : -1.f;
                    }
                    while (dir > 0 && da < 0) da += 2 * kPi;
                    while (dir < 0 && da > 0) da -= 2 * kPi;
                    p.arc = true, p.cx = cx, p.cz = cz, p.a0 = a0, p.da = da, p.rr = rr;
                }
            }
        }
    }
}

AiCourse::~AiCourse() = default;

const AiCourse::Line* AiCourse::line(int game_line) const {
    if (game_line < 1 || game_line > 7) return nullptr;
    const Line& l = lines_[size_t(game_line)];
    return l.empty() ? nullptr : &l;
}

int AiCourse::resolve(int wanted) const {
    static const int kFallback[8][3] = {{-1, -1, -1}, {1, 6, 2}, {2, 6, 1}, {3, -1, -1},
                                        {4, -1, -1},  {-1, -1, -1}, {6, 1, 2}, {7, 6, -1}};
    if (wanted < 0 || wanted > 7) wanted = 6;
    for (int c : kFallback[wanted])
        if (c > 0 && line(c)) return c;
    return -1;
}

float AiCourse::ahead(float a, float b) const {
    float d = std::fmod(b - a, length_);
    return d < 0 ? d + length_ : d;
}

int AiCourse::next_point(int game_line, float s) const {
    const Line* l = line(game_line);
    if (!l) return 0;
    const auto& pts = l->points;
    int n = int(pts.size());
    // Points are in lap order starting anywhere; find the bracketing span
    // [i, i+1) containing s (cyclic for lap lines).
    int spans = l->lap ? n : n - 1;
    for (int i = 0; i < spans; ++i) {
        const Point &p = pts[size_t(i)], &q = pts[size_t((i + 1) % n)];
        float span = ahead(p.progress, q.progress);
        if (span <= 0) continue;
        if (ahead(p.progress, s) < span) return (i + 1) % n == 0 && !l->lap ? n : (i + 1) % n;
    }
    // Outside a partial line: before its start -> 0, otherwise past the end.
    if (!l->lap && ahead(s, pts.front().progress) < 0.5f * length_) return 0;
    return l->lap ? 0 : n;
}

sim::Vec3 AiCourse::evaluate(int game_line, float s) const {
    const Line* l = line(game_line);
    if (!l) return track_.point_at(s);
    const auto& pts = l->points;
    int n = int(pts.size());
    int j = next_point(game_line, s);
    if (!l->lap && (j <= 0 || j >= n)) return j <= 0 ? pts.front().pos : pts.back().pos;
    int i = (j - 1 + n) % n;
    const Point &p = pts[size_t(i)], &q = pts[size_t(j % n)];
    float span = ahead(p.progress, q.progress);
    float t = span > 0 ? std::clamp(ahead(p.progress, s) / span, 0.0f, 1.0f) : 0.0f;
    float y = p.pos.y + (q.pos.y - p.pos.y) * t;
    if (p.arc) {
        float a = p.a0 + p.da * t;
        return {p.cx + p.rr * std::cos(a), y, p.cz + p.rr * std::sin(a)};
    }
    return {p.pos.x + (q.pos.x - p.pos.x) * t, y, p.pos.z + (q.pos.z - p.pos.z) * t};
}

bool AiCourse::side_swapped(float s) const {
    const Line* l = line(kSideMap);
    if (!l) return false;
    int j = next_point(kSideMap, s);
    int n = int(l->points.size());
    if (!l->lap && (j <= 0 || j >= n)) return false;
    return l->points[size_t((j - 1 + n) % n)].raw_kind != 0;
}

} // namespace gt3r::ai
