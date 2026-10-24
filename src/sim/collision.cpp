#include "sim/collision.h"

#include <algorithm>
#include <cmath>

namespace gt3r::sim::collide {

namespace {
constexpr double kMaxYawRate = 62.83185307179586; // commit clamp, 20 pi rad/s [physics §5.3]
// "Early hit" threshold of the near-stopped end-pose recompute
// [physics §5.3]: |vx| < 1 and |vy| < 1 and last hit t < 0.1, all strict.
constexpr double kEarlyHit = 0.1;
constexpr double kOnLine = 1e-5; // m
constexpr double kStuckYaw = 20; // rad/s^2 at full steer [physics §5.3]
} // namespace

std::array<V2, 8> octagon(double a, double b, double w) {
    double h = w * 0.5, d = 0.2 * w;
    return {{{h, a - d}, {h - d, a}, {-h + d, a}, {-h, a - d}, {-h, -b + d}, {-h + d, -b}, {h - d, -b}, {h, -b + d}}};
}

double bounding_radius(const std::array<V2, 8>& pts) {
    double r = 0;
    for (const V2& p : pts) r = std::max(r, std::sqrt(p.dot(p)));
    return r;
}

V2 to_world_dir(double c, const V2& l) {
    // lateral (x) -> world (cos c, -sin c); forward (y) -> world (-sin c, -cos c)
    double sc = std::sin(c), cc = std::cos(c);
    return {l.x * cc - l.z * sc, -l.x * sc - l.z * cc};
}

V2 world_point(const V2& pos, double heading, const V2& local) { return pos + to_world_dir(heading, local); }

namespace {

V2 point_velocity(const Body& b, const V2& r) {
    // omega about +Y: omega x r = (w * r.z, -w * r.x) in x/z.
    return {b.vel.x + b.yaw_rate * r.z, b.vel.z - b.yaw_rate * r.x};
}

double side(const Segment& s, const V2& p) {
    V2 d = s.b - s.a;
    double len = std::sqrt(d.dot(d));
    if (len < 1e-9) return 0;
    return d.cross(p - s.a) / len; // > 0 on the left_normal() side
}

double along(const Segment& s, const V2& p) {
    V2 d = s.b - s.a;
    double l2 = d.dot(d);
    return l2 > 1e-12 ? d.dot(p - s.a) / l2 : 0;
}

V2 left_normal(const Segment& s) {
    V2 d = s.b - s.a;
    double len = std::sqrt(d.dot(d));
    if (len < 1e-9) return {0, 0};
    // normal n with d.cross(n) > 0 (the positive side of side())
    return {d.z / len, -d.x / len};
}

} // namespace

double wall_impulse(Body& b, const V2& r, const V2& n, double e, double mu) {
    V2 vp = point_velocity(b, r);
    double vn = vp.dot(n);
    if (vn >= 0) return 0;
    double rn = r.cross(n);
    double k = b.inv_mass + b.inv_inertia * rn * rn;
    if (k <= 0) return 0;
    double j = -(1.0 + e) * vn / k;
    // Friction against the tangential contact velocity (3-D incl. vertical), no torque.
    V2 vt = vp - n * vn;
    double vtl = std::sqrt(vt.dot(vt) + b.vy * b.vy);
    b.vel = b.vel + n * (j * b.inv_mass);
    b.yaw_rate += b.inv_inertia * rn * j;
    if (vtl > 1e-9 && b.inv_mass > 0) {
        double mass = 1.0 / b.inv_mass;
        double f = std::min(mu * j, mass * vtl);
        double s = f * b.inv_mass / vtl;
        b.vel = b.vel - vt * s;
        double dvy = -b.vy * s;
        if (b.vy * (b.vy + dvy) < 0) dvy = -b.vy; // never crosses zero
        b.vy += dvy;
    }
    return j;
}

namespace {

V2 unit(const V2& d) {
    double l = std::sqrt(d.dot(d));
    return l > 1e-9 ? d * (1.0 / l) : V2{1, 0}; // a zero-length segment gives (1, 0) [physics §5.3]
}

// [physics §5.3] Persistence of flagged wall contacts on the pose (pos, heading): each
// flagged point walks to the neighbouring segment once if it slid past an end; the flag drops
// when the centre velocity points away from the wall or the point is on the track side.
// Otherwise the body moves so that the point lies on the wall line, the velocity is projected
// on the last hit wall's tangent and the yaw rate gets the angular part of that impulse,
// applied at the point. Returns true if the body was moved or its velocity changed.
bool hold_wall_contacts(Body& b, V2& pos, double heading, const std::vector<Segment>& walls, WallContacts& wc) {
    bool changed = false;
    for (int k = 0; k < 8; ++k) {
        if (!wc.on[size_t(k)]) continue;
        Segment& sg = wc.seg[size_t(k)];
        V2 p = world_point(pos, heading, b.local[size_t(k)]);
        V2 t = unit(sg.b - sg.a);
        double s = (p - sg.a).dot(t), len = std::sqrt((sg.b - sg.a).dot(sg.b - sg.a));
        if (s < 0 || s > len) { // one step along the wall line
            for (const Segment& o : walls)
                if (s < 0 ? (o.b.x == sg.a.x && o.b.z == sg.a.z) : (o.a.x == sg.b.x && o.a.z == sg.b.z)) {
                    sg = o;
                    break;
                }
            t = unit(sg.b - sg.a);
        }
        V2 n = left_normal(sg); // into the track
        double dn = (p - sg.a).dot(n);
        // A point on the line (as the hit left it; the original game's float rounding usually reads it
        // as on the track side) drops: a car still pressing in then takes a fresh impulse, with
        // its friction, on the next crossing.
        if (b.vel.dot(n) > 0 || dn >= -kOnLine) {
            wc.on[size_t(k)] = false;
            continue;
        }
        pos = pos - n * dn; // the point onto the wall line
        V2 v = b.vel, vt = wc.tangent * b.vel.dot(wc.tangent);
        V2 r = p - pos;
        if (b.inv_mass > 0) b.yaw_rate += b.inv_inertia * r.cross((vt - v) * (1.0 / b.inv_mass));
        b.vel = vt;
        changed = true;
    }
    return changed;
}

} // namespace

std::vector<WallHit> collide_walls(Body& b, const V2& start_pos, double start_heading,
                                   const std::vector<Segment>& walls, double dt, int iterations, WallContacts* wc) {
    std::vector<WallHit> hits;
    if (walls.empty()) {
        if (wc) wc->clear();
        return hits;
    }
    // Side of every point/segment pair at the start pose: the track side.
    const size_t nseg = walls.size();
    std::vector<signed char> start_side(8 * nseg, 0);
    for (int k = 0; k < 8; ++k) {
        V2 p = world_point(start_pos, start_heading, b.local[size_t(k)]);
        for (size_t s = 0; s < nseg; ++s) {
            double sd = side(walls[s], p);
            start_side[size_t(k) * nseg + s] = sd > 0 ? 1 : sd < 0 ? -1 : 0;
        }
    }
    std::array<bool, 8> excluded{};
    V2 p0 = start_pos;
    double h0 = start_heading;
    double remaining = 1.0; // fraction of dt still to move
    for (int it = 0; it < iterations; ++it) {
        if (wc && hold_wall_contacts(b, p0, h0, walls, *wc)) { // then the end pose again
            b.pos = p0 + b.vel * (dt * remaining);
            b.heading = h0 + b.yaw_rate * dt * remaining;
        }
        // Earliest crossing over points x segments along p0 -> end pose.
        double best_t = 2;
        int best_k = -1;
        size_t best_s = 0;
        for (int k = 0; k < 8; ++k) {
            if (excluded[size_t(k)] || (wc && wc->on[size_t(k)])) continue;
            V2 a = world_point(p0, h0, b.local[size_t(k)]);
            V2 e = world_point(b.pos, b.heading, b.local[size_t(k)]);
            for (size_t s = 0; s < nseg; ++s) {
                signed char ss = start_side[size_t(k) * nseg + s];
                if (ss <= 0) continue; // one-sided: only points coming from the left (track) side are stopped
                double s0 = side(walls[s], a) * ss, s1 = side(walls[s], e) * ss;
                if (s0 < 0 || s1 >= 0) continue;
                double t = s0 / (s0 - s1);
                V2 x = a + (e - a) * t;
                double u = along(walls[s], x);
                if (u < 0 || u > 1) continue;
                if (t < best_t) {
                    best_t = t;
                    best_k = k;
                    best_s = s;
                }
            }
        }
        if (best_k < 0) break;
        const Segment& w = walls[best_s];
        V2 n = left_normal(w) * double(start_side[size_t(best_k) * nseg + best_s]);
        // Advance to the hit (linear in position and heading).
        V2 pos_t = p0 + (b.pos - p0) * best_t;
        double head_t = h0 + (b.heading - h0) * best_t;
        V2 hit = world_point(pos_t, head_t, b.local[size_t(best_k)]);
        excluded[size_t(best_k)] = true;
        WallHit wh;
        wh.point = hit;
        wh.normal = n;
        wh.sensor = w.sensor;
        wh.body_point = best_k;
        if (!w.sensor) {
            b.pos = pos_t;
            b.heading = head_t;
            wh.impulse = wall_impulse(b, hit - pos_t, n);
            if (wc) {
                wc->on[size_t(best_k)] = true;
                wc->seg[size_t(best_k)] = w;
                wc->tangent = unit(w.b - w.a);
            }
            // Re-sweep the remainder with the new velocity.
            remaining *= (1 - best_t);
            p0 = b.pos;
            h0 = b.heading;
            b.pos = b.pos + b.vel * (dt * remaining);
            b.heading += b.yaw_rate * dt * remaining;
        }
        hits.push_back(wh);
    }
    // Penetration fix-up: points still beyond a wall are pushed back perpendicular.
    for (int pass = 0; pass < 2; ++pass) {
        for (int k = 0; k < 8; ++k) {
            if (wc && wc->on[size_t(k)]) continue;
            V2 e = world_point(b.pos, b.heading, b.local[size_t(k)]);
            for (size_t s = 0; s < nseg; ++s) {
                const Segment& w = walls[s];
                signed char ss = start_side[size_t(k) * nseg + s];
                if (ss <= 0 || w.sensor) continue; // only points that started on the track side
                double sd = side(w, e) * ss;
                if (sd >= 0 || sd < -1.5) continue; // only shallow penetrations of this segment
                double u = along(w, e);
                if (u < 0 || u > 1) continue;
                b.pos = b.pos + left_normal(w) * (double(ss) * -sd);
                e = world_point(b.pos, b.heading, b.local[size_t(k)]);
                if (wc && sd >= -0.5) { // a push-out of at most 0.5 m flags the point
                    wc->on[size_t(k)] = true;
                    wc->seg[size_t(k)] = w;
                    wc->tangent = unit(w.b - w.a);
                    break;
                }
            }
        }
    }
    return hits;
}

namespace {

// Octagon edge e of a body in its local frame: from point e to point e+1, outward normal n and
// offset d (inside: n.q + d < 0). The outline is counter-clockwise in (x lateral, y forward).
void edge_plane(const std::array<V2, 8>& pts, int e, V2& n, double& d, V2& p0, V2& p1) {
    p0 = pts[size_t(e)];
    p1 = pts[size_t((e + 1) % 8)];
    V2 t = p1 - p0;
    double len = std::sqrt(t.dot(t));
    n = len > 1e-12 ? V2{t.z / len, -t.x / len} : V2{0, 0};
    d = -n.dot(p0);
}

// World point -> body-local (x lateral, y forward).
V2 to_local(const V2& pos, double heading, const V2& w) {
    V2 r = w - pos;
    V2 lat = to_world_dir(heading, {1, 0}), fwd = to_world_dir(heading, {0, 1});
    return {r.dot(lat), r.dot(fwd)};
}

// Signed distance of local point q to edge e's line and its position u along the edge (0..1 = span).
void edge_distance(const Body& o, int e, const V2& q, double& dist, double& u) {
    V2 n, p0, p1;
    double d;
    edge_plane(o.local, e, n, d, p0, p1);
    dist = n.dot(q) + d;
    V2 t = p1 - p0;
    u = t.dot(q - p0) / std::max(t.dot(t), 1e-24);
}

// The car-car step works in the spec's plane frame (X, Y) = (x, -z): heading h rotates local
// (lx, ly) to (X + cos h lx - sin h ly, Y + sin h lx + cos h ly). The order of the arithmetic
// follows the step's structure so that a vertex left exactly on an edge line (after a static
// separation or a response) is decided the same way as in the original game.
struct PB {
    double im = 0, ii = 0;
    double lx[8]{}, ly[8]{};
    double enx[8]{}, eny[8]{}, ed[8]{}; // edge k: point k -> k+1, outward normal, inside n.q + d < 0
    double sx = 0, sy = 0, sh = 0, vx = 0, vy = 0, w = 0; // start pose, velocity
    double ex = 0, ey = 0, eh = 0;                          // end pose
    double radius = 0;
    double pwx[8]{}, pwy[8]{}; // start-world points (refreshed at the advance, moved by the separation)
    bool flag[8]{};            // set on the hitting vertex by a response
    int fe[8]{};               // the other body's edge that vertex hit
};

void pb_points(PB& b) {
    double c = std::cos(b.sh), s = std::sin(b.sh);
    for (int k = 0; k < 8; ++k) {
        b.pwx[k] = b.sx + c * b.lx[k] - s * b.ly[k];
        b.pwy[k] = b.sy + s * b.lx[k] + c * b.ly[k];
    }
}

void pb_shift(PB& b, double dx, double dy) {
    b.sx += dx;
    b.sy += dy;
    for (int k = 0; k < 8; ++k) {
        b.pwx[k] += dx;
        b.pwy[k] += dy;
    }
}

PB pb_from(const Body& u, const V2& start, double start_heading) {
    PB b;
    b.im = u.inv_mass;
    b.ii = u.inv_inertia;
    for (int k = 0; k < 8; ++k) {
        b.lx[k] = u.local[size_t(k)].x;
        b.ly[k] = u.local[size_t(k)].z;
    }
    for (int k = 0; k < 8; ++k) {
        int k1 = (k + 1) & 7;
        double ex = b.lx[k1] - b.lx[k], ey = b.ly[k1] - b.ly[k], l = std::sqrt(ex * ex + ey * ey);
        b.enx[k] = l > 0 ? ey / l : 0;
        b.eny[k] = l > 0 ? -ex / l : 0;
        b.ed[k] = -(b.enx[k] * b.lx[k] + b.eny[k] * b.ly[k]);
    }
    b.sx = start.x;
    b.sy = -start.z;
    b.sh = start_heading;
    b.vx = u.vel.x;
    b.vy = -u.vel.z;
    b.w = u.yaw_rate;
    b.radius = u.radius;
    pb_points(b);
    return b;
}

// Static separation [physics §5.3] of o's points out of t's outline along the line of
// centres t -> o: for each point of o inside t, the vector to where that line through the point
// meets t's boundary; the deepest (largest component along the centre line) decides, and both
// bodies move half of it.
void pb_separate(PB& t, PB& o) {
    double dcx = o.sx - t.sx, dcy = o.sy - t.sy, rr = t.radius + o.radius;
    if (dcx * dcx + dcy * dcy > rr * rr) return;
    double best = 0, bx = 0, by = 0;
    for (int m = 0; m < 8; ++m) {
        double mx = o.pwx[m], my = o.pwy[m], pen = 0, fx = 0, fy = 0;
        bool found = false, inside = true;
        for (int k = 0; k < 8 && inside; ++k) {
            int kp = (k + 7) & 7; // edge kp -> k
            double px = t.pwx[kp], py = t.pwy[kp], qx = t.pwx[k], qy = t.pwy[k];
            double wx = mx - px, wy = my - py, ex = qx - px, ey = qy - py;
            if (ex * wy - ey * wx < 0) {
                inside = false;
                break;
            }
            if (found) continue;
            double c1 = dcx * wy - dcy * wx;
            if (c1 < 0) continue;
            double c2 = dcx * (my - qy) - dcy * (mx - qx);
            if (c2 > 0) continue;
            double den = c1 - c2, sp = den == 0 ? 0 : c1 / den;
            fx = sp * ex - wx;
            fy = sp * ey - wy;
            pen = fx * dcx + fy * dcy;
            found = true;
        }
        if (inside && best < pen) {
            best = pen;
            bx = fx;
            by = fy;
        }
    }
    if (best > 0) {
        pb_shift(o, 0.5 * bx, 0.5 * by);
        pb_shift(t, -0.5 * bx, -0.5 * by);
    }
}

// Contact persistence [physics §5.3] for the flagged points of bd[i] (start-world points):
// left the edge span -> flag drops; outside the edge line -> flag drops; else both bodies move half
// way onto the line. In the last two cases, when the centres approach through the edge
// (v_rel x edge < 0, v_rel = v_hitter - v_other), the relative normal velocity is removed half
// from each body (not mass weighted).
void pb_persist(PB bd[2], int i) {
    PB &me = bd[i], &o = bd[1 - i];
    for (int k = 0; k < 8; ++k) {
        if (!me.flag[k]) continue;
        int e = me.fe[k];
        double px = o.pwx[e], py = o.pwy[e], qx = o.pwx[(e + 1) & 7], qy = o.pwy[(e + 1) & 7];
        double wx = me.pwx[k] - px, wy = me.pwy[k] - py, ex = qx - px, ey = qy - py, l2 = ex * ex + ey * ey;
        double u = (wx * ex + wy * ey) / l2;
        if (u < 0 || u > 1) {
            me.flag[k] = false;
            continue;
        }
        if (wx * ey - wy * ex > 0) {
            me.flag[k] = false;
        } else {
            double dx = 0.5 * (ex * u - wx), dy = 0.5 * (ey * u - wy);
            me.sx += dx;
            me.sy += dy;
            o.sx -= dx;
            o.sy -= dy;
        }
        double rvx = me.vx - o.vx, rvy = me.vy - o.vy;
        if (rvx * ey - rvy * ex < 0) {
            double f = (rvx * ex + rvy * ey) / l2;
            double ax = 0.5 * (ex * f - rvx), ay = 0.5 * (ey * f - rvy);
            me.vx += ax;
            me.vy += ay;
            o.vx -= ax;
            o.vy -= ay;
        }
    }
}

struct PHit {
    double t = 2;
    int A = 0, e = 0, k = 0;
    double cpx = 0, cpy = 0; // crossing point in A's frame
};

// Pair sweep [physics §5.3]: each point of B on a straight chord in A's frame, from the
// start-pose frame to the end-pose frame; a hit when it goes from outside (>= 0) to inside (<= 0)
// of A's edge e with the chord passing between the edge's end points. ts = the current start time.
void pb_sweep(const PB bd[2], int A, double ts, PHit& best) {
    const PB &a = bd[A], &b = bd[1 - A];
    double c0 = std::cos(-a.sh), s0 = std::sin(-a.sh), c1 = std::cos(-a.eh), s1 = std::sin(-a.eh);
    double dx0 = b.sx - a.sx, dy0 = b.sy - a.sy, dx1 = b.ex - a.ex, dy1 = b.ey - a.ey;
    double p0x = c0 * dx0 - s0 * dy0, p0y = s0 * dx0 + c0 * dy0, p1x = c1 * dx1 - s1 * dy1, p1y = s1 * dx1 + c1 * dy1;
    double rs = b.sh - a.sh, re = b.eh - a.eh;
    double cs = std::cos(rs), ss = std::sin(rs), ce = std::cos(re), se = std::sin(re);
    for (int k = 0; k < 8; ++k) {
        if (b.flag[k]) continue;
        double x0 = cs * b.lx[k] - ss * b.ly[k] + p0x, y0 = ss * b.lx[k] + cs * b.ly[k] + p0y;
        double x1 = ce * b.lx[k] - se * b.ly[k] + p1x, y1 = se * b.lx[k] + ce * b.ly[k] + p1y;
        double dx = x1 - x0, dy = y1 - y0;
        for (int e = 0; e < 8; ++e) {
            double d0 = x0 * a.enx[e] + y0 * a.eny[e] + a.ed[e];
            double d1 = x1 * a.enx[e] + y1 * a.eny[e] + a.ed[e];
            // A point starting on the edge line to float noise (after a static separation, or a
            // vertex whose flag just dropped) is decided by single-precision rounding in the
            // original. Reading it as already inside (no new hit) matches the original game best
            // (measured at 10x: 3941 vs 3972 impulses on overlapping starts; 5 of 7290
            // swept contacts differ by a missed or extra pulling re-hit).
            if (std::fabs(d0) < kOnLine) continue;
            if (d1 > 0 || d0 < 0) continue;
            int e1 = (e + 1) & 7;
            double f0 = (a.lx[e] - x0) * dy - (a.ly[e] - y0) * dx;
            double f1 = (a.lx[e1] - x0) * dy - (a.ly[e1] - y0) * dx;
            if ((f0 < 0 && f1 < 0) || (f0 > 0 && f1 > 0)) continue;
            double f2 = d0 - d1, f3 = f2 == 0 ? 1.0 : d0 / f2;
            double t = ts + (1 - ts) * f3;
            if (t < 0 || t > 1) continue;
            if (t < best.t) {
                best.t = t;
                best.A = A;
                best.e = e;
                best.k = k;
                best.cpx = x0 + dx * f3;
                best.cpy = y0 + dy * f3;
            }
        }
    }
}

} // namespace

double collide_pair_swept(Body& ua, const V2& a0, double ah0, Body& ub, const V2& b0, double bh0, double dt, int iterations,
                          V2* point, V2* normal, const float* steer, double* first_t, unsigned* sides) {
    // [physics §5.3] From the committed (start) poses; end pose = start + v dt (1 - t). At
    // most two rounds of: static separation (both orderings), persistence, end pose, pair sweep
    // (both orderings, earliest wins, ties to body 0), advance to the hit, response.
    PB bd[2] = {pb_from(ua, a0, ah0), pb_from(ub, b0, bh0)};
    double ts = 0, last = 1, total = 0;
    bool first = true;
    auto end_pose = [&](int i) {
        PB& b = bd[i];
        b.ex = b.sx + b.vx * dt * (1 - ts);
        b.ey = b.sy + b.vy * dt * (1 - ts);
        b.eh = b.sh + b.w * dt * (1 - ts);
    };
    for (int round = 0; round < std::min(std::max(iterations, 1), 2); ++round) {
        pb_separate(bd[0], bd[1]);
        pb_separate(bd[1], bd[0]);
        pb_persist(bd, 0);
        pb_persist(bd, 1);
        end_pose(0);
        end_pose(1);
        PHit best;
        pb_sweep(bd, 0, ts, best);
        PHit h1;
        pb_sweep(bd, 1, ts, h1);
        if (h1.t < best.t) best = h1;
        if (best.t >= 2) break;
        last = best.t;
        double frac = (1 - ts) != 0 ? (best.t - ts) / (1 - ts) : 0;
        for (PB& m : bd) {
            m.sx += (m.ex - m.sx) * frac;
            m.sy += (m.ey - m.sy) * frac;
            m.sh += (m.eh - m.sh) * frac;
            pb_points(m);
        }
        ts = best.t;
        PB &a = bd[best.A], &b = bd[1 - best.A];
        double ca = std::cos(a.sh), sa = std::sin(a.sh), cb = std::cos(b.sh), sb = std::sin(b.sh);
        double nx = ca * a.enx[best.e] - sa * a.eny[best.e], ny = sa * a.enx[best.e] + ca * a.eny[best.e];
        double rax = ca * best.cpx - sa * best.cpy, ray = sa * best.cpx + ca * best.cpy;
        double rbx = cb * b.lx[best.k] - sb * b.ly[best.k], rby = sb * b.lx[best.k] + cb * b.ly[best.k];
        double px = a.sx + rax, py = a.sy + ray;
        // touch-up: B's point still outside A's edge plane -> half the gap each along n
        double gap = (b.sx - a.sx + rbx) * nx + (b.sy - a.sy + rby) * ny + a.ed[best.e];
        if (gap > 0) {
            a.sx += 0.5 * gap * nx;
            a.sy += 0.5 * gap * ny;
            b.sx -= 0.5 * gap * nx;
            b.sy -= 0.5 * gap * ny;
        }
        // response: j = -1.5 (v_rel . n) / K, no friction, not clamped
        double vax = a.vx - a.w * ray, vay = a.vy + a.w * rax, vbx = b.vx - b.w * rby, vby = b.vy + b.w * rbx;
        double vn = (vax - vbx) * nx + (vay - vby) * ny;
        double rna = rax * ny - ray * nx, rnb = rbx * ny - rby * nx;
        double K = a.im + b.im + a.ii * rna * rna + b.ii * rnb * rnb;
        double j = K != 0 ? -1.5 * vn / K : 0;
        double jx = j * nx, jy = j * ny;
        a.vx += a.im * jx;
        a.vy += a.im * jy;
        a.w += a.ii * (rax * jy - ray * jx);
        b.vx -= b.im * jx;
        b.vy -= b.im * jy;
        b.w -= b.ii * (rbx * jy - rby * jx);
        b.flag[best.k] = true;
        b.fe[best.k] = best.e;
        total += std::fabs(j);
        if (sides) { // [ai §7] the edge body by the hit edge, the vertex body by its vertex
            sides[best.A] |= edge_side(best.e);
            sides[1 - best.A] |= point_side(best.k);
        }
        if (first) {
            if (point) *point = {px, -py};
            if (first_t) *first_t = best.t;
            double sg = best.A == 0 ? 1.0 : -1.0; // from body 0 towards body 1
            if (normal) *normal = {nx * sg, -ny * sg};
            first = false;
        }
    }
    // Commit: the bodies end at the last end pose computed (an impulse in the last
    // round changes only velocities); a nearly stopped body whose last hit was early gets the
    // stuck-steer yaw nudge and a recomputed end pose.
    Body* out[2] = {&ua, &ub};
    for (int i = 0; i < 2; ++i) {
        PB& b = bd[i];
        if (std::fabs(b.vx) < 1 && std::fabs(b.vy) < 1 && last < kEarlyHit) {
            // [interpretation: the nudge comes before the end pose, so it turns the car this step]
            if (steer) b.w += -double(steer[i]) * kStuckYaw * dt;
            end_pose(i);
        }
        Body& u = *out[i];
        u.pos = {b.ex, -b.ey};
        u.heading = b.eh;
        u.vel = {b.vx, -b.vy};
        u.yaw_rate = std::clamp(b.w, -kMaxYawRate, kMaxYawRate);
    }
    return total;
}

namespace {

// Impulse response [physics §5.3] of x's edge against y's vertex: rx = the contact point,
// ry = y's vertex; j = -1.5 (v_rel . n) / K, not clamped for separating contact; no friction.
double respond(Body& x, Body& y, const V2& nw, const V2& contact, const V2& vertex) {
    V2 rx = contact - x.pos, ry = vertex - y.pos;
    V2 vrel = point_velocity(x, rx) - point_velocity(y, ry);
    double kx = rx.cross(nw), ky = ry.cross(nw);
    double K = x.inv_mass + y.inv_mass + x.inv_inertia * kx * kx + y.inv_inertia * ky * ky;
    double j = K != 0 ? -1.5 * vrel.dot(nw) / K : 0;
    x.vel = x.vel + nw * (j * x.inv_mass);
    x.yaw_rate += x.inv_inertia * kx * j;
    y.vel = y.vel - nw * (j * y.inv_mass);
    y.yaw_rate -= y.inv_inertia * ky * j;
    return j;
}

} // namespace

double collide_pair(Body& a, Body& b, V2* point, V2* normal) {
    V2 dc = b.pos - a.pos;
    double rr = a.radius + b.radius;
    if (dc.dot(dc) > rr * rr) return 0;
    std::array<V2, 8> pa, pb;
    for (size_t k = 0; k < 8; ++k) {
        pa[k] = world_point(a.pos, a.heading, a.local[k]);
        pb[k] = world_point(b.pos, b.heading, b.local[k]);
    }
    // Separating axes (edge normals of both octagons): any gap = no contact.
    for (int poly = 0; poly < 2; ++poly) {
        const auto& P = poly == 0 ? pa : pb;
        for (size_t k = 0; k < 8; ++k) {
            V2 e = P[(k + 1) % 8] - P[k];
            double len = std::sqrt(e.dot(e));
            if (len < 1e-9) continue;
            V2 n{e.z / len, -e.x / len};
            double amin = 1e30, amax = -1e30, bmin = 1e30, bmax = -1e30;
            for (size_t j = 0; j < 8; ++j) {
                double x = pa[j].dot(n), y = pb[j].dot(n);
                amin = std::min(amin, x);
                amax = std::max(amax, x);
                bmin = std::min(bmin, y);
                bmax = std::max(bmax, y);
            }
            if (std::min(amax, bmax) - std::max(amin, bmin) <= 0) return 0;
        }
    }
    // The contact feature: the deepest vertex of either body inside the other, against the edge it is
    // nearest to (vertex into edge, as the swept response sees it).
    Body* body[2] = {&a, &b};
    double best = -1;
    int owner = -1, edge = -1, vertex = -1;
    for (int o = 0; o < 2; ++o) {
        const Body& x = *body[o];
        const Body& y = *body[1 - o];
        for (int k = 0; k < 8; ++k) {
            V2 q = to_local(x.pos, x.heading, world_point(y.pos, y.heading, y.local[size_t(k)]));
            double depth = 1e30;
            int de = -1;
            for (int e = 0; e < 8; ++e) {
                double dist, u;
                edge_distance(x, e, q, dist, u);
                if (-dist < depth) {
                    depth = -dist;
                    de = e;
                }
            }
            if (depth >= 0 && depth > best) {
                best = depth;
                owner = o;
                edge = de;
                vertex = k;
            }
        }
    }
    if (owner < 0) return 1e-6; // overlapping edges without a vertex inside: touching, no response
    Body& x = *body[owner];
    Body& y = *body[1 - owner];
    V2 n, p0, p1;
    double d;
    edge_plane(x.local, edge, n, d, p0, p1);
    V2 nw = to_world_dir(x.heading, n);
    V2 vtx = world_point(y.pos, y.heading, y.local[size_t(vertex)]);
    V2 c = vtx + nw * best; // the vertex projected onto x's edge
    double j = respond(x, y, nw, c, vtx);
    // split the overlap half and half
    x.pos = x.pos - nw * (0.5 * best);
    y.pos = y.pos + nw * (0.5 * best);
    if (point) *point = c;
    if (normal) *normal = owner == 0 ? nw : nw * -1.0; // from a towards b
    return std::max(std::fabs(j), 1e-6);
}

} // namespace gt3r::sim::collide
