// Tests for the track runtime (src/track/): synthetic ring road, then the
// real courses from the disc (skipped without GT3R_DISC).

#include "test_framework.h"

#include "track/track.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <random>

using namespace gt3r;
using namespace gt3r::track;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kR = 100, kHalfWidth = 10;
constexpr int kSegs = 64;

sim::Vec3 ring(float theta, float r = kR, float y = 0) { return {r * std::cos(theta), y, r * std::sin(theta)}; }

// A flat ring road of radius 100 m, 20 m wide, travelled with increasing
// angle; one checkpoint per segment; walls on both edges; a flat "bridge"
// deck 10 m above the road around theta = pi/2; one grid slot behind the line.
course::Road make_ring() {
    course::Road r;
    r.length = 2 * kPi * kR;
    for (int i = 0; i < kSegs; ++i) {
        float t0 = 2 * kPi * float(i) / kSegs, t1 = 2 * kPi * float(i + 1) / kSegs;
        course::RoadSegment s;
        for (auto p : {ring(t0, kR - kHalfWidth), ring(t0, kR + kHalfWidth), ring(t1, kR + kHalfWidth),
                       ring(t1, kR - kHalfWidth)})
            s.vertices.push_back({p.x, p.y, p.z});
        u16 mat = i == 5 ? 2 : 0; // one grass segment
        s.triangles.push_back({0, 1, 2, mat, 0, mat, 0});
        s.triangles.push_back({0, 2, 3, mat, 0x0201, mat, 0x02}); // wet flag on half of it
        s.bounds_min = {-kR - kHalfWidth, 0, -kR - kHalfWidth}; // loose bounds suffice for height_at
        s.bounds_max = {kR + kHalfWidth, 0, kR + kHalfWidth};
        r.segments.push_back(std::move(s));
        course::Checkpoint c;
        sim::Vec3 a = ring(t0, kR - kHalfWidth), b = ring(t0, kR + kHalfWidth);
        c.x0 = a.x, c.z0 = a.z, c.x1 = b.x, c.z1 = b.z;
        c.distance = kR * t0;
        c.dir_x = std::sin(t0), c.dir_z = -std::cos(t0); // stored against travel, like the game data
        r.checkpoints.push_back(c);
    }
    // Bridge deck: a 30 m square 10 m above the road at theta = pi/2.
    course::RoadSegment deck;
    for (auto p : {sim::Vec3{-15, 10, 85}, sim::Vec3{15, 10, 85}, sim::Vec3{15, 10, 115}, sim::Vec3{-15, 10, 115}})
        deck.vertices.push_back({p.x, p.y, p.z});
    deck.triangles.push_back({0, 2, 1, 0, 0, 0, 0});
    deck.triangles.push_back({0, 3, 2, 0, 0, 0, 0});
    deck.bounds_min = {-15, 10, 85};
    deck.bounds_max = {15, 10, 115};
    r.segments.push_back(std::move(deck));
    // Walls as raw tagged points (closed loops, drivable side on the left of
    // the direction): the inner wall runs with increasing angle, the outer
    // one against it. Point 3 of the outer wall is a corner; the outer edge
    // starting at point 10 is a sensor.
    for (int w = 0; w < 2; ++w) {
        float rad = w == 0 ? kR - kHalfWidth : kR + kHalfWidth;
        const int n = 128;
        for (int i = 0; i < n; ++i) {
            float a = 2 * kPi * float(w == 0 ? i : n - i) / n;
            sim::Vec3 p = ring(a, rad);
            u32 tag = i == 0 ? u32(n) : i == n - 1 ? u32(0x10000 - n) : 0;
            if (w == 1 && i == 3) tag |= u32(kWallCorner) << 16;
            if (w == 1 && i == 10) tag |= u32(kWallSensor) << 16;
            r.wall_points.push_back({{p.x, 0, p.z}, tag});
        }
    }
    course::GridSlot g;
    sim::Vec3 gp = ring(-0.05f);
    g.pos = {gp.x, 0, gp.z};
    g.yaw = 0.05f; // heading = yaw + pi faces along increasing theta here
    r.grid.push_back(g);
    return r;
}

bool near(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

} // namespace

TEST(track_surface_helpers) {
    CHECK(surface_type(0x0000) == Surface::Tarmac);
    CHECK(surface_type(0x0001) == Surface::Kerb);
    CHECK(surface_type(0x0102) == Surface::Grass);
    CHECK(surface_type(0x0005) == Surface::Dirt);
    CHECK(surface_type(0x0006) == Surface::Unknown);
    CHECK(surface_flags(0x1803) == 0x18 && surface_material(0x1803) == 3);
    CHECK(surface_physics_index(0x0200) == 12);
    CHECK(surface_physics_index(0x0201) == 13);
    CHECK(surface_physics_index(0x0209) == 14);
    CHECK(surface_physics_index(0x0207) == 12);
    CHECK(surface_physics_index(0x0202) == 2);
    CHECK(surface_physics_index(0x0009) == 9);
}

TEST(track_synthetic_probe) {
    Track t(make_ring());
    CHECK(std::string(t.index_source()) == "built");
    sim::Vec3 p = ring(1.0f);
    sim::SurfaceHit h = t.probe(p.x, p.z, 5);
    CHECK(h.hit && near(h.height, 0, 1e-4f) && near(h.normal.y, 1, 1e-4f) && h.segment >= 0);
    CHECK(!t.probe(0, 0, 100).hit); // ring centre: off the road
    CHECK(!t.probe(p.x, p.z, -1).hit); // road above y_max
    // Bridge: the deck above wins unless y_max is below it.
    sim::Vec3 b = ring(kPi / 2);
    CHECK(near(t.probe(b.x, b.z, 100).height, 10, 1e-4f));
    CHECK(near(t.probe(b.x, b.z, 5).height, 0, 1e-4f));
    CHECK(near(t.probe(b.x, b.z, 10).height, 10, 1e-4f)); // "at or below"
    // Surface packing: material | flags << 8.
    sim::Vec3 g = ring(2 * kPi * 5.5f / kSegs);
    sim::SurfaceHit gh = t.probe(g.x, g.z, 5);
    CHECK(gh.hit && surface_type(gh.surface) == Surface::Grass);
    // Agrees with the brute-force reference.
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-120, 120), uy(-5, 20);
    int bad = 0;
    for (int i = 0; i < 2000; ++i) {
        float x = u(rng), z = u(rng), y = uy(rng);
        auto ref = t.road().height_at(x, z, y);
        sim::SurfaceHit q = t.probe(x, z, y);
        bad += bool(ref) != q.hit || (ref && !near(*ref, q.height, 1e-3f));
    }
    CHECK(bad == 0);
    // Off the mesh: ground estimate, never a hit.
    sim::SurfaceHit off = t.probe_or_ground(0, 0, 100);
    CHECK(!off.hit && off.segment == -1 && near(off.normal.y, 1, 1e-6f));
}

TEST(track_synthetic_geometry) {
    Track t(make_ring());
    CHECK(near(t.length(), 2 * kPi * kR, 1e-3f));
    // Grid: on the road, nose along the lap.
    CHECK(t.grid_count() == 1);
    Pose g = t.grid_pose(0);
    TrackPosition gp = t.locate(g.position);
    CHECK(gp.distance > t.length() - 10);
    CHECK(g.forward.x * gp.direction.x + g.forward.z * gp.direction.z > 0.99f);
    // The orientation rotates the car nose (-Z) onto `forward`.
    {
        const sim::Quat& q = g.orientation;
        // v' = q v q*, v = (0, 0, -1)
        float fx = -(2 * (q.x * q.z + q.w * q.y)), fz = -(1 - 2 * (q.x * q.x + q.y * q.y));
        CHECK(near(fx, g.forward.x, 1e-4f) && near(fz, g.forward.z, 1e-4f));
    }
    // locate / point_at round trip; lateral positive to the left of travel.
    for (float s : {5.f, 100.f, 333.f, 600.f}) {
        for (float d : {-6.f, 0.f, 7.f}) {
            sim::Vec3 p = t.point_at(s, d);
            TrackPosition tp = t.locate(p);
            CHECK(near(tp.distance, s, 0.5f) && near(tp.lateral, d, 0.2f));
        }
    }
    // Travelling with increasing theta (+X towards +Z), left = up x forward
    // points away from the ring centre, so the inside is negative.
    TrackPosition in = t.locate(ring(1.0f, kR - 5));
    CHECK(in.lateral < -4 && in.lateral > -6);
    Pose rp = t.centre_pose(200);
    CHECK(t.probe(rp.position.x, rp.position.z, rp.position.y + 1).hit);
    CHECK(rp.forward.x * t.direction_at(200).x + rp.forward.z * t.direction_at(200).z > 0.99f);
}

TEST(track_synthetic_walls) {
    Track t(make_ring());
    CHECK(t.wall_lines().size() == 2 && t.wall_lines()[0].closed && t.wall_lines()[1].closed);
    CHECK(t.wall_lines()[1].points[3].flags == kWallCorner && t.wall_lines()[1].points[10].flags == kWallSensor);
    // The sensor edge (outer wall, points 10 -> 11) never collides.
    {
        const auto& pts = t.wall_lines()[1].points;
        sim::Vec3 mid = (pts[10].pos + pts[11].pos) * 0.5f;
        sim::Vec3 in = mid * (1.0f - 0.5f / (kR + kHalfWidth)); // 0.5 m inside
        in.y = 0.5f;
        CHECK(!t.wall_contact(in, 1.0f).hit);
        sim::Vec3 mid2 = (pts[20].pos + pts[21].pos) * 0.5f;
        sim::Vec3 in2 = mid2 * (1.0f - 0.5f / (kR + kHalfWidth));
        in2.y = 0.5f;
        CHECK(t.wall_contact(in2, 1.0f).hit);
    }
    // Close to the outer wall from the inside: pushed back inwards.
    sim::Vec3 p = ring(0.3f, kR + kHalfWidth - 0.5f, 0.5f);
    WallContact c = t.wall_contact(p, 1.0f);
    CHECK(c.hit && near(c.penetration, 0.5f, 0.05f));
    sim::Vec3 out = ring(0.3f, 1.0f); // radial unit vector
    CHECK(c.normal.x * out.x + c.normal.z * out.z < -0.99f);
    // Slightly behind the wall (tunnelled through): still pushed inwards.
    WallContact b = t.wall_contact(ring(0.3f, kR + kHalfWidth + 0.5f, 0.5f), 1.0f);
    CHECK(b.hit && b.penetration > 1.4f && b.normal.x * out.x + b.normal.z * out.z < -0.99f);
    // Mid-road: nothing.
    CHECK(!t.wall_contact(ring(0.3f, kR, 0.5f), 1.0f).hit);
    // Far above the wall (bridge deck height + margin): ignored.
    CHECK(!t.wall_contact(ring(0.3f, kR + kHalfWidth - 0.5f, 30.f), 1.0f).hit);
}

TEST(track_synthetic_laps) {
    Track t(make_ring());
    LapTimer lt(t);
    double time = 0;
    int laps_events = 0;
    double prev = -1e9;
    bool monotonic = true;
    // From the grid, two full laps plus a bit, 1 m steps at 50 m/s.
    for (float s = -8; s < 2 * t.length() + 20; s += 1.0f) {
        sim::Vec3 p = t.point_at(s, 3.0f);
        time += 0.02;
        laps_events += lt.update(p, time);
        monotonic &= lt.race_distance() >= prev - 1e-3;
        prev = lt.race_distance();
        CHECK(!lt.wrong_way());
    }
    CHECK(monotonic);
    CHECK(laps_events == 2 && lt.laps_completed() == 2);
    CHECK(near(float(lt.last_lap_time()), t.length() * 0.02f, 0.05f));
    CHECK(lt.best_lap_time() > 0 && lt.last_sector_times().size() == 3);
    double sum = 0;
    for (double st : lt.last_sector_times()) sum += st;
    CHECK(near(float(sum), float(lt.last_lap_time()), 1e-3f));
    // Reverse 40 m: wrong way, and crossing the line backwards and forwards
    // again does not count a lap.
    LapTimer w(t);
    time = 0;
    w.update(t.point_at(-8, 0), time);
    for (float s = -8; s < 30; s += 1) w.update(t.point_at(s, 0), time += 0.02);
    CHECK(w.lap_started() && !w.wrong_way());
    for (float s = 30; s > -20; s -= 1) w.update(t.point_at(s, 0), time += 0.02);
    CHECK(w.wrong_way());
    for (float s = -20; s < 30; s += 1) w.update(t.point_at(s, 0), time += 0.02);
    CHECK(!w.wrong_way() && w.laps_completed() == 0);
    // Teleport keeps the lap count and does not add progress.
    double before = w.race_distance();
    w.teleport(t.point_at(32, 0));
    CHECK(near(float(w.race_distance() - before), 3, 0.6f)); // last update was at s = 29
}

// The original's lap clock (presentation §B9): a crossing at
// clock-before-the-tick + trunc(50 f) units of 1/3000 s, ms = floor(units / 3); a race times
// lap 1 from GO, a time trial from the first crossing; a frozen (finished) timer counts no
// more laps.
TEST(track_lap_clock) {
    Track t(make_ring());
    const double L = t.length(), dt = 1.0 / 60;
    // 30 m/s (0.5 m a tick) from 10.205 m behind the line: the line is passed on tick 20
    // (0-based) at f = 0.41, so at floor((50 x 20 + trunc(20.5)) / 3) = 340 ms.
    const double s0 = -10.205, step = 0.5;
    auto run = [&](LapTimer& lt, int ticks, int first_tick = 0) {
        int done = -1;
        for (int k = first_tick; k < first_tick + ticks; ++k)
            if (lt.update(t.point_at(float(s0 + step * double(k + 1)), 0), double(k + 1) * dt) && done < 0) done = k;
        return done;
    };
    LapTimer tt(t), race(t), lic(t);
    race.set_lap1_from_go(true);
    lic.set_lap1_from_go(true, false); // a standing licence test: from GO, shown past the line
    for (LapTimer* lt : {&tt, &race, &lic}) lt->update(t.point_at(float(s0), 0), 0);
    // Before the line: the race's lap time runs from GO, the others show 0.
    for (LapTimer* lt : {&tt, &race, &lic}) run(*lt, 6);
    CHECK(near(float(race.current_lap_time(6 * dt)), 0.1f, 1e-6f)); // floor(300 / 3) ms
    CHECK(tt.current_lap_time(6 * dt) == 0 && lic.current_lap_time(6 * dt) == 0);
    for (LapTimer* lt : {&tt, &race, &lic}) run(*lt, 15, 6); // over the line on tick 20
    CHECK(tt.lap_started() && race.lap_started() && lic.lap_started());
    CHECK(near(float(tt.current_lap_time(21 * dt)), 0.350f - 0.340f, 1e-6f)); // from the crossing
    CHECK(near(float(lic.current_lap_time(21 * dt)), 0.350f, 1e-6f));         // from GO
    // Lap 1 ends at the line's crossing time; lap 1 = that minus 340 ms (time trial) or minus
    // 0 (from GO).
    int k_end = int(std::ceil((L - s0) / step)) - 1;
    double before = s0 + step * double(k_end);
    float f = float((L - before) / step);
    long long end_ms = (50LL * k_end + (long long)(50.0f * f)) / 3;
    int k_tt = run(tt, k_end + 5 - 21, 21), k_race = run(race, k_end + 5 - 21, 21);
    CHECK(k_tt == k_end && k_race == k_end);
    CHECK(std::fabs(race.last_lap_time() - double(end_ms) / 1000.0) <= 0.0015); // (locate precision)
    CHECK(std::fabs(tt.last_lap_time() - double(end_ms - 340) / 1000.0) <= 0.0015);
    CHECK(near(float(race.last_crossing_time()), float(race.last_lap_time()), 1e-6f));
    // Frozen after its finish: no more laps; the lap time runs on from the finish.
    race.freeze();
    int k2 = run(race, int(L / step) + 10, k_end + 5);
    CHECK(k2 < 0 && race.laps_completed() == 1);
    CHECK(race.current_lap_time(double(k_end + 5 + int(L / step) + 10) * dt) > 0.9 * L / 30.0);
}

// --- real courses -----------------------------------------------------------------

namespace {

struct CourseResult {
    bool grid_ok = false, lap_ok = false, probe_ok = false;
    int centre_poses = 0, centre_on_road = 0; // every 25 m: on the racing surface
};

CourseResult check_course(const Track& t, bool licence, bool verbose) {
    CourseResult r;
    // Grid: every slot (licence variants: slot 0 only, the rest are leftovers)
    // is on the road and faces along the lap.
    int slots = licence ? std::min(1, t.grid_count()) : t.grid_count();
    r.grid_ok = slots > 0;
    for (int i = 0; i < slots; ++i) {
        Pose p = t.grid_pose(i);
        TrackPosition tp = t.locate(p.position);
        bool ok = t.probe(p.position.x, p.position.z, p.position.y + 0.5f).hit &&
                  p.forward.x * tp.direction.x + p.forward.z * tp.direction.z > 0.7f;
        if (!ok && verbose) std::printf("  %s: grid slot %d off road or facing backwards\n", t.label().c_str(), i);
        r.grid_ok &= ok;
    }
    // Racing line: one lap exactly, monotonic progress.
    std::vector<sim::Vec3> line = t.racing_line();
    Pose pole = t.grid_pose(0);
    size_t start = 0;
    float best = 1e30f;
    for (size_t i = 0; i < line.size(); ++i) {
        float d = std::hypot(line[i].x - pole.position.x, line[i].z - pole.position.z);
        if (d < best) best = d, start = i;
    }
    LapTimer lt(t);
    double time = 0, prev = -1e30;
    bool mono = true;
    lt.update(pole.position, time);
    size_t n = line.size() + line.size() / 20;
    for (size_t k = 0; k < n && !line.empty(); ++k) {
        lt.update(line[(start + k) % line.size()], time += 0.05);
        mono &= lt.race_distance() >= prev - 1.0;
        prev = lt.race_distance();
    }
    r.lap_ok = !line.empty() && mono && lt.laps_completed() == 1;
    if (!r.lap_ok && verbose)
        std::printf("  %s: racing line %zu samples, laps %d, monotonic %d\n", t.label().c_str(), line.size(),
                    lt.laps_completed(), int(mono));
    // Probe vs brute force.
    float bx0 = 1e30f, bz0 = 1e30f, bx1 = -1e30f, bz1 = -1e30f;
    for (const auto& s : t.road().segments) {
        bx0 = std::min(bx0, s.bounds_min.x), bz0 = std::min(bz0, s.bounds_min.z);
        bx1 = std::max(bx1, s.bounds_max.x), bz1 = std::max(bz1, s.bounds_max.z);
    }
    std::mt19937 rng(99);
    std::uniform_real_distribution<float> ux(bx0, bx1), uz(bz0, bz1), uy(-50, 300);
    int bad = 0;
    for (int i = 0; i < 1500; ++i) {
        float x = ux(rng), z = uz(rng), y = uy(rng);
        auto ref = t.road().height_at(x, z, y);
        sim::SurfaceHit h = t.probe(x, z, y);
        bad += bool(ref) != h.hit || (ref && std::fabs(*ref - h.height) > 1e-3f);
    }
    for (const sim::Vec3& p : line) { // along the line too (mostly hits)
        auto ref = t.road().height_at(p.x, p.z, p.y + 1);
        sim::SurfaceHit h = t.probe(p.x, p.z, p.y + 1);
        bad += bool(ref) != h.hit || (ref && std::fabs(*ref - h.height) > 1e-3f);
    }
    r.probe_ok = bad == 0;
    for (float sd = 0; sd < t.length(); sd += 25.0f) {
        Pose rp = t.centre_pose(sd);
        sim::SurfaceHit h = t.probe(rp.position.x, rp.position.z, rp.position.y + 0.5f);
        ++r.centre_poses;
        r.centre_on_road += h.hit && surface_type(h.surface) != Surface::Grass &&
                              surface_type(h.surface) != Surface::Gravel && surface_type(h.surface) != Surface::Sand;
    }
    if (!r.probe_ok && verbose) std::printf("  %s: %d probe mismatches\n", t.label().c_str(), bad);
    return r;
}

} // namespace

TEST(track_trialmt) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    const Volume::Entry* e = disc->volume().find("courses/US/trialmt");
    CHECK(e != nullptr);
    if (!e) return;
    Bytes data = disc->volume().read(*e);
    auto t = Track::from_course_file(data, "trialmt");
    CHECK(std::string(t->index_source()) == "game");
    CHECK(std::fabs(t->length() - 3922.f) < 1.f);
    CourseResult r = check_course(*t, false, true);
    CHECK(r.grid_ok && r.lap_ok && r.probe_ok);
    // The racing line stays on the road and clear of the walls.
    std::vector<sim::Vec3> line = t->racing_line();
    size_t on = 0, walls = 0;
    for (const sim::Vec3& p : line) {
        on += t->probe(p.x, p.z, p.y + 1).hit;
        walls += t->wall_contact({p.x, p.y + 0.5f, p.z}, 1.0f).hit;
    }
    CHECK(line.size() > 1500 && on == line.size() && walls == 0);
    // Trial Mountain's pit lane carries flag bit 0; most of the lap is tarmac.
    Pose pole = t->grid_pose(0);
    CHECK(surface_type(t->probe(pole.position.x, pole.position.z, pole.position.y + 1).surface) == Surface::Tarmac);

    // Benchmark: probes along the racing line (hits) and at random points.
    std::vector<std::array<float, 3>> qs;
    for (const sim::Vec3& p : line) qs.push_back({p.x, p.z, p.y + 1});
    std::mt19937 rng(5);
    std::uniform_real_distribution<float> ux(-700, 630), uz(-240, 490);
    for (int i = 0; i < 2000; ++i) qs.push_back({ux(rng), uz(rng), 200});
    volatile float sink = 0;
    auto t0 = std::chrono::steady_clock::now();
    const int reps = 50;
    for (int k = 0; k < reps; ++k)
        for (const auto& q : qs) sink = sink + t->probe(q[0], q[1], q[2]).height;
    double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - t0).count() /
                double(qs.size() * reps);
    std::printf("  trialmt: %.1f ns per probe (%zu probes)\n", ns, qs.size() * reps);
    CHECK(ns < 1000);
}

TEST(track_all_us) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    size_t files = 0, grid = 0, lap = 0, lap_expected = 0, probe = 0, loaded = 0;
    int centre_poses = 0, centre_ok = 0;
    size_t lines = 0, closed = 0, points = 0, corners = 0, sensors = 0, other_flags = 0;
    for (const auto& e : disc->volume().entries()) {
        if (e.path.rfind("courses/US/", 0) != 0) continue;
        ++files;
        std::string name = e.path.substr(11);
        Bytes data = disc->volume().read(e);
        std::unique_ptr<Track> t;
        try {
            t = Track::from_course_file(data, name);
        } catch (const std::exception& ex) {
            std::printf("  %s: %s\n", name.c_str(), ex.what());
            continue;
        }
        ++loaded;
        // The 60 m circle licence tests have a 3-point "line" that is not a lap.
        bool lap_line = name.rfind("L_circle60", 0) != 0;
        CourseResult r = check_course(*t, name.rfind("L_", 0) == 0, true);
        grid += r.grid_ok;
        probe += r.probe_ok;
        lap_expected += lap_line;
        lap += lap_line && r.lap_ok;
        centre_poses += r.centre_poses, centre_ok += r.centre_on_road;
        for (const WallLine& l : t->wall_lines()) {
            ++lines;
            closed += l.closed;
            for (const WallPoint& p : l.points) {
                ++points;
                corners += (p.flags & kWallCorner) != 0;
                sensors += (p.flags & kWallSensor) != 0;
                other_flags += (p.flags & ~(kWallCorner | kWallSensor)) != 0;
            }
        }
    }
    std::printf("  walls: %zu lines (%zu closed), %zu points: %zu corner, %zu sensor, %zu other flags\n", lines, closed,
                points, corners, sensors, other_flags);
    // Every polyline is a closed loop; corner flags are common. The US disc
    // has no sensor-flagged wall points at all.
    CHECK(lines > 0 && closed == lines && corners > 1000 && sensors == 0 && other_flags == 0);
    std::printf("  centre-line poses on the racing surface: %d/%d\n", centre_ok, centre_poses);
    CHECK(centre_ok >= centre_poses * 99 / 100);
    std::printf("  %zu US courses: loaded %zu, grid ok %zu, one lap along the racing line %zu/%zu, probe ok %zu\n",
                files, loaded, grid, lap, lap_expected, probe);
    CHECK(files == 80 && loaded == files && grid == files && probe == files);
    CHECK(lap == lap_expected && lap_expected == 78);
}
