#pragma once

// Adapter: track::Track walls as a sim::WorldQuery (surfaces use the sim's
// own physics §3.6 tables via the raw SurfaceHit::surface value), so physics
// stays independent of the track runtime.
//
//   sim::TrackWorld world(*track);
//   car->step(inputs, *track, dt, &world);

#include "sim/vehicle.h"
#include "track/track.h"

#include <algorithm>
#include <vector>

namespace gt3r::sim {

class TrackWorld : public WorldQuery {
public:
    explicit TrackWorld(const track::Track& t) : track_(t) {}

    bool wall_contact(const Vec3& centre, float radius, Vec3& normal, float& penetration) const override {
        track::WallContact c = track_.wall_contact(centre, radius);
        if (!c.hit) return false;
        normal = c.normal;
        penetration = c.penetration;
        return true;
    }

    // Edges of each wall line (track::Track::wall_lines: per-point flags and
    // closure), within `radius` in x/z and in the height band [-5 m, +8 m]
    // around the centre (bridges). Edges starting at a kWallSensor point are
    // marked sensor: crossings are reported but get no response [physics §5.3].
    bool wall_segments(const Vec3& c, float radius, std::vector<collide::Segment>& out) const override {
        for (const auto& line : track_.wall_lines()) {
            size_t n = line.points.size();
            size_t edges = line.closed ? n : (n ? n - 1 : 0);
            for (size_t i = 0; i < edges; ++i) {
                const auto& pa = line.points[i];
                const auto& pb = line.points[(i + 1) % n];
                const Vec3 &p = pa.pos, &q = pb.pos;
                float ylo = std::min(p.y, q.y), yhi = std::max(p.y, q.y);
                if (c.y < ylo - 8.0f || c.y > yhi + 5.0f) continue;
                float minx = std::min(p.x, q.x) - radius, maxx = std::max(p.x, q.x) + radius;
                float minz = std::min(p.z, q.z) - radius, maxz = std::max(p.z, q.z) + radius;
                if (c.x < minx || c.x > maxx || c.z < minz || c.z > maxz) continue;
                collide::Segment sg;
                sg.a = {p.x, p.z};
                sg.b = {q.x, q.z};
                sg.sensor = (pa.flags & track::kWallSensor) != 0;
                out.push_back(sg);
            }
        }
        return true;
    }

private:
    const track::Track& track_;
};

} // namespace gt3r::sim
