#pragma once

// Point-light courses (night: Special Stage Route 5 / 11 and the circle tests; GTRW byte +0x21
// = 1). GTRW +0x58 count, +0x5C offset of 0x20-byte lamp records {u32 type n (Env0 car-light
// record n gives colours, attenuation and spot), vec3 spot axis, vec3 position, f32 kind (0
// directional, else point)}. The road vertices' attribute A holds the nearest lamp's index.
// Each car is lit by up to four lamps chosen from the nine around that index, by weight
// w = 1 / (c0 + c1 d + c2 d^2) (x max(r.axis, 0)^exp with a spot cutoff), as directional lights
// along car -> lamp with the record's colours x w; node 1 casts a shadow away from each.

#include "base/bytes.h"
#include "race/sun_glare.h"
#include "render/math3d.h"
#include "render/view3d.h"
#include "scene/model.h"

#include <array>
#include <memory>
#include <vector>

namespace gt3r::render {
class GpuContext;
class GpuModel;
} // namespace gt3r::render

namespace gt3r::race {

struct CourseLamp {
    int type = 0;
    render::Vec3 axis{0, 1, 0}, pos;
    float kind = 1;
};

struct SelectedLamp {
    render::Vec3 dir; // unit, car -> lamp (or the record's direction for kind 0)
    float w = 0;
    int type = 0;
    int lamp = -1; // index into lamps()
};

// A lamp's disc in the car reflection map (GT3 draws them over the sky / scenery map before
// the car: centre at the lamp's sphere-map point, radius size / distance in map units, an
// untextured 10-wedge fan, centre (r, g, b, a sqrt(w)) fading to alpha 0 at the rim, normal
// alpha blend, no depth test, the destination alpha kept, weakest last).
struct ReflectionDisc {
    render::Vec3 pos;                // the lamp (world)
    std::array<float, 4> rgba{};     // car-light record +0x70..+0x7C
    float sqrt_w = 0, size = 0;      // sqrt of the lamp weight, record +0x40 (metres)
};

class CoursePointLights {
public:
    // `gtrw` = course file slot 28. Inactive (day course) when byte +0x21 is 0.
    void load(ByteSpan gtrw);
    bool active() const { return active_ && !lamps_.empty(); }
    const std::vector<CourseLamp>& lamps() const { return lamps_; }
    // Up to four lamps for a car at `p` whose front-left wheel is over lamp index `hint`.
    std::vector<SelectedLamp> select(const SunParams& env, const render::Vec3& p, int hint) const;
    // The reflection map's discs for a car at `p`: of the same nine candidates sorted by weight,
    // the first six point lamps (kind != 0; weight-0 ones still take a slot, invisible).
    std::vector<ReflectionDisc> reflection_discs(const SunParams& env, const render::Vec3& p, int hint) const;

private:
    // The nine lamps around `hint` with their weights, strongest first (stable).
    std::vector<SelectedLamp> candidates(const SunParams& env, const render::Vec3& p, int hint) const;
    bool active_ = false;
    std::vector<CourseLamp> lamps_;
};

// The discs' geometry for the reflection cube (we render GT3's sphere map as a cube and read it
// through GT3's lookup, DrawOptions::env_gt3_lookup): each disc is built in the sphere map's
// space - axis `basis[2]` (towards the viewer, tilted 15 degrees up), `basis[0]` / `basis[1]` -
// and every vertex is mapped back to its world direction (map radius sin(phi / 2)), 1 m from the
// map's eye. One dynamic model for up to six discs.
class ReflectionDiscMesh {
public:
    ReflectionDiscMesh();
    ~ReflectionDiscMesh();
    void update(render::GpuContext& ctx, const std::vector<ReflectionDisc>& discs, const render::Vec3& eye,
                const render::Vec3 basis[3]);
    void add_item(std::vector<render::DrawItem>& items) const;
    static constexpr int kMaxDiscs = 6, kWedges = 10;

private:
    std::unique_ptr<render::GpuModel> gpu_;
    std::vector<scene::Vertex> verts_;
    render::Vec3 eye_;
    bool any_ = false;
};

} // namespace gt3r::race
