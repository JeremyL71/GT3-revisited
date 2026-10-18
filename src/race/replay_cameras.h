#pragma once

// GT3's replay camera script ("GTRC": course file slot 34, also
// cameras/start.cam and cameras/attach.cam). Layout and behaviour as found by
// the race workstream's research (summary in docs/dev/RACE.md):
//
//   header (offsets from "GTRC"): +0x14 u32 course id, +0x18 f32 lap length L,
//     +0x1C f32 script end T, +0x24 u32 record count, +0x28 rel-ptr records
//     (0x80 bytes each, sorted by start), +0x2C/+0x30 second list (start shots).
//   record: +0x00 u32 kind, +0x04 f32 start D, +0x08 f32 end D, +0x0C f32 duration,
//     +0x18 u32 on-board mount, +0x28/+0x2C f32 zoom min/max, rel-ptr curves:
//     +0x34 zoom, +0x40/+0x44/+0x48 camera x/y/z, +0x58/+0x5C/+0x60 target x/y/z.
//   curve: u32 n, n x {f32 t, f32 value}; piecewise linear in t (0..1), linear
//     extrapolation, n = 1 is constant.
//   kind: &3 mode (0 trackside world coordinates, 3 on-board, 1 car-local),
//     &0xC != 0 look at the car (+0.4 m), else at the target curves;
//     0x10 zoom = |camera - target| * zoom(0), else zoom(t).
//
// Records are laid out by track distance D = k * L + s, where s is the car's
// distance along the lap and k a "lap variant" (1..nvar; nvar = max(1,
// int(T / L) - 1 - int(first.start / L))) so consecutive laps use different
// cuts. Within a record t = clamp((D - start) / (end - start), 0, 1).

#include "base/bytes.h"
#include "render/math3d.h"

#include <optional>
#include <utility>
#include <vector>

namespace gt3r::race {

struct GtrcCurve {
    std::vector<std::pair<float, float>> keys; // (t, value)
    float eval(float t) const;
    bool empty() const { return keys.empty(); }
};

struct GtrcRecord {
    u32 kind = 0;
    float start = 0, end = 0, duration = 0;
    u32 mount = 0;
    float zoom_min = 1, zoom_max = 100;
    GtrcCurve zoom, pos[3], target[3];

    int mode() const { return int(kind & 3); }
    bool looks_at_car() const { return (kind & 0xC) != 0; }
    bool zoom_by_distance() const { return (kind & 0x10) != 0; }
    render::Vec3 position(float t) const { return {pos[0].eval(t), pos[1].eval(t), pos[2].eval(t)}; }
};

struct GtrcScript {
    u32 course_id = 0;
    float lap_length = 0, script_end = 0;
    std::vector<GtrcRecord> records;
    std::vector<GtrcRecord> start_shots; // second list

    static bool is_gtrc(ByteSpan d) { return has_magic(d, "GTRC") && d.size() >= 0x40; }
    static GtrcScript parse(ByteSpan d); // throws FormatError
    int variants() const;
};

// Picks and evaluates the current replay camera, like the game's simple path:
// keep the shot while the car is inside its range; otherwise take the first
// record whose range contains D with u = (D - start) / (end - start) in [0, 0.6],
// trying lap variants k = lap + 1, ... (wrapping to 1).
class ReplayDirector {
public:
    explicit ReplayDirector(const GtrcScript* s = nullptr) : s_(s) {}
    void reset() { cur_ = -1, k_ = 0, zoom_ = 0, offset_ = 0; }

    struct Shot {
        bool valid = false;       // false: no record covers the car (use a fallback camera)
        bool onboard = false;     // mode 3: use an on-board view
        int record = -1;
        render::Vec3 eye, target;
        float fovy = 0.8f;        // radians, vertical
    };
    // `s` = car's track distance (0..L), `lap` = completed laps, `car` = car position,
    // `dt` = frame time (zoom smoothing), `aspect` = viewport width / height.
    Shot update(float s, int lap, const render::Vec3& car, double dt, float aspect);

private:
    std::optional<std::pair<int, float>> find(float D) const; // record, wrapped D

    const GtrcScript* s_;
    int cur_ = -1, k_ = 0;
    float zoom_ = 0;
    float offset_ = 0; // added to k * L + s (wrap of the script, line crossings during a shot)
};

} // namespace gt3r::race
