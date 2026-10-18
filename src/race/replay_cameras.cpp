#include "race/replay_cameras.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::race {

namespace {

float rdf(ByteSpan d, size_t o) {
    u32 v = rd32(d, o);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}

GtrcCurve read_curve(ByteSpan d, u32 off) {
    GtrcCurve c;
    if (off == 0 || off + 4 > d.size()) return c;
    u32 n = rd32(d, off);
    if (n > 4096 || off + 4 + size_t(n) * 8 > d.size()) throw FormatError("GTRC curve out of range");
    for (u32 i = 0; i < n; ++i) c.keys.push_back({rdf(d, off + 4 + 8 * i), rdf(d, off + 8 + 8 * i)});
    return c;
}

GtrcRecord read_record(ByteSpan d, size_t o) {
    if (o + 0x80 > d.size()) throw FormatError("GTRC record out of range");
    GtrcRecord r;
    r.kind = rd32(d, o);
    r.start = rdf(d, o + 0x04);
    r.end = rdf(d, o + 0x08);
    r.duration = rdf(d, o + 0x0C);
    r.mount = rd32(d, o + 0x18);
    r.zoom_min = rdf(d, o + 0x28);
    r.zoom_max = rdf(d, o + 0x2C);
    r.zoom = read_curve(d, rd32(d, o + 0x34));
    for (int i = 0; i < 3; ++i) {
        r.pos[i] = read_curve(d, rd32(d, o + 0x40 + 4 * size_t(i)));
        r.target[i] = read_curve(d, rd32(d, o + 0x58 + 4 * size_t(i)));
    }
    return r;
}

} // namespace

float GtrcCurve::eval(float t) const {
    if (keys.empty()) return 0;
    if (keys.size() == 1) return keys[0].second;
    size_t i = 0;
    while (i + 1 < keys.size() && keys[i + 1].first <= t) ++i;
    i = std::min(i, keys.size() - 2);
    const auto &a = keys[i], &b = keys[i + 1];
    float span = b.first - a.first;
    if (span == 0) return a.second;
    return a.second + (b.second - a.second) * (t - a.first) / span;
}

GtrcScript GtrcScript::parse(ByteSpan d) {
    if (!is_gtrc(d)) throw FormatError("not a GTRC block");
    GtrcScript s;
    s.course_id = rd32(d, 0x14);
    s.lap_length = rdf(d, 0x18);
    s.script_end = rdf(d, 0x1C);
    u32 n = rd32(d, 0x24), off = rd32(d, 0x28);
    u32 n2 = rd32(d, 0x2C), off2 = rd32(d, 0x30);
    if (n > 1024 || n2 > 1024) throw FormatError("GTRC record count");
    for (u32 i = 0; i < n; ++i) s.records.push_back(read_record(d, off + 0x80 * size_t(i)));
    for (u32 i = 0; i < n2; ++i) s.start_shots.push_back(read_record(d, off2 + 0x80 * size_t(i)));
    return s;
}

int GtrcScript::variants() const {
    if (lap_length <= 0 || records.empty()) return 1;
    return std::max(1, int(script_end / lap_length) - 1 - int(records[0].start / lap_length));
}

std::optional<std::pair<int, float>> ReplayDirector::find(float D) const {
    float period = float(s_->variants()) * s_->lap_length;
    for (float d : {D, D - period, D + period})
        for (size_t i = 0; i < s_->records.size(); ++i) {
            const GtrcRecord& r = s_->records[i];
            if (r.end <= r.start || d < r.start || d >= r.end) continue;
            float u = (d - r.start) / (r.end - r.start);
            if (u <= 0.6f) return std::make_pair(int(i), d - D);
        }
    return std::nullopt;
}

ReplayDirector::Shot ReplayDirector::update(float s, int lap, const render::Vec3& car, double dt, float aspect) {
    Shot shot;
    if (!s_ || s_->records.empty() || s_->lap_length <= 0) return shot;
    const float L = s_->lap_length;
    int nvar = s_->variants();
    // Continue the current shot while the car is inside its range.
    if (cur_ >= 0) {
        const GtrcRecord& r = s_->records[size_t(cur_)];
        float D = float(k_) * L + s + offset_;
        // The car crossed the line during the shot.
        if (D < r.start - L * 0.5f) offset_ += L, D += L;
        if (D < r.start || D > r.end) cur_ = -1;
    }
    if (cur_ < 0) {
        offset_ = 0;
        int k = (lap % nvar) + 1;
        for (int tries = 0; tries < nvar && cur_ < 0; ++tries) {
            if (auto f = find(float(k) * L + s)) {
                cur_ = f->first;
                k_ = k;
                offset_ = f->second;
                zoom_ = 0;
            }
            k = k % nvar + 1;
        }
    }
    if (cur_ < 0) return shot;
    const GtrcRecord& r = s_->records[size_t(cur_)];
    shot.record = cur_;
    if (r.mode() == 3) {
        shot.valid = shot.onboard = true;
        return shot;
    }
    float D = float(k_) * L + s + offset_;
    float t = std::clamp((D - r.start) / std::max(r.end - r.start, 1e-3f), 0.0f, 1.0f);
    shot.eye = r.position(t);
    shot.target = r.looks_at_car() ? car + render::Vec3{0, 0.4f, 0}
                                   : render::Vec3{r.target[0].eval(t), r.target[1].eval(t), r.target[2].eval(t)};
    float want = r.zoom_by_distance() ? render::length(shot.target - shot.eye) * r.zoom.eval(0) : r.zoom.eval(t);
    if (r.zoom_max > r.zoom_min) want = std::clamp(want, r.zoom_min, r.zoom_max);
    want = std::max(want, 0.2f);
    // Smoothed like the game (exact rate not decoded); snap on a cut.
    zoom_ = zoom_ <= 0 ? want : zoom_ + (want - zoom_) * float(1 - std::exp(-4.0 * dt));
    // zoom = 1 / tan(half FOV). Taken as the horizontal half-angle of the PS2's
    // 4:3 frame (axis not confirmed); converted to our vertical FOV.
    (void)aspect;
    shot.fovy = 2.0f * std::atan(0.75f / zoom_);
    shot.valid = true;
    return shot;
}

} // namespace gt3r::race
