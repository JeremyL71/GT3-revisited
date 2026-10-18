#include "race/replay.h"

#include "base/file.h"
#include "base/log.h"
#include "sim/car_spec_io.h"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>

namespace gt3r::race {

namespace {

constexpr char kMagic[8] = {'G', 'T', '3', 'R', 'R', 'P', 'L', '1'};

class Writer {
public:
    template <class T>
    void put(T v) {
        u8 b[sizeof(T)];
        std::memcpy(b, &v, sizeof(T)); // little-endian hosts only (all our targets)
        out.insert(out.end(), b, b + sizeof(T));
    }
    void str(const std::string& s) {
        put<u16>(u16(std::min<size_t>(s.size(), 0xFFFF)));
        out.insert(out.end(), s.begin(), s.begin() + std::min<size_t>(s.size(), 0xFFFF));
    }
    void pose(const TrackPose& p) {
        for (float f : {p.position.x, p.position.y, p.position.z, p.orientation.w, p.orientation.x, p.orientation.y,
                        p.orientation.z})
            put<float>(f);
    }
    Bytes out;
};

class Reader {
public:
    explicit Reader(ByteSpan d) : d_(d) {}
    template <class T>
    T get() {
        if (pos_ + sizeof(T) > d_.size()) throw FormatError("replay truncated");
        T v;
        std::memcpy(&v, d_.data() + pos_, sizeof(T));
        pos_ += sizeof(T);
        return v;
    }
    std::string str() {
        u16 n = get<u16>();
        if (pos_ + n > d_.size()) throw FormatError("replay truncated");
        std::string s(reinterpret_cast<const char*>(d_.data() + pos_), n);
        pos_ += n;
        return s;
    }
    TrackPose pose() {
        TrackPose p;
        p.position.x = get<float>(), p.position.y = get<float>(), p.position.z = get<float>();
        p.orientation.w = get<float>(), p.orientation.x = get<float>(), p.orientation.y = get<float>();
        p.orientation.z = get<float>();
        return p;
    }
    ByteSpan bytes(size_t n) {
        if (pos_ + n > d_.size()) throw FormatError("replay truncated");
        ByteSpan s = d_.subspan(pos_, n);
        pos_ += n;
        return s;
    }
    size_t pos() const { return pos_; }

private:
    ByteSpan d_;
    size_t pos_ = 0;
};

s16 q_steer(float v) { return s16(std::lround(std::clamp(v, -1.0f, 1.0f) * 32767.0f)); }
u16 q_pedal(float v) { return u16(std::lround(std::clamp(v, 0.0f, 1.0f) * 65535.0f)); }

} // namespace

const char* replay_kind_name(ReplayKind k) {
    switch (k) {
    case ReplayKind::TimeTrial: return "Time Trial";
    case ReplayKind::ArcadeRace: return "Arcade";
    case ReplayKind::TwoPlayer: return "2P Battle";
    case ReplayKind::GtRace: return "GT Mode";
    case ReplayKind::License: return "License";
    case ReplayKind::Practice: return "Practice";
    case ReplayKind::Online: return "Online";
    default: return "-";
    }
}

sim::ControlInputs quantize(const sim::ControlInputs& in) {
    sim::ControlInputs q = in;
    q.steer = float(q_steer(in.steer)) / 32767.0f;
    q.throttle = float(q_pedal(in.throttle)) / 65535.0f;
    q.brake = float(q_pedal(in.brake)) / 65535.0f;
    return q;
}

Bytes Replay::serialize() const {
    Writer w;
    w.out.insert(w.out.end(), kMagic, kMagic + 8);
    w.put<u32>(kVersion);
    w.put<u32>(ticks());
    w.put<double>(tick_hz);
    w.str(course);
    w.str(car);
    w.put<u8>(automatic ? 1 : 0);
    w.put<u8>(u8(std::clamp(paint, 0, 255)));
    w.pose(start);
    w.put<u32>(u32(events.size()));
    for (const ReplayEvent& e : events) {
        w.put<u32>(e.tick);
        w.put<u8>(e.kind);
        w.pose(e.pose);
    }
    w.put<double>(best_lap);
    w.put<u32>(best_lap_begin);
    w.put<u32>(best_lap_end);
    w.put<float>(final_position.x), w.put<float>(final_position.y), w.put<float>(final_position.z);
    w.str(event);
    w.put<u16>(u16(std::min<size_t>(opponents.size(), 64)));
    for (size_t i = 0; i < opponents.size() && i < 64; ++i) w.str(opponents[i]);
    w.str(car2);
    w.put<float>(start_speed);
    w.put<u32>(paint_seed);
    w.str(comment);
    w.put<u8>(u8(kind));
    w.put<u16>(u16(std::clamp(laps, 0, 0xFFFF)));
    w.put<double>(total_time);
    w.put<float>(power_ps);
    w.str(car_model);
    w.str(wheel);
    w.put<s64>(saved);
    {
        Bytes sb = spec ? sim::serialize_car_spec(*spec) : Bytes{};
        w.put<u32>(u32(sb.size()));
        w.out.insert(w.out.end(), sb.begin(), sb.end());
    }
    for (size_t i = 0; i < inputs.size(); ++i) {
        const sim::ControlInputs& in = inputs[i];
        u8 dk = i < driver_kind.size() ? driver_kind[i] : 0;
        w.put<s16>(q_steer(in.steer));
        w.put<u16>(q_pedal(in.throttle));
        w.put<u16>(q_pedal(in.brake));
        w.put<u8>(u8((in.handbrake ? 1 : 0) | (in.shift_up ? 2 : 0) | (in.shift_down ? 4 : 0) | (in.automatic ? 8 : 0) | (in.controller_kind == 2 ? 16 : 0) | (in.controller_kind == 3 ? 32 : 0) |
                     (dk == 1 ? 64 : 0) | (dk == 5 ? 128 : 0)));
    }
    if (!car2.empty())
        for (size_t i = 0; i < inputs.size(); ++i) {
            sim::ControlInputs in = i < inputs2.size() ? inputs2[i] : sim::ControlInputs{};
            w.put<s16>(q_steer(in.steer));
            w.put<u16>(q_pedal(in.throttle));
            w.put<u16>(q_pedal(in.brake));
            w.put<u8>(u8((in.handbrake ? 1 : 0) | (in.shift_up ? 2 : 0) | (in.shift_down ? 4 : 0) | (in.automatic ? 8 : 0) | (in.controller_kind == 2 ? 16 : 0) | (in.controller_kind == 3 ? 32 : 0)));
        }
    return std::move(w.out);
}

Replay Replay::parse(ByteSpan data) {
    if (data.size() < 8 || std::memcmp(data.data(), kMagic, 8) != 0) throw FormatError("not a GT3 Revisited replay");
    Reader r(data.subspan(8));
    Replay rp;
    u32 version = r.get<u32>();
    if (version < 1 || version > kVersion) throw FormatError("unsupported replay version " + std::to_string(version));
    u32 n = r.get<u32>();
    rp.tick_hz = r.get<double>();
    if (!(rp.tick_hz > 1 && rp.tick_hz < 10000)) throw FormatError("bad replay tick rate");
    rp.course = r.str();
    rp.car = r.str();
    rp.automatic = r.get<u8>() != 0;
    rp.paint = r.get<u8>();
    rp.start = r.pose();
    u32 ne = r.get<u32>();
    if (ne > n + 1) throw FormatError("bad replay event count");
    for (u32 i = 0; i < ne; ++i) {
        ReplayEvent e;
        e.tick = r.get<u32>();
        e.kind = ReplayEvent::Kind(r.get<u8>());
        e.pose = r.pose();
        rp.events.push_back(e);
    }
    rp.best_lap = r.get<double>();
    rp.best_lap_begin = r.get<u32>();
    rp.best_lap_end = r.get<u32>();
    rp.final_position.x = r.get<float>(), rp.final_position.y = r.get<float>(), rp.final_position.z = r.get<float>();
    if (version >= 2) {
        rp.event = r.str();
        u16 no = r.get<u16>();
        if (no > 64) throw FormatError("bad replay opponent count");
        for (u16 i = 0; i < no; ++i) rp.opponents.push_back(r.str());
    }
    if (version >= 3) rp.car2 = r.str();
    if (version >= 4) rp.start_speed = r.get<float>();
    if (version >= 6) rp.paint_seed = r.get<u32>();
    if (version >= 7) {
        rp.comment = r.str();
        rp.kind = ReplayKind(r.get<u8>());
        rp.laps = r.get<u16>();
        rp.total_time = r.get<double>();
        rp.power_ps = r.get<float>();
        rp.car_model = r.str();
        rp.wheel = r.str();
        rp.saved = r.get<s64>();
        u32 ns = r.get<u32>();
        if (ns > data.size()) throw FormatError("replay truncated");
        if (ns) {
            ByteSpan sb = r.bytes(ns);
            if (std::optional<sim::CarSpec> sp = sim::parse_car_spec(sb))
                rp.spec = std::make_shared<const sim::CarSpec>(std::move(*sp));
            else
                LOG_WARN("Replay: car spec not readable (the database car is used)");
        }
    }
    if (size_t(n) * 7 > data.size()) throw FormatError("replay truncated");
    rp.inputs.reserve(n);
    for (u32 i = 0; i < n; ++i) {
        sim::ControlInputs in;
        in.steer = float(r.get<s16>()) / 32767.0f;
        in.throttle = float(r.get<u16>()) / 65535.0f;
        in.brake = float(r.get<u16>()) / 65535.0f;
        u8 f = r.get<u8>();
        in.handbrake = f & 1, in.shift_up = f & 2, in.shift_down = f & 4, in.automatic = f & 8;
            in.controller_kind = (f & 16) ? 2 : (f & 32) ? 3 : 0;
        rp.inputs.push_back(in);
        rp.driver_kind.push_back(u8((f & 64) ? 1 : (f & 128) ? 5 : 0));
    }
    if (!rp.car2.empty())
        for (u32 i = 0; i < n; ++i) {
            sim::ControlInputs in;
            in.steer = float(r.get<s16>()) / 32767.0f;
            in.throttle = float(r.get<u16>()) / 65535.0f;
            in.brake = float(r.get<u16>()) / 65535.0f;
            u8 f = r.get<u8>();
            in.handbrake = f & 1, in.shift_up = f & 2, in.shift_down = f & 4, in.automatic = f & 8;
            in.controller_kind = (f & 16) ? 2 : (f & 32) ? 3 : 0;
            rp.inputs2.push_back(in);
        }
    return rp;
}

void ReplayRecorder::begin(std::string course, std::string car, bool automatic, int paint, const TrackPose& start,
                           double hz, std::string event, std::vector<std::string> opponents) {
    replay_ = {};
    replay_.course = std::move(course);
    replay_.car = std::move(car);
    replay_.automatic = automatic;
    replay_.paint = paint;
    replay_.start = start;
    replay_.tick_hz = hz;
    replay_.event = std::move(event);
    replay_.opponents = std::move(opponents);
    active_ = true;
}

sim::ControlInputs ReplayRecorder::record(const sim::ControlInputs& in) {
    sim::ControlInputs q = quantize(in);
    if (active_) replay_.inputs.push_back(q);
    return q;
}

void ReplayRecorder::set_driver_kind(int kind) {
    if (!active_ || replay_.inputs.empty()) return;
    replay_.driver_kind.resize(replay_.inputs.size(), 0);
    replay_.driver_kind.back() = u8(kind == 1 || kind == 5 ? kind : 0);
}

sim::ControlInputs ReplayRecorder::record_second(const sim::ControlInputs& in) {
    sim::ControlInputs q = quantize(in);
    if (active_) replay_.inputs2.push_back(q);
    return q;
}

void ReplayRecorder::respawn_second(const TrackPose& pose) {
    if (active_) replay_.events.push_back({tick(), ReplayEvent::Respawn2, pose});
}

void ReplayRecorder::respawn(const TrackPose& pose) {
    if (active_) replay_.events.push_back({tick(), ReplayEvent::Respawn, pose});
}

void ReplayRecorder::lap(double time, u32 begin_tick, u32 end_tick) {
    if (!active_ || time <= 0) return;
    if (replay_.best_lap < 0 || time < replay_.best_lap) {
        replay_.best_lap = time;
        replay_.best_lap_begin = begin_tick;
        replay_.best_lap_end = end_tick;
    }
}

const ReplayEvent* ReplayPlayer::pending_event() {
    if (!r_ || event_ >= r_->events.size() || r_->events[event_].tick > tick_) return nullptr;
    return &r_->events[event_++];
}

sim::ControlInputs ReplayPlayer::second_inputs() const {
    if (!r_ || tick_ == 0 || tick_ - 1 >= r_->inputs2.size()) return {};
    return r_->inputs2[tick_ - 1];
}

int ReplayPlayer::driver_kind() const {
    if (!r_ || tick_ == 0 || tick_ - 1 >= r_->driver_kind.size()) return 0;
    return r_->driver_kind[tick_ - 1];
}

sim::ControlInputs ReplayPlayer::next_inputs() {
    if (done()) return {};
    return r_->inputs[tick_++];
}

std::optional<std::string> save_replay(const Replay& r, const std::string& dir) {
    SDL_CreateDirectory(dir.c_str());
    std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&now));
    const std::string base = dir + (dir.empty() || dir.back() == '/' ? "" : "/") + r.course + "_" + r.car + "_";
    std::string path = base + stamp + ".gt3replay";
    // Two saves in the same second: the next free second's stamp (names stay sortable).
    SDL_PathInfo info;
    for (int i = 1; i < 60 && SDL_GetPathInfo(path.c_str(), &info); ++i) {
        std::time_t t = now + i;
        std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&t));
        path = base + stamp + ".gt3replay";
    }
    Replay copy = r;
    if (copy.saved == 0) copy.saved = s64(now);
    Bytes b = copy.serialize();
    if (!write_host_file(path, b)) return std::nullopt;
    LOG_INFO("Replay saved: %s (%u ticks, %.1f s)", path.c_str(), r.ticks(), r.duration());
    return path;
}

std::vector<ReplayFile> list_replays(const std::string& dir) {
    std::vector<ReplayFile> out;
    int count = 0;
    char** names = SDL_GlobDirectory(dir.c_str(), "*.gt3replay", 0, &count);
    if (!names) return out;
    for (int i = 0; i < count; ++i) {
        std::string n = names[i];
        out.push_back({dir + (dir.empty() || dir.back() == '/' ? "" : "/") + n, n});
    }
    SDL_free(names);
    // File names end in a timestamp: newest first by name order within a course/car, then overall by stamp.
    auto stamp = [](const std::string& n) { return n.size() > 25 ? n.substr(n.size() - 25) : n; };
    std::sort(out.begin(), out.end(), [&](const ReplayFile& a, const ReplayFile& b) { return stamp(a.name) > stamp(b.name); });
    return out;
}

} // namespace gt3r::race
