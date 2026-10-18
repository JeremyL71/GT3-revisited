#include "game/net_car.h"

#include "game/data/car_database.h"
#include "game/gt_mode.h"
#include "game/gt_parts.h"
#include "game/gt_state.h"
#include "sim/models.h"
#include "base/ps2_float.h"
#include "net/protocol.h"

#include <algorithm>
#include <bit>
#include <cmath>

namespace gt3r::game {

namespace {

constexpr u8 kVersion = 1;

// Every CarSettings value in a fixed order (encode / decode walk the same list).
template <class F>
void each_setting(sim::CarSettings& s, F&& f) {
    for (int& v : s.gears) f(v);
    f(s.final_drive), f(s.gear_slider), f(s.front_torque), f(s.brake_front), f(s.brake_rear);
    f(s.downforce_front), f(s.downforce_rear);
    for (int& v : s.turbo) f(v);
    for (int& v : s.camber) f(v);
    for (int& v : s.ride_height) f(v);
    for (int& v : s.toe) f(v);
    for (int& v : s.spring) f(v);
    for (int& v : s.motion_ratio) f(v);
    for (int& v : s.damper) f(v);
    for (int& v : s.arb) f(v);
    for (int& v : s.lsd) f(v);
    f(s.tcs);
    for (int& v : s.ascc) f(v);
}

std::string ascii(const std::string& s) {
    std::string out;
    for (unsigned char c : s)
        if (c >= 0x20 && c < 0x7F) out += char(c);
        else if ((c & 0xC0) == 0xC0) out += '-';
    return out;
}

} // namespace

Bytes encode_online_car(const OnlineCar& c) {
    net::Writer w;
    w.u8_(kVersion);
    w.u8_(u8(c.kind));
    w.str(c.car);
    w.str(c.name);
    w.u8_(c.paint);
    if (c.kind == OnlineCar::Kind::Garage) {
        for (u64 p : c.parts) w.u32_(u32(p)), w.u32_(u32(p >> 32));
        sim::CarSettings s = c.settings;
        each_setting(s, [&](int& v) { w.u32_(u32(s32(v))); });
        w.f32_(c.engine_wear_m);
        w.f32_(c.oil_m);
        w.str(c.wheel);
    } else if (c.kind == OnlineCar::Kind::Mod) {
        w.str(c.package);
        w.u32_(c.package_size);
    }
    return w.data;
}

std::optional<OnlineCar> decode_online_car(ByteSpan data) {
    if (data.size() > kMaxOnlineCarBytes) return std::nullopt;
    net::Reader r(data);
    if (r.u8_() != kVersion) return std::nullopt;
    OnlineCar c;
    u8 kind = r.u8_();
    if (kind > u8(OnlineCar::Kind::Mod)) return std::nullopt;
    c.kind = OnlineCar::Kind(kind);
    c.car = r.str();
    c.name = ascii(r.str());
    c.paint = r.u8_();
    if (c.kind == OnlineCar::Kind::Garage) {
        for (u64& p : c.parts) {
            u64 lo = r.u32_(), hi = r.u32_();
            p = lo | hi << 32;
        }
        each_setting(c.settings, [&](int& v) { v = int(s32(r.u32_())); });
        c.engine_wear_m = r.f32_();
        c.oil_m = r.f32_();
        c.wheel = r.str();
    } else if (c.kind == OnlineCar::Kind::Mod) {
        c.package = r.str();
        c.package_size = r.u32_();
        if (c.package.size() != 64 || !std::all_of(c.package.begin(), c.package.end(), [](char ch) {
                return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
            }))
            return std::nullopt;
    }
    if (!r.ok() || c.car.empty()) return std::nullopt;
    return c;
}

OnlineCar online_car_from_db(const data::CarDatabase& db, const std::string& label, u8 paint) {
    OnlineCar c;
    c.car = label;
    c.paint = paint;
    c.kind = db.mod_car(label) ? OnlineCar::Kind::Mod : OnlineCar::Kind::Disc;
    if (const data::Car* dc = db.car(label)) c.name = ascii(dc->name.empty() ? label : dc->name);
    return c;
}

OnlineCar online_car_from_garage(const data::CarDatabase& db, const GarageCar& g) {
    OnlineCar c = online_car_from_db(db, g.car, u8(std::clamp(g.color, 0, 255)));
    c.kind = OnlineCar::Kind::Garage;
    c.parts = g.parts;
    c.settings = g.settings;
    c.engine_wear_m = g.cond.engine_wear_m;
    c.oil_m = g.cond.oil_m;
    c.wheel = g.wheel;
    return c;
}

std::optional<sim::CarSpec> online_car_spec(const data::CarDatabase& db, const OnlineCar& c, std::string* error) {
    if (!db.car(c.car)) {
        if (error) *error = "no car " + c.car + " here";
        return std::nullopt;
    }
    if (c.kind != OnlineCar::Kind::Garage) return sim::build_car_spec(db, c.car, error);
    std::optional<sim::CarSpec> s = sim::build_car_spec(db, c.car, c.parts, c.settings, error);
    if (!s) return s;
    CarCondition cond;
    cond.engine_wear_m = std::isfinite(c.engine_wear_m) ? c.engine_wear_m : 0;
    cond.oil_m = std::isfinite(c.oil_m) ? c.oil_m : 0;
    float f = GtMode::engine_factor(cond);
    for (float& t : s->engine.curve_torque) t *= f;
    return s;
}

PowerPeak spec_power_peak(const sim::CarSpec& s) {
    const sim::EngineSpec& e = s.engine;
    const size_t n = std::min(e.curve_rpm.size(), e.curve_torque.size());
    PowerPeak best;
    if (n == 0) return best;
    // Sample from the first curve point (rpm rounded to 100) up to max(rev limit, last
    // point); only rpm up to the rev limit can set the peak; strictly greater wins.
    const int limit = int(e.rev_limit_rpm);
    const int first = (int(e.curve_rpm[0]) + 50) / 100 * 100;
    const int last = std::max(limit, (int(e.curve_rpm[n - 1]) + 50) / 100 * 100);
    // The original's curve axis is in rad/s; look up in the same domain so float ties on a
    // flat power plateau resolve the same way.
    std::vector<float> w(n);
    for (size_t i = 0; i < n; ++i) w[i] = ps2::mul(e.curve_rpm[i], sim::model::kRpmToRadS);
    for (int rpm = first; rpm <= last; rpm += 100) {
        if (rpm > limit) continue;
        float omega = ps2::mul(float(rpm), sim::model::kRpmToRadS);
        float kgfm = sim::model::table_lookup(w.data(), e.curve_torque.data(), int(n), omega) / 9.8f;
        float ps = ps2::mul(kgfm, float(rpm)) / 716.2f; // PS2 multiply (rounds towards zero)
        if (ps > best.ps) {
            best.ps = ps;
            best.rpm = rpm;
        }
    }
    return best;
}

float spec_power_ps(const sim::CarSpec& s) { return spec_power_peak(s).ps; }

int display_power(float ps, bool imperial) { return ps2::to_int(imperial ? ps2::mul(ps, std::bit_cast<float>(0x3F7C76F3u)) : ps); }

const char* power_unit(bool imperial) { return imperial ? "HP" : "PS"; }

float geared_top_speed(const sim::CarSpec& s) {
    float r = 0;
    for (const sim::AxleSpec& a : s.axle) r = std::max(r, a.tire_radius);
    float ratio = 1e9f;
    for (size_t g = 1; g <= s.gearbox.ratios.size(); ++g)
        ratio = std::min(ratio, std::fabs(sim::model::total_ratio(s.gearbox, int(g))));
    float rpm = s.engine.rev_limit_rpm;
    if (!(ratio > 0 && ratio < 1e8f) || r <= 0 || rpm <= 0) return 150.0f; // unknown gearing: a generous bound
    return rpm * 0.10471976f / ratio * r;
}

bool car_meets_rules(const OnlineCar& c, const sim::CarSpec* spec, bool stock_only, int max_ps, std::string* why) {
    if (stock_only && c.kind == OnlineCar::Kind::Garage) {
        if (why) *why = "tuned garage cars are not allowed (stock cars only)";
        return false;
    }
    if (max_ps > 0 && spec) {
        float ps = spec_power_ps(*spec);
        if (ps > float(max_ps) + 0.5f) {
            if (why) *why = std::to_string(int(ps + 0.5f)) + " PS is over the " + std::to_string(max_ps) + " PS limit";
            return false;
        }
    }
    return true;
}

bool online_car_racing_gearbox(const data::CarDatabase& db, const OnlineCar& c) {
    if (c.kind != OnlineCar::Kind::Garage) return false;
    const ByteSpan* gear = db.record(data::TableId::Gear, c.parts[size_t(part_slot(17))]);
    return gear && gear->size() > 0x10 && (*gear)[0x10] != 0;
}

} // namespace gt3r::game
