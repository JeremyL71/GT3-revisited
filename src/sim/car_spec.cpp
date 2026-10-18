#include "sim/car_spec.h"

#include "game/data/car_database.h"
#include "sim/models.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::sim {

namespace {

// Slot indices into Car::parts (see data::kCarPartTables).
enum Slot : int {
    kBrake = 0, kSteer = 2, kChassis = 3, kLightweight = 4, kBody = 5, kEngine = 6, kPortPolish = 7,
    kEngineBalance = 8, kDisplacement = 9, kComputer = 10, kNaTune = 11, kTurbo = 12, kDrivetrain = 13,
    kFlywheel = 14, kClutch = 15, kPropShaft = 16, kLsd = 17, kGear = 18, kSuspension = 19, kIntercooler = 20, kMuffler = 21, kFrontTire = 22, kRearTire = 23,
};

const ByteSpan* part_record(const data::CarDatabase& db, const std::array<u64, 27>& parts, int slot) {
    u64 id = parts[size_t(slot)];
    if (id == 0) return nullptr;
    return db.record(data::TableId(data::kCarPartTables[size_t(slot)]), id);
}

u8 byte_at(const ByteSpan* r, size_t off, u8 fallback = 0) { return r && r->size() > off ? (*r)[off] : fallback; }

// Geometric gear set for gearboxes that store "computed" ratios (-0.999).
std::vector<float> fill_ratios(std::vector<float> r, int gears) {
    if (gears == 1 && !r.empty() && r[0] > 0) return {r[0]}; // single speed (mod EVs; no disc box has < 4)
    gears = std::clamp(gears, 3, 7);
    r.resize(size_t(gears), -1.0f);
    float first = r[0] > 0 ? r[0] : 3.2f;
    float top = 0.80f;
    for (int i = gears - 1; i >= 0; --i)
        if (r[size_t(i)] > 0 && i > 0) {
            top = r[size_t(i)] * std::pow(0.85f, float(gears - 1 - i));
            break;
        }
    for (int i = 0; i < gears; ++i)
        if (r[size_t(i)] <= 0) r[size_t(i)] = first * std::pow(top / first, float(i) / float(gears - 1));
    return r;
}

} // namespace

void TireSpec::default_curves() {
    // [placeholder] shapes; replaced by the database curves in build_car_spec.
    width = 0;
    load = {{0.0f, 1.0f, 2.0f, 3.0f}, {1.12f, 1.0f, 0.88f, 0.78f}};
    slip_a = {{0.0f, 0.04f, 0.08f, 0.14f, 0.25f, 0.5f, 1.5708f}, {0.0f, 0.55f, 0.88f, 1.0f, 0.95f, 0.85f, 0.75f}};
    slip_b = {{0.0f, 0.1f, 0.3f, 1.5708f}, {0.0f, 0.02f, 0.15f, 0.40f}};
    limit_d = {{-1.0f, -0.2f, -0.1f, -0.05f, 0.0f, 0.05f, 0.1f, 0.2f, 1.0f},
               {0.35f, 0.45f, 0.6f, 0.85f, 1.0f, 0.85f, 0.6f, 0.45f, 0.35f}};
    long_c = {{-1.0f, -0.5f, -0.2f, -0.10f, -0.06f, -0.03f, 0.0f, 0.03f, 0.06f, 0.10f, 0.2f, 0.5f, 1.0f},
              {0.75f, 0.82f, 0.93f, 1.0f, 0.9f, 0.6f, 0.0f, 0.6f, 0.9f, 1.0f, 0.93f, 0.82f, 0.75f}};
    opt_drive = -0.10f;
    opt_brake = 0.10f;
}

namespace {

// [spec §6.5] Spec tyre block (0x70 bytes) from a compound record c (table 27)
// and force-vol record f (table 26), before defaults/zeroing.
std::array<u8, 0x70> tire_block(const ByteSpan* c, const ByteSpan* f) {
    std::array<u8, 0x70> B{};
    auto cb = [&](size_t off) -> u8 { return c && c->size() > off ? (*c)[off] : 0; };
    auto fb = [&](size_t off) -> u8 { return f && f->size() > off ? (*f)[off] : 0; };
    B[0x00] = cb(0x08) ? cb(0x08) : 100;
    B[0x01] = cb(0x09) ? cb(0x09) : 0x2D;
    B[0x02] = 8;
    for (int i = 0; i < 8; ++i) {
        B[size_t(0x03 + i)] = cb(size_t(0x12 + i));
        B[size_t(0x0B + i)] = cb(size_t(0x1A + i));
    }
    B[0x13] = 6;
    B[0x26] = 6;
    for (int i = 0; i < 6; ++i) {
        B[size_t(0x14 + i)] = cb(size_t(0x22 + i));
        B[size_t(0x1A + i)] = cb(size_t(0x28 + i));
        B[size_t(0x20 + i)] = cb(size_t(0x2E + i));
        B[size_t(0x27 + i)] = cb(size_t(0x34 + i));
        B[size_t(0x2D + i)] = cb(size_t(0x3A + i));
        B[size_t(0x33 + i)] = cb(size_t(0x40 + i));
        B[size_t(0x5F + i)] = cb(size_t(0x46 + i));
        B[size_t(0x65 + i)] = cb(size_t(0x4C + i));
    }
    B[0x39] = 4;
    for (int i = 0; i < 4; ++i) {
        B[size_t(0x3A + i)] = cb(size_t(0x0A + i));
        B[size_t(0x3E + i)] = cb(size_t(0x0E + i));
    }
    // force-vol +0x08..+0x11 -> surfaces 0..9, +0x12..+0x14 -> surfaces 12..14 (10, 11, 15 = 0)
    for (int sidx = 0; sidx < 10; ++sidx) B[size_t(0x4B + sidx)] = fb(size_t(0x08 + sidx));
    for (int k = 0; k < 3; ++k) B[size_t(0x4B + 12 + k)] = fb(size_t(0x12 + k));
    B[0x5E] = 6;
    B[0x6B] = cb(0x52);
    return B;
}

// [spec §6.5] Runtime record from a finished block (after defaults/zeroing).
TireSpec tire_record(std::array<u8, 0x70> B, float width) {
    const float kPi = 3.14159265f;
    // The game zeroes the first point of each curve.
    for (size_t o : {0x03, 0x0B, 0x14, 0x1A, 0x27, 0x2D, 0x3A, 0x5F, 0x65}) B[o] = 0;
    B[0x20] = B[0x33];
    TireSpec t;
    t.width = width;
    t.mu = B[0x00] * 0.01f;
    t.wear_rate = B[0x6B] * 0.01f; // [spec §8.2] compound wear rate
    for (int s = 0; s < 16; ++s) {
        u8 pct = B[size_t(0x4B + s)];
        t.surface_mu[size_t(s)] = pct ? t.mu * pct * 0.01f : t.mu;
    }
    t.surface_mu[11] = std::min(1.0f, 1.25f / t.mu);
    float th = std::min((B[0x01] ? B[0x01] : 90) * kPi / 180.0f, kPi / 2);
    for (int i = 0; i < 8; ++i) {
        t.slip_a.x.push_back(B[size_t(0x03 + i)] * th / 255.0f);
        t.slip_a.y.push_back(B[size_t(0x0B + i)] / 200.0f);
    }
    bool b_empty = true;
    for (int j = 1; j < 6; ++j) b_empty = b_empty && B[size_t(0x5F + j)] == 0;
    if (b_empty) {
        TireSpec d;
        d.default_curves();
        t.slip_b = d.slip_b; // [placeholder] the static default curve bytes are not in the spec
    } else {
        for (int j = 0; j < 6; ++j) {
            t.slip_b.x.push_back(B[size_t(0x5F + j)] * (kPi / 2) / 255.0f);
            t.slip_b.y.push_back(B[size_t(0x65 + j)] / 200.0f);
        }
    }
    for (int i = 5; i >= 0; --i) {
        t.long_c.x.push_back(-B[size_t(0x14 + i)] / 255.0f);
        t.long_c.y.push_back(B[size_t(0x1A + i)] / 200.0f);
        t.limit_d.y.push_back(B[size_t(0x20 + i)] / 200.0f);
    }
    for (int j = 1; j < 6; ++j) {
        t.long_c.x.push_back(B[size_t(0x27 + j)] / 255.0f);
        t.long_c.y.push_back(B[size_t(0x2D + j)] / 200.0f);
        t.limit_d.y.push_back(B[size_t(0x33 + j)] / 200.0f);
    }
    t.limit_d.x = t.long_c.x;
    // Optimum slips: first maximum of C going outwards from 0 on each side.
    auto first_max = [&](int from, int step, int end) {
        int best = from;
        for (int i = from + step; i != end; i += step) {
            if (t.long_c.y[size_t(i)] > t.long_c.y[size_t(best)]) best = i;
            else break;
        }
        return t.long_c.x[size_t(best)];
    };
    t.opt_drive = first_max(5, -1, -1);
    t.opt_brake = first_max(5, 1, 11);
    for (int j = 0; j < 4; ++j) {
        t.load.x.push_back(B[size_t(0x3A + j)] * 196.0f);
        t.load.y.push_back(B[size_t(0x3E + j)] / 200.0f);
    }
    return t;
}

} // namespace

float CarSpec::peak_torque() const {
    float m = 0;
    for (float t : engine.curve_torque) m = std::max(m, t);
    return m;
}

float CarSpec::peak_power_rpm() const {
    float best = 0, best_rpm = engine.rev_limit_rpm;
    for (float rpm = engine.idle_rpm; rpm <= engine.rev_limit_rpm; rpm += 50) {
        float p = model::curve_torque(engine, rpm) * rpm;
        if (p > best) {
            best = p;
            best_rpm = rpm;
        }
    }
    return best_rpm;
}

float CarSpec::peak_power_kw() const {
    float rpm = peak_power_rpm();
    return model::curve_torque(engine, rpm) * rpm * model::kRpmToRadS * 0.001f;
}

float showroom_top_speed(const CarSpec& c) {
    const int n = int(c.gearbox.ratios.size());
    if (n == 0 || c.engine.curve_rpm.empty()) return 0;
    float r = 0;
    int driven_axles = 0;
    for (int a = 0; a < 2; ++a)
        if (c.drivetrain.drive == Drive::AWD || (a == 0) == (c.drivetrain.drive == Drive::FF)) {
            r += c.axle[size_t(a)].tire_radius;
            ++driven_axles;
        }
    r = std::max(r / float(std::max(driven_axles, 1)), 0.05f);
    auto per_metre = [&](int gear) { return model::total_ratio(c.gearbox, gear) / r; }; // engine rad/s per m/s
    // The start value: the rev limit in top gear.
    float top = c.engine.rev_limit_rpm * model::kRpmToRadS / per_metre(n);
    const int kmh_max = int(top * 3.6f), step = kmh_max / 500 + 1;
    // The aero coefficients as built (air density 1.2; the showroom's global holds 0.8
    // until a race has been set up, but the reference screens match 1.2).
    const float drag = c.aero.c_drag, down = c.aero.c_front + c.aero.c_rear;
    const float weight = c.mass * 9.8f, rr = model::surface_rolling(0);
    const float up = c.engine.upshift_rpm > 0 ? c.engine.upshift_rpm : c.engine.rev_limit_rpm - 500;
    int gear = 1;
    for (int kmh = 0; kmh < kmh_max; kmh += step) {
        const float v = float(kmh) / 3.6f, w = v * per_metre(gear);
        const float drive = model::curve_torque(c.engine, w / model::kRpmToRadS) * per_metre(gear);
        const float resist = model::rolling_curve(v) * rr * (weight + down * v * v) + drag * v * v;
        if (resist < drive) top = v;
        if (gear < n && up < float(int(w * 60.0f / 6.2831855f))) ++gear;
    }
    return top;
}

void CarSpec::finalize() {
    float g = model::kGravity;
    mass = std::max(mass, 300.0f);
    front_weight = std::clamp(front_weight, 0.3f, 0.7f);
    float share[2] = {front_weight, 1 - front_weight};
    float total_brake = brake_decel * mass * g; // N at the ground
    for (int a = 0; a < 2; ++a) {
        AxleSpec& ax = axle[size_t(a)];
        float corner = mass * share[a] * 0.5f;
        if (ax.spring_rate <= 0) {
            float w = 2 * 3.14159265f * ax.ride_frequency;
            ax.spring_rate = corner * w * w;
        }
        if (ax.damper <= 0) ax.damper = 2 * ax.damping_ratio * std::sqrt(ax.spring_rate * corner);
        if (ax.arb_rate <= 0) ax.arb_rate = ax.spring_rate * (a == 0 ? 0.9f : 0.5f);
        float sag = corner * g / ax.spring_rate;
        ax.rest_length = std::max(ax.rest_length, sag + ax.travel + 0.03f);
        if (ax.max_brake_torque <= 0) {
            float bal = a == 0 ? brake_balance : 1 - brake_balance;
            ax.max_brake_torque = total_brake * bal * 0.5f * ax.tire_radius;
        }
        if (ax.wheel_inertia <= 0) ax.wheel_inertia = 1.2f;
        TireSpec& t = tire[size_t(a)];
        if (t.slip_a.x.empty()) t.default_curves();
        t.fz_ref = corner * g;
        for (TireSpec& r : tire_slots[size_t(a)]) {
            if (r.slip_a.x.empty()) r = t;
            r.fz_ref = corner * g;
        }
    }
    if (aids.handbrake_torque <= 0) aids.handbrake_torque = 1.5f * axle[1].max_brake_torque;
    if (gearbox.ratios.empty()) gearbox.ratios = {3.2f, 2.0f, 1.4f, 1.05f, 0.82f};
    if (gearbox.final_drive <= 0) gearbox.final_drive = 4.0f;
    if (gearbox.reverse <= 0) gearbox.reverse = 3.3f;
    if (engine.curve_rpm.empty()) {
        engine.curve_rpm = {1000, 3000, 5000, 7000};
        engine.curve_torque = {150, 220, 240, 200};
    }
    engine.idle_rpm = engine.electric ? 0.0f : std::clamp(engine.idle_rpm, 500.0f, 2500.0f);
    if (engine.rev_limit_rpm < engine.idle_rpm + 1500) engine.rev_limit_rpm = engine.curve_rpm.back();
    float peak = peak_torque();
    if (drivetrain.clutch_max_torque <= 0) drivetrain.clutch_max_torque = 1.6f * peak;
    if (drivetrain.launch_rpm <= 0 && !engine.electric)
        drivetrain.launch_rpm = std::min(0.5f * (engine.idle_rpm + peak_power_rpm()), engine.rev_limit_rpm * 0.7f);
    for (AxleSpec& ax : axle) ax.toe = model::toe_angle(ax.toe_byte, ax.tire_radius);
    model::ackermann_ratios(wheelbase, 0.5f * axle[0].track, steering.ack_inner, steering.ack_outer);
    if (front_extent <= 0.3f) front_extent = length * 0.5f;
    if (rear_extent <= 0.3f) rear_extent = length * 0.5f;
    if (cg_height <= 0) cg_height = std::max(0.35f, height * 0.37f);
    if (aero.c_drag <= 0) aero.c_drag = 0.5f * aero.air_density * aero.drag_area;
    if (yaw_inertia <= 0) yaw_inertia = 6000; // [spec §6.4] default
}

CarSpec CarSpec::test_car(Drive drive) {
    CarSpec s;
    s.label = "test_car";
    s.name = "Test car (R34 GT-R-like)";
    s.mass = 1680; // 1560 kg + 120 kg [spec §6.4]
    s.front_weight = 0.55f;
    s.wheelbase = 2.665f;
    s.length = 4.6f;
    s.width = 1.785f;
    s.height = 1.36f;
    s.cg_height = 0.50f;
    s.power_ps = 280;
    for (int a = 0; a < 2; ++a) {
        s.axle[size_t(a)].track = a == 0 ? 1.48f : 1.49f;
        s.axle[size_t(a)].tire_width = 0.245f;
        s.axle[size_t(a)].tire_radius = 0.0254f * 18 * 0.5f + 0.245f * 0.40f;
        s.tire[size_t(a)].mu = 1.0f;
        s.tire[size_t(a)].compound = "normal";
    }
    // Roughly an RB26DETT as the game rates it (~330 PS at the crank).
    s.engine.curve_rpm = {1000, 2000, 3000, 4000, 4400, 5000, 6000, 6800, 7500, 8000};
    s.engine.curve_torque = {170, 250, 330, 385, 392, 380, 360, 345, 300, 260};
    s.engine.idle_rpm = 1000;
    s.engine.rev_limit_rpm = 8000;
    s.yaw_inertia = 9500;
    s.steering.max_angle = 0.61f;
    s.gearbox.ratios = {3.827f, 2.360f, 1.685f, 1.312f, 1.000f, 0.793f};
    s.gearbox.final_drive = 3.545f;
    s.gearbox.reverse = 3.28f;
    s.drivetrain.drive = drive;
    s.drivetrain.front_share = drive == Drive::AWD ? 0.35f : 0;
    if (drive == Drive::AWD) { // [placeholder] a 50:50 coupled AWD (spec §2.7 system 1, L3)
        s.drivetrain.awd_system = 1;
        s.drivetrain.awd_share_byte = 50;
    }
    s.aero.drag_area = 0.34f * s.width * s.height * 0.85f;
    s.finalize();
    return s;
}

std::optional<CarSpec> build_car_spec(const data::CarDatabase& db, const std::string& label, std::string* error) {
    if (const data::Car* c = db.car(label)) return build_car_spec(db, c->label, c->parts, error);
    const data::CarVariant* v = db.variant(label);
    if (!v)
        for (const auto& a : db.arcade_cars())
            if (a.label == label) v = &a;
    if (v) {
        const data::Car* base = db.car(v->car);
        if (!base) {
            if (error) *error = "variant " + label + ": base car " + v->car + " not found";
            return std::nullopt;
        }
        std::array<u64, 27> parts = v->parts;
        for (size_t i = 0; i < parts.size(); ++i)
            if (parts[i] == 0) parts[i] = base->parts[i];
        auto s = build_car_spec(db, base->label, parts, error);
        if (s) s->label = label;
        return s;
    }
    if (error) *error = "unknown car label " + label;
    return std::nullopt;
}

namespace {
int interp_setting(int steps, int mn, int mx, int set) { // [spec §6.1]
    return steps < 2 ? mn : mn + (mx - mn) * (set - 1) / (steps - 1);
}
} // namespace

bool apply_gear_slider(const data::CarDatabase& db, const std::array<u64, 27>& parts, CarSettings& c) {
    const ByteSpan* g = part_record(db, parts, kGear);
    const ByteSpan* er = part_record(db, parts, kEngine);
    if (!g || g->size() <= 0x29 || !(*g)[0x28] || !er || er->size() < 0x58) return false;
    // Rev byte = engine +0x45 + engine-balance / NA-tune / turbo adders.
    int rev = (*er)[0x45];
    for (int slot : {kEngineBalance, kNaTune}) rev += byte_at(part_record(db, parts, slot), 0x12);
    rev += byte_at(part_record(db, parts, kTurbo), 0x18);
    auto dt = db.drivetrain(parts[kDrivetrain]);
    int drive = dt ? dt->drive_type : 0, sys = dt ? dt->awd_system : 0xFE;
    int axle = (drive == 1 || (drive == 2 && sys == 0)) ? 0 : 1;
    int rim = 0, wc = 0, ac = 0;
    if (auto tr = db.tire(axle ? data::TableId::RearTire : data::TableId::FrontTire, parts[size_t(axle ? kRearTire : kFrontTire)]))
        if (const ByteSpan* sz = db.record(data::TableId::TireSize, tr->size); sz && sz->size() >= 0x0B) {
            rim = (*sz)[0x08];
            ac = (*sz)[0x09];
            wc = (*sz)[0x0A];
        }
    int n = (*g)[0x11];
    s16 out[8] = {};
    model::generate_gear_ratios(n, rev, c.final_drive, rim, wc, ac, c.gear_slider, out, drive, sys);
    for (int i = 0; i <= std::min(n, 7); ++i) c.gears[size_t(i)] = out[i];
    return true;
}

CarSettings stock_settings(const data::CarDatabase& db, const std::string& car_label, const std::array<u64, 27>& parts) {
    (void)car_label;
    CarSettings c;
    auto rec = [&](int slot) { return part_record(db, parts, slot); };
    if (const ByteSpan* g = rec(kGear); g && g->size() > 0x2B) {
        c.gears[0] = int(s16(rd16(*g, 0x12)));
        for (int i = 0; i < 7; ++i) c.gears[size_t(i + 1)] = int(s16(rd16(*g, size_t(0x14 + 2 * i))));
        c.final_drive = rd16(*g, 0x22);
        c.gear_slider = (*g)[0x29];
        if ((*g)[0x28]) apply_gear_slider(db, parts, c); // [spec §6.7] computed gearbox: generated default words
    }
    if (const ByteSpan* d = rec(kDrivetrain); d && d->size() > 0x13) c.front_torque = (*d)[0x13];
    if (const ByteSpan* bc = rec(1); bc && bc->size() > 0x14) c.brake_front = c.brake_rear = (*bc)[0x14];
    if (const ByteSpan* b = rec(kBody); b && b->size() > 0x3C) {
        c.downforce_front = (*b)[0x13];
        c.downforce_rear = (*b)[0x3C];
    }
    if (const ByteSpan* t = rec(kTurbo); t && t->size() > 0x17)
        for (int i = 0; i < 6; ++i) c.turbo[size_t(i)] = (*t)[size_t(0x12 + i)];
    if (const ByteSpan* su = rec(kSuspension); su && su->size() >= 0x7C) {
        auto B = [&](size_t o) -> int { return (*su)[o]; };
        c.camber = {B(0x13), B(0x16)};
        c.ride_height = {rd16(*su, 0x54), rd16(*su, 0x5A)};
        c.toe = {B(0x19), B(0x1C)};
        c.spring = {B(0x23), B(0x26)};
        c.motion_ratio = {B(0x27), B(0x28)};
        c.damper = {B(0x2E), B(0x31), B(0x35), B(0x38), B(0x3C), B(0x3F), B(0x43), B(0x46)};
        c.arb = {B(0x4A), B(0x4E)};
    }
    if (const ByteSpan* ls = rec(kLsd); ls && ls->size() > 0x22)
        c.lsd = {(*ls)[0x12], (*ls)[0x1C], (*ls)[0x15], (*ls)[0x1F], (*ls)[0x18], (*ls)[0x22]};
    if (const ByteSpan* tc = db.record(data::TableId::Tcsc, parts[25]); tc && tc->size() > 0x11) c.tcs = (*tc)[0x11];
    if (const ByteSpan* as = db.record(data::TableId::Ascc, parts[24]); as && as->size() > 0x26)
        c.ascc = {(*as)[0x12], (*as)[0x1B], (*as)[0x21], (*as)[0x26]};
    return c;
}

std::optional<CarSpec> build_car_spec(const data::CarDatabase& db, const std::string& car_label,
                                      const std::array<u64, 27>& parts, std::string* error) {
    return build_car_spec(db, car_label, parts, stock_settings(db, car_label, parts), error);
}

std::optional<CarSpec> build_car_spec(const data::CarDatabase& db, const std::string& car_label,
                                      const std::array<u64, 27>& parts, const CarSettings& set, std::string* error) {
    auto fail = [&](const std::string& msg) -> std::optional<CarSpec> {
        if (error) *error = car_label + ": " + msg;
        return std::nullopt;
    };
    const data::Car* car = db.car(car_label);
    if (!car) return fail("not in the car table");

    CarSpec s;
    s.label = car_label;
    s.name = car->name;

    // --- chassis / body ---
    auto ch = db.chassis(parts[kChassis]);
    auto body = db.body(parts[kBody]);
    if (!ch || ch->weight_kg == 0 || ch->wheelbase_mm == 0) return fail("missing chassis record");
    // [spec §6.2/6.4] weight x lightweight(+0x12 per mille) x body(+0x30 per mille), integer maths, + 120 kg.
    int weight = ch->weight_kg;
    const ByteSpan* lw = part_record(db, parts, kLightweight);
    if (lw && lw->size() >= 0x14 && rd16(*lw, 0x12)) weight = weight * rd16(*lw, 0x12) / 1000;
    if (body && body->weight_permille) weight = weight * body->weight_permille / 1000;
    s.mass = float(weight + 120);
    s.front_weight = ch->front_weight_pct > 0 ? ch->front_weight_pct * 0.01f : 0.55f;
    s.wheelbase = ch->wheelbase_mm * 0.001f;
    if (ch->length_mm) s.length = ch->length_mm * 0.001f;
    if (ch->height_mm) s.height = ch->height_mm * 0.001f;
    // Yaw inertia parameter: chassis +0x1C x lightweight(+0x11 %) x body(+0x38 %), x100.
    {
        int yi = ch->unknown_1c;
        if (lw && lw->size() > 0x11 && (*lw)[0x11]) yi = yi * (*lw)[0x11] / 100;
        if (body && body->inertia_pct) yi = yi * body->inertia_pct / 100;
        s.yaw_inertia = yi ? float(yi * 100) : 6000.0f;
    }
    if (body) {
        if (body->width_mm) s.width = body->width_mm * 0.001f;
        const ByteSpan* br = part_record(db, parts, kBody);
        int off_f = br && br->size() > 0x3E ? int(static_cast<signed char>((*br)[0x3D])) : 0;
        int off_r = br && br->size() > 0x3E ? int(static_cast<signed char>((*br)[0x3E])) : 0;
        if (body->front_tread_mm) s.axle[0].track = (body->front_tread_mm - 2 * off_f) * 0.001f;
        if (body->rear_tread_mm) s.axle[1].track = (body->rear_tread_mm - 2 * off_r) * 0.001f;
    }
    if (body) {
        // [spec §6.2] collision extents (body f32 +0x20 / +0x24, mm, truncated).
        // Extents default to length / 2 in integer mm; width/height are the collision
        // extents = body f32 +0x28 / +0x2C.
        int half = int(ch->length_mm) / 2;
        int fe = int(body->inertia[0]), re = int(body->inertia[1]);
        s.front_extent = float(fe ? fe : half) * 0.001f;
        s.rear_extent = float(re ? re : half) * 0.001f;
        if (int(body->inertia[2]) > 0) s.width = float(int(body->inertia[2])) * 0.001f;
        if (int(body->inertia[3]) > 0) s.height = float(int(body->inertia[3])) * 0.001f;
    }
    // Steering lock [spec §6.4]: steer record +0x11 degrees.
    if (const ByteSpan* st = part_record(db, parts, kSteer); st && st->size() > 0x11 && (*st)[0x11])
        s.steering.max_angle = (*st)[0x11] * 3.14159265f / 180.0f;
    // Suspension [spec §4.2 / §6.2 / §6.6]: stock settings -> spec bytes -> axle blocks.
    if (const ByteSpan* su = part_record(db, parts, kSuspension); su && su->size() >= 0x7C) {
        auto B = [&](size_t o) -> int { return (*su)[o]; };
        auto interp = interp_setting;
        const auto& dm = set.damper;
        for (int a = 0; a < 2; ++a) {
            SuspensionSpec& p = s.axle[size_t(a)].susp;
            int spring = set.spring[size_t(a)], c = set.motion_ratio[size_t(a)];
            if (!spring || !c) continue;
            int arb = a ? interp(B(0x4B), B(0x4C), B(0x4D), set.arb[1]) : interp(B(0x47), B(0x48), B(0x49), set.arb[0]);
            int b2 = B(a ? 0x2A : 0x29), b3 = B(a ? 0x5D : 0x5C), b13 = B(a ? 0x7B : 0x7A), g1 = B(a ? 0x79 : 0x78);
            int v_b1 = B(size_t(a ? 0x74 : 0x70)), v_b2 = B(size_t(a ? 0x75 : 0x71));
            int v_r1 = B(size_t(a ? 0x76 : 0x72)), v_r2 = B(size_t(a ? 0x77 : 0x73));
            int f_b1, f_b2, f_r1, f_r2;
            if (a == 0) { // the front second-stage dampers reuse the first-stage indices
                f_b1 = interp(B(0x2B), B(0x2C), B(0x2D), dm[0]);
                f_b2 = interp(B(0x2B), B(0x2F), B(0x30), dm[1]);
                f_r1 = interp(B(0x32), B(0x33), B(0x34), dm[0]);
                f_r2 = interp(B(0x32), B(0x36), B(0x37), dm[1]);
            } else {
                f_b1 = interp(B(0x39), B(0x3A), B(0x3B), dm[4]);
                f_b2 = interp(B(0x39), B(0x3D), B(0x3E), dm[5]);
                f_r1 = interp(B(0x40), B(0x41), B(0x42), dm[6]);
                f_r2 = interp(B(0x40), B(0x44), B(0x45), dm[7]);
            }
            float sc = 100.0f / float(c);
            p.k = spring * 980.0f * float(c) / 100.0f;
            p.arb_gain = arb * 98.0f;
            p.k2 = b2 * 98000.0f;
            p.unsprung = b3 * 0.25f;
            p.bump_damping = b13 * 98.0f;
            p.bump = model::fit_damper(v_b1 * 0.01f * sc, f_b1 * 49.0f, v_b2 * 0.01f * sc, f_b2 * 49.0f);
            p.rebound = model::fit_damper(v_r1 * 0.01f * sc, f_r1 * 49.0f, v_r2 * 0.01f * sc, f_r2 * 49.0f);
            // [spec §4.2/§6] q = the front ride height for BOTH blocks: the rear block reads the front byte again, not
            // the rear one (verified on the rear bump-stop limit of 4 cars whose rear ride height differs)
            p.ride_height = set.ride_height[0] * 0.001f;
            p.bump_room = B(size_t(0x1D + a)) * 0.001f;
            p.droop = B(size_t(0x1F + a)) * 0.001f;
            p.knee_gap = g1 * 0.001f;
            p.valid = p.k > 0;
            if (p.valid) s.axle[size_t(a)].spring_rate = p.k;
            // [spec §4.2] CoG-height parameter X = 0.5 (chassis +0x1D x 0.001 + front ride height).
            if (p.valid && a == 0) s.cg_height = 0.5f * (ch->unknown_1d * 0.001f + p.ride_height);
        }
    }
    // [spec §3.1/4.4, draft 16] axle factor, camber offset and toe from the settings.
    for (int a = 0; a < 2; ++a) {
        const ByteSpan* cr = part_record(db, parts, kChassis);
        int b = cr && cr->size() > 0x1E ? (((*cr)[size_t(a ? 0x1E : 0x12)] + 124) & 0xFF) : 100;
        s.axle[size_t(a)].axle_factor = (b + 900) * 0.001f;
        s.axle[size_t(a)].camber_offset = -set.camber[size_t(a)] * 0.1f * 3.14159265f / 180.0f;
        s.axle[size_t(a)].toe_byte = set.toe[size_t(a)] ? set.toe[size_t(a)] : 128;
    }
    // Camber curves [spec §3.1/6.4]: suspension +0x60.. angle codes, +0x64.. multiplier codes (front), +0x68/+0x6C rear.
    if (const ByteSpan* su = part_record(db, parts, kSuspension); su && su->size() >= 0x70) {
        for (int a = 0; a < 2; ++a) {
            std::vector<std::pair<float, float>> pts;
            for (int i = 0; i < 4; ++i) {
                u8 ac = (*su)[size_t(0x60 + 8 * a + i)], mc = (*su)[size_t(0x64 + 8 * a + i)];
                pts.push_back({(128.0f - ac) * 0.0698132f / 32.0f, mc / 200.0f});
            }
            std::sort(pts.begin(), pts.end());
            bool valid = true;
            for (auto& p : pts) valid = valid && p.second > 0.2f;
            if (!valid) continue;
            Curve c;
            for (auto& p : pts) {
                c.x.push_back(p.first);
                c.y.push_back(p.second);
            }
            s.axle[size_t(a)].camber = c;
        }
    }
    if (!s.axle[0].susp.valid) s.cg_height = 0; // derived from height in finalize()
    s.aero.drag_area = 0.33f * s.width * s.height * 0.85f;

    // --- tyres ---
    const int tire_slot[2] = {kFrontTire, kRearTire};
    const data::TableId tire_table[2] = {data::TableId::FrontTire, data::TableId::RearTire};
    for (int a = 0; a < 2; ++a) {
        auto t = db.tire(tire_table[a], parts[size_t(tire_slot[a])]);
        if (!t) continue;
        if (const ByteSpan* sz = db.record(data::TableId::TireSize, t->size); sz && sz->size() >= 0x0B) {
            float rim_in = (*sz)[0x08], aspect = (*sz)[0x09] * 5.0f, width_mm = (*sz)[0x0A] * 5.0f;
            if (rim_in > 0 && width_mm > 0) {
                s.axle[size_t(a)].tire_width = width_mm * 0.001f;
                s.axle[size_t(a)].tire_radius = rim_in * 0.0254f * 0.5f + width_mm * 0.001f * aspect * 0.01f;
            }
        }
        // [spec §6.5] three records per axle from compounds/force-vols 0..2.
        std::array<std::array<u8, 0x70>, 3> blocks;
        const ByteSpan* tr = part_record(db, parts, tire_slot[a]);
        for (int k = 0; k < 3; ++k) {
            const ByteSpan* c = tr && tr->size() >= size_t(0x28 + 8 * k) ? db.record(data::TableId::TireCompound, rd64(*tr, size_t(0x20 + 8 * k))) : nullptr;
            const ByteSpan* f = tr && tr->size() >= size_t(0x48 + 8 * k) ? db.record(data::TableId::TireForceVol, rd64(*tr, size_t(0x40 + 8 * k))) : nullptr;
            blocks[size_t(k)] = tire_block(c, f);
        }
        auto& b0 = blocks[0];
        if (!b0[0x57]) b0[0x57] = b0[0x4B];
        if (!b0[0x58]) b0[0x58] = b0[0x4C];
        if (!b0[0x59]) b0[0x59] = b0[0x54];
        for (int k = 1; k < 3; ++k)
            for (size_t o = 0x4B; o <= 0x5A; ++o)
                if (!blocks[size_t(k)][o]) blocks[size_t(k)][o] = b0[o];
        for (int k = 0; k < 3; ++k) {
            TireSpec rec = tire_record(blocks[size_t(k)], s.axle[size_t(a)].tire_width);
            rec.compound = t->compound[size_t(k)];
            s.tire_slots[size_t(a)][size_t(k)] = rec;
        }
        s.tire[size_t(a)] = s.tire_slots[size_t(a)][0];
    }
    for (int a = 0; a < 2; ++a) {
        float r = s.axle[size_t(a)].tire_radius;
        s.axle[size_t(a)].wheel_inertia = std::clamp(12.0f * r * r * 1.0f + 0.3f, 0.6f, 3.0f); // [placeholder]
    }

    // --- engine ---
    auto en = db.engine(parts[kEngine]);
    if (!en || en->rpm.empty()) return fail("missing engine record");
    s.power_ps = en->power_ps;
    const ByteSpan* er = part_record(db, parts, kEngine);
    if (!er || er->size() < 0x58) return fail("short engine record");
    // [spec §2.1 / §6.2] raw spec values plus the part adders/multipliers
    // (docs/formats/database.md "runtime car spec"); integer percent maths.
    u8 axis[16];
    u16 torque[16];
    int n = std::clamp(int((*er)[0x57]), 1, 16);
    for (int i = 0; i < 16; ++i) {
        axis[i] = (*er)[size_t(0x47 + i)];
        torque[i] = rd16(*er, size_t(0x18 + 2 * i));
    }
    int limit = (*er)[0x45], limit2 = (*er)[0x46], idle = (*er)[0x44];
    u32 hi = 1000, lo = 100;
    // [spec §6.2] the game runs one helper per parts table in its own order;
    // the factors are integer percent steps, so the order matters, and it runs the port
    // polish helper twice (its factors apply squared: 102 % -> 1000 * 1.02 * 1.02 = 1040).
    auto engine_part = [&](int slot) {
        const ByteSpan* r = part_record(db, parts, slot);
        if (!r) return;
        if (slot == kEngineBalance || slot == kNaTune) {
            for (int i = 0; i < n; ++i) axis[i] = u8(axis[i] + byte_at(r, 0x11));
            limit += byte_at(r, 0x12);
            if (u8 p = byte_at(r, 0x13)) hi = hi * p / 100;
            if (u8 p = byte_at(r, 0x14)) lo = lo * p / 100;
        } else if (slot == kTurbo) {
            if (r->size() < 0x1E) return;
            if (u16 p = rd16(*r, 0x1C)) hi = hi * p / 100;
            if (u8 p = (*r)[0x1B]) lo = lo * p / 100;
            limit += (*r)[0x18];
            limit2 += (*r)[0x19];
            // [spec §2.3, draft 14c field map] stock boost settings = turbo +0x12..+0x17 (peak, knee,
            // rate of stage 0, then stage 1), cap = 1 + 0.1 x +0x11.
            TurboSpec& t = s.engine.turbo;
            u8 cap = (*r)[0x11];
            int p0 = set.turbo[0], k0 = set.turbo[1], r0 = set.turbo[2], p1 = set.turbo[3], k1 = set.turbo[4], r1 = set.turbo[5];
            t.on = cap && k0 && p0 && r0;
            t.cap = 1 + 0.1f * cap;
            t.stage[0] = {true, p0 * 0.1f, k0 * 100.0f, r0 * 0.1f};
            t.stage[1] = {p1 && k1 && r1, p1 * 0.1f, k1 * 100.0f, r1 * 0.1f};
        } else {
            if (u8 p = byte_at(r, 0x11)) hi = hi * p / 100;
            if (u8 p = byte_at(r, 0x12)) lo = lo * p / 100;
        }
    };
    for (int slot : {kPortPolish, kEngineBalance, kDisplacement, kComputer, kNaTune, kTurbo, kPortPolish, kMuffler, kIntercooler})
        engine_part(slot);
    s.engine.launch_rpm = (*er)[0x43] ? (*er)[0x43] * 10.0f : 2500.0f; // [spec §2.4]
    s.engine.upshift_rpm = limit2 * 100.0f;                            // [spec §2.6] UP
    float rpm[16], nm[16];
    n = model::build_torque_curve(torque, axis, n, u16((*er)[0x42] * 10), u16(std::min<u32>(hi, 65535)),
                                  u8(std::min<u32>(lo, 255)), rpm, nm, &s.engine.turbo);
    for (int i = 0; i < n; ++i) {
        if (!s.engine.curve_rpm.empty() && rpm[i] <= s.engine.curve_rpm.back()) continue; // padding
        s.engine.curve_rpm.push_back(rpm[i]);
        s.engine.curve_torque.push_back(nm[i]);
    }
    if (s.engine.curve_rpm.empty()) return fail("empty torque curve");
    // [spec §2.1] rev limit = +0x45 x100 (last axis point if 0), at least +0x46 x100 + 200; idle = +0x44 x10.
    float lim = limit > 0 ? limit * 100.0f : s.engine.curve_rpm.back();
    s.engine.rev_limit_rpm = std::max(lim, limit2 * 100.0f + 200.0f);
    s.engine.idle_rpm = idle ? idle * 10.0f : 900.0f;
    if (const data::ModCar* mc = db.mod_car(car_label); mc && mc->electric) {
        s.engine.electric = true;
        s.engine.idle_rpm = 0;
        s.engine.launch_rpm = 0;
        s.engine.regen_torque = mc->regen_torque_nm;
        s.engine.regen_power = mc->regen_kw * 1000.0f;
    }

    // --- gearbox ---
    if (auto ge = db.gearbox(parts[kGear])) {
        // User gear words [spec §6.6]: reverse, gears 1..7 (x1000, -999 = none), final drive.
        std::vector<float> ratios;
        for (int i = 1; i <= 7; ++i) ratios.push_back(set.gears[size_t(i)] * 0.001f);
        s.gearbox.ratios = fill_ratios(ratios, ge->gears ? ge->gears : 5);
        if (set.final_drive > 0) s.gearbox.final_drive = set.final_drive * 0.001f;
        if (set.gears[0] != 0) s.gearbox.reverse = std::fabs(set.gears[0] * 0.001f);
        if (ge->adjustable) {
            // [spec §6.7] computed gearbox: the game regenerates from the slider.
            CarSettings g = set;
            if (apply_gear_slider(db, parts, g)) {
                int gear_count = ge->gears;
                s.gearbox.ratios.clear();
                for (int i = 1; i <= std::min(gear_count, 7); ++i) s.gearbox.ratios.push_back(g.gears[size_t(i)] * 0.001f);
                s.gearbox.reverse = g.gears[0] * 0.001f;
            }
        }
    }

    // --- drivetrain ---
    if (auto dt = db.drivetrain(parts[kDrivetrain])) {
        s.drivetrain.drive = dt->drive_type <= 4 ? Drive(dt->drive_type) : Drive::FR;
        int f = dt->front_torque[2]; // B4 = drivetrain +0x13 (the game does not read setting 0x118)
        if (s.drivetrain.drive == Drive::AWD) s.drivetrain.front_share = (f > 0 && f < 100) ? f * 0.01f : 0.4f;
        s.drivetrain.awd_system = dt->awd_system;
        s.drivetrain.awd_coupling = f * 0.01f;
        s.drivetrain.awd_share_byte = f;
        // [spec §2.2/2.4/6.2] engine inertia (+0x20 x flywheel +0x12 % x clutch +0x12 %) and
        // friction (+0x16 x flywheel/clutch/shaft +0x11 %), integer percent maths.
        const ByteSpan* fw = part_record(db, parts, kFlywheel);
        const ByteSpan* cl = part_record(db, parts, kClutch);
        const ByteSpan* ps = part_record(db, parts, kPropShaft);
        u32 inertia = dt->params[5], fric = dt->params[0];
        if (u8 p = byte_at(fw, 0x12)) inertia = inertia * p / 100;
        if (u8 p = byte_at(cl, 0x12)) inertia = inertia * p / 100;
        for (const ByteSpan* r : {fw, cl, ps})
            if (u8 p = byte_at(r, 0x11)) fric = fric * p / 100;
        if (inertia) s.engine.inertia = inertia * 0.001f;
        // [spec §2.7/6.2] axle inertia words: 0xD2/0xD4 = +0x18/+0x1A x flywheel(+0x13/+0x14)% x
        // clutch(+0x13/+0x14)% x prop(+0x12/+0x13)%; 0xCE/0xD0 = +0x1C/+0x1E x prop(+0x14/+0x15)%.
        for (int a = 0; a < 2; ++a) {
            u32 spin = dt->params[size_t(1 + a)], shaft = dt->params[size_t(3 + a)];
            if (u8 p = byte_at(fw, size_t(0x13 + a))) spin = spin * p / 100;
            if (u8 p = byte_at(cl, size_t(0x13 + a))) spin = spin * p / 100;
            if (u8 p = byte_at(ps, size_t(0x12 + a))) spin = spin * p / 100;
            if (u8 p = byte_at(ps, size_t(0x14 + a))) shaft = shaft * p / 100;
            s.axle[size_t(a)].spin_inertia = float(u16(spin)) * 0.01f;
            s.axle[size_t(a)].shaft_inertia = float(u16(shaft)) * 0.01f;
        }
        // k_f = u16 +0x16 x 0.98 / omega_last (last rpm-axis point in rad/s): engine braking at the
        // top of the curve = u16 x 0.98 N*m [spec §2.2, verified against the original game].
        if (fric && !s.engine.curve_rpm.empty())
            s.engine.friction_coeff = fric * 0.98f / (s.engine.curve_rpm.back() * model::kRpmToRadS);
    }

    // --- differentials [spec §2.7]: LSD type +0x11 / +0x1B, front initial/accel/decel
    // +0x12/+0x15/+0x18, rear +0x1C/+0x1F/+0x22.
    if (const ByteSpan* ls = part_record(db, parts, kLsd); ls && ls->size() > 0x22) {
        for (int a = 0; a < 2; ++a) {
            // The byte is the ASCII letter (f, h, m, n, v, y on disc); type = its index in "frnmhvyt".
            static const char kTypes[] = "frnmhvyt";
            const char* hit = std::strchr(kTypes, char((*ls)[size_t(a ? 0x1B : 0x11)]));
            if (!hit || !*hit) continue;
            int type = int(hit - kTypes);
            s.drivetrain.lsd_type[size_t(a)] = type;
            s.drivetrain.lsd_initial[size_t(a)] = set.lsd[size_t(0 + a)]; // setting 0x194/0x198
            s.drivetrain.lsd_accel[size_t(a)] = set.lsd[size_t(2 + a)];   // 0x19C/0x1A0
            s.drivetrain.lsd_decel[size_t(a)] = set.lsd[size_t(4 + a)];   // 0x1A4/0x1A8
        }
    }

    // --- brakes [spec §4.5]: front/rear torque per wheel, handbrake ---
    if (const ByteSpan* br = part_record(db, parts, kBrake); br && br->size() > 0x13) {
        if ((*br)[0x11]) s.axle[0].max_brake_torque = (*br)[0x11] * 98.0f;
        if ((*br)[0x12]) s.axle[1].max_brake_torque = (*br)[0x12] * 98.0f;
        s.aids.handbrake_torque = (*br)[0x13] * 98.0f;
    }

    // --- aero [spec §4.7]: A from body f32 +0x44 (m^2), Cd = body +0x39 %, rho = 1.2 ---
    if (body) {
        const ByteSpan* br = part_record(db, parts, kBody);
        // the stored area = int(body f32 +0x44 x 10000): the area is quantised to 1e-4 m^2.
        int a16 = int(body->unknown_44 * 10000.0f);
        float area = a16 > 1000 ? a16 / 10000.0f : s.width * s.height;
        float cd = br && br->size() > 0x39 && (*br)[0x39] ? (*br)[0x39] * 0.01f : 0.33f;
        s.aero.air_density = 1.2f;
        s.aero.drag_area = area * cd;
        float af = 0.5f * 1.2f * area;
        // Stock downforce settings [spec §6.6]: body +0x13 front, +0x3C rear (percent of A_f).
        s.aero.c_front = af * set.downforce_front * 0.01f; // setting 0x124
        s.aero.c_rear = af * set.downforce_rear * 0.01f;   // setting 0x128
        s.aero.c_drag = af * cd + 0.1f * (s.aero.c_front + s.aero.c_rear);
    }

    // --- steering [spec §4.4/6.4]: pad curve B, steer +0x1E.. (6 x km/h, 6 x 0.2 deg) ---
    if (const ByteSpan* st = part_record(db, parts, kSteer); st && st->size() > 0x29) {
        Curve c;
        for (int i = 0; i < 6; ++i) {
            c.x.push_back((*st)[size_t(0x1E + i)] / 3.6f);
            c.y.push_back((*st)[size_t(0x24 + i)] * 0.2f * 3.14159265f / 180.0f);
        }
        bool ok = c.y[0] > 0;
        for (int i = 1; i < 6; ++i) ok = ok && c.x[size_t(i)] >= c.x[size_t(i - 1)];
        if (ok) s.steering.pad_curve = c;
    }

    // --- aids ---
    // [spec §4.6/6.2/6.6] TCS: level = interp(TCSC +0x15, +0x16 / steps +0x12 /
    // index +0x11); gain = +0x13 (x 0.1, 0 when the level is 100);
    // lateral a = +0x14 (x 0.1), b = +0x17 (x 0.01).
    if (const ByteSpan* tc = db.record(data::TableId::Tcsc, parts[25]); tc && tc->size() > 0x17) {
        int steps = (*tc)[0x12], mn = (*tc)[0x15], mx = (*tc)[0x16], idx = set.tcs;
        int lvl = steps < 2 ? mn : mn + (mx - mn) * (idx - 1) / (steps - 1);
        s.aids.tcs = lvl != 0 && lvl != 100;
        s.aids.tcs_level = lvl * 0.01f;
        s.aids.tcs_scale = (lvl == 0 || lvl == 100) ? 1.0f : lvl * 0.01f;
        s.aids.tcs_gain = lvl == 100 ? 0.0f : (*tc)[0x13] * 0.1f;
        s.aids.tcs_lat_a = (*tc)[0x14] * 0.1f;
        s.aids.tcs_lat_b = (*tc)[0x17] * 0.01f;
    } else {
        s.aids.tcs = false;
    }
    // [spec §4.6/6.2/6.6] ASCC channels (indices default +0x12, +0x1B, +0x21, +0x26).
    if (const ByteSpan* as = db.record(data::TableId::Ascc, parts[24]); as && as->size() > 0x26) {
        auto A = [&](size_t o) -> int { return (*as)[o]; };
        auto interp = interp_setting;
        s.aids.ascc_flags = u8((A(0x17) & 3) | (A(0x16) & 3) << 2 | (A(0x1D) & 3) << 4 | (A(0x1C) & 3) << 6);
        s.aids.ascc_a_gain = interp(A(0x11), A(0x14), A(0x15), set.ascc[0]) * 0.01f;
        s.aids.ascc_a_thr = A(0x13) * 0.01f;
        s.aids.ascc_d_gain = interp(A(0x18), A(0x19), A(0x1A), set.ascc[1]) * 0.01f;
        s.aids.ascc_c_gain = interp(A(0x1E), A(0x1F), A(0x20), set.ascc[2]) * 0.01f;
        s.aids.ascc_c_thr = A(0x22) * 0.01f;
        s.aids.ascc_e_gain = interp(A(0x23), A(0x24), A(0x25), set.ascc[3]) * 0.01f;
    }
    // ABS gain G = brake-controller setting x 0.1 (index = +0x14) [spec §4.5/6.6].
    if (const ByteSpan* bc = part_record(db, parts, 1); bc && bc->size() > 0x17) {
        // front = interp(+0x12, +0x13 / steps +0x11 / front index), rear = interp(+0x16, +0x17 / +0x15 / rear).
        s.aids.abs_gain = interp_setting((*bc)[0x11], (*bc)[0x12], (*bc)[0x13], set.brake_front) * 0.1f;
        s.aids.abs_gain_rear = interp_setting((*bc)[0x15], (*bc)[0x16], (*bc)[0x17], set.brake_rear) * 0.1f;
    }

    s.finalize();
    return s;
}

} // namespace gt3r::sim
