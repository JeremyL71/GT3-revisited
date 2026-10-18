#include "sim/car_spec_io.h"

#include <cstring>
#include <stdexcept>

namespace gt3r::sim {

namespace {

constexpr u32 kSpecVersion = 1;

struct Out {
    Bytes b;
    template <class T>
    void raw(const T& v) {
        u8 t[sizeof(T)];
        std::memcpy(t, &v, sizeof(T)); // little-endian hosts only (all our targets)
        b.insert(b.end(), t, t + sizeof(T));
    }
    void operator()(float& v) { raw(v); }
    void operator()(bool& v) { raw(u8(v ? 1 : 0)); }
    void operator()(u8& v) { raw(v); }
    void operator()(int& v) { raw(s32(v)); }
    void operator()(std::string& s) {
        raw(u32(s.size()));
        b.insert(b.end(), s.begin(), s.end());
    }
    void operator()(std::vector<float>& v) {
        raw(u32(v.size()));
        for (float& f : v) (*this)(f);
    }
    template <class E>
        requires std::is_enum_v<E>
    void operator()(E& e) {
        raw(static_cast<u8>(e));
    }
};

struct In {
    ByteSpan d;
    size_t pos = 0;
    template <class T>
    T raw() {
        if (pos + sizeof(T) > d.size()) throw std::runtime_error("car spec truncated");
        T v;
        std::memcpy(&v, d.data() + pos, sizeof(T));
        pos += sizeof(T);
        return v;
    }
    u32 count() {
        u32 n = raw<u32>();
        if (n > d.size() - pos) throw std::runtime_error("car spec: bad count");
        return n;
    }
    void operator()(float& v) { v = raw<float>(); }
    void operator()(bool& v) { v = raw<u8>() != 0; }
    void operator()(u8& v) { v = raw<u8>(); }
    void operator()(int& v) { v = raw<s32>(); }
    void operator()(std::string& s) {
        u32 n = count();
        s.assign(reinterpret_cast<const char*>(d.data() + pos), n);
        pos += n;
    }
    void operator()(std::vector<float>& v) {
        u32 n = count();
        v.resize(n);
        for (float& f : v) (*this)(f);
    }
    template <class E>
        requires std::is_enum_v<E>
    void operator()(E& e) {
        e = static_cast<E>(raw<u8>());
    }
};

template <class A, class T, size_t N>
void arr(A& a, std::array<T, N>& v) {
    for (T& x : v) a(x);
}

template <class A>
void visit(A& a, Curve& c) {
    a(c.x), a(c.y);
}

template <class A>
void visit(A& a, TireSpec& t) {
    visit(a, t.load), visit(a, t.slip_a), visit(a, t.slip_b), visit(a, t.limit_d), visit(a, t.long_c);
    arr(a, t.surface_mu);
    a(t.width), a(t.fz_ref), a(t.mu), a(t.wear_grip), a(t.wear_rate), a(t.opt_drive), a(t.opt_brake);
    a(t.compound);
}

template <class A>
void visit(A& a, SuspensionSpec::Damper& d) {
    a(d.t), a(d.o), a(d.g);
}

template <class A>
void visit(A& a, AxleSpec& x) {
    a(x.track), a(x.tire_radius), a(x.tire_width), a(x.wheel_inertia), a(x.spin_inertia), a(x.shaft_inertia);
    a(x.spring_rate), a(x.ride_frequency), a(x.damping_ratio), a(x.damper), a(x.rest_length), a(x.travel);
    a(x.arb_rate), a(x.max_brake_torque);
    SuspensionSpec& s = x.susp;
    a(s.valid), a(s.k), a(s.arb_gain), a(s.k2), a(s.unsprung), a(s.bump_damping);
    visit(a, s.bump), visit(a, s.rebound);
    a(s.ride_height), a(s.bump_room), a(s.droop), a(s.knee_gap);
    visit(a, x.camber);
    a(x.axle_factor), a(x.camber_offset), a(x.toe_byte), a(x.toe);
}

template <class A>
void visit(A& a, CarSpec& c) {
    a(c.label), a(c.name), a(c.mass), a(c.front_weight), a(c.wheelbase), a(c.length), a(c.width), a(c.height);
    a(c.cg_height), a(c.front_extent), a(c.rear_extent), a(c.yaw_inertia);
    arr(a, c.surface_slot);
    a(c.brake_balance), a(c.brake_decel), a(c.power_ps);
    for (AxleSpec& x : c.axle) visit(a, x);
    for (TireSpec& t : c.tire) visit(a, t);
    for (auto& slots : c.tire_slots)
        for (TireSpec& t : slots) visit(a, t);
    EngineSpec& e = c.engine;
    a(e.curve_rpm), a(e.curve_torque), a(e.idle_rpm), a(e.rev_limit_rpm), a(e.limiter_hysteresis_rpm), a(e.inertia);
    a(e.friction_coeff), a(e.launch_rpm), a(e.upshift_rpm);
    a(e.turbo.on), a(e.turbo.cap);
    for (TurboSpec::Stage& s : e.turbo.stage) a(s.active), a(s.peak), a(s.knee_rpm), a(s.rate);
    a(e.electric), a(e.regen_torque), a(e.regen_power);
    GearboxSpec& g = c.gearbox;
    a(g.ratios), a(g.reverse), a(g.final_drive), a(g.shift_time), a(g.efficiency);
    DrivetrainSpec& d = c.drivetrain;
    a(d.drive), a(d.front_share), a(d.awd_system), a(d.center_lock), a(d.lsd_preload), a(d.lsd_lock), a(d.lsd_viscous);
    arr(a, d.lsd_type), arr(a, d.lsd_initial), arr(a, d.lsd_accel), arr(a, d.lsd_decel);
    a(d.shift_clutch_torque), a(d.awd_share_byte), a(d.awd_strength_byte), a(d.awd_coupling);
    a(d.clutch_max_torque), a(d.launch_rpm);
    AeroSpec& r = c.aero;
    a(r.c_drag), a(r.c_front), a(r.c_rear), a(r.slip_a), a(r.slip_b), a(r.slip_k_up), a(r.slip_k_down);
    a(r.drag_area), a(r.air_density);
    SteeringSpec& s = c.steering;
    visit(a, s.pad_curve);
    a(s.follow_k1), a(s.follow_k2), a(s.follow_k3), a(s.follow_rate_max), a(s.follow_return_rate_max);
    a(s.follow_rate_digital), a(s.max_angle), a(s.speed_reduction), a(s.rate), a(s.ackermann), a(s.ack_inner);
    a(s.ack_outer);
    AidsSpec& x = c.aids;
    a(x.abs), a(x.abs_gain), a(x.abs_gain_rear), a(x.slip_opt), a(x.handbrake_torque), a(x.tcs), a(x.tcs_slip);
    a(x.tcs_level), a(x.tcs_gain), a(x.tcs_lat_a), a(x.tcs_lat_b), a(x.tcs_scale);
    a(x.ascc_a_gain), a(x.ascc_a_thr), a(x.ascc_d_gain), a(x.ascc_c_gain), a(x.ascc_c_thr), a(x.ascc_e_gain);
    a(x.ascc_flags);
}

} // namespace

Bytes serialize_car_spec(const CarSpec& spec) {
    Out o;
    o.raw(kSpecVersion);
    CarSpec c = spec;
    visit(o, c);
    return std::move(o.b);
}

std::optional<CarSpec> parse_car_spec(ByteSpan data) {
    try {
        In in{data};
        if (in.raw<u32>() != kSpecVersion) return std::nullopt;
        CarSpec c;
        visit(in, c);
        if (in.pos != data.size()) return std::nullopt;
        return c;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

} // namespace gt3r::sim
