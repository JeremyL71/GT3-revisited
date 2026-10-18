// Physics test: drive any car from the database on a flat infinite plane
// with live telemetry. Top-down view (car pointing up) plus engine,
// driveline and per-wheel tyre data, so changes to the sim can be felt with
// the pressure-sensitive virtual pad.
//
// Controls (virtual DualShock 2): Cross pressure = throttle, Square pressure
// = brake, LX = steer, R1 / L1 = shift up / down, Circle = handbrake,
// Select = automatic/manual, Start = reset, Triangle = car list / back.
// GT3R_SIM_AUTODRIVE=1 holds throttle and some steering (for --shot).

#include "app/app.h"
#include "app/screen_registry.h"
#include "game/data/car_database.h"
#include "sim/models.h"
#include "sim/vehicle.h"

#include <SDL3/SDL_render.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace gt3r {

namespace {

using sim::Vec3;

struct FlatPlane : sim::TrackQuery {
    sim::SurfaceHit probe(float, float, float y_max) const override {
        sim::SurfaceHit h;
        h.hit = y_max >= 0;
        return h;
    }
};

std::string ascii(const std::string& s) {
    std::string out;
    for (unsigned char c : s) out += c >= 0x20 && c < 0x7F ? char(c) : (c >= 0xC0 ? '-' : '\0');
    out.erase(std::remove(out.begin(), out.end(), '\0'), out.end());
    return out;
}

std::string fmt(const char* f, double a, double b = 0, double c = 0) {
    char buf[96];
    std::snprintf(buf, sizeof buf, f, a, b, c);
    return buf;
}

Vec3 rotate(const sim::Quat& q, const Vec3& v) {
    Vec3 u{q.x, q.y, q.z};
    Vec3 t = u.cross(v) * 2.0f;
    return v + t * q.w + u.cross(t);
}

class SimScreen : public Screen {
public:
    bool wants_menu_input() const override { return mode_ == Mode::List; }

    void update(App& app, double dt) override {
        if (!loaded_) load(app);
        const PadState& pad = app.input().pad(0);
        if (mode_ == Mode::List) {
            const MenuInput& m = app.menu();
            int n = int(entries_.size());
            if (n) sel_ = (sel_ + m.vertical() + n) % n;
            if (n && (m.pressed(PadButton::Cross))) start(sel_);
            if (m.pressed(PadButton::Triangle)) {
                if (car_) mode_ = Mode::Drive;
                else app.pop();
            }
            return;
        }
        if (!car_) {
            mode_ = Mode::List;
            return;
        }
        auto edge = [&](PadButton b) {
            bool down = pad.down(b);
            bool was = prev_buttons_ & (1u << unsigned(b));
            return down && !was;
        };
        if (edge(PadButton::Triangle)) {
            mode_ = Mode::List;
            app.menu().suppress();
        }
        if (edge(PadButton::Start)) reset();
        if (edge(PadButton::Select)) automatic_ = !automatic_;
        shift_up_ |= edge(PadButton::R1);
        shift_down_ |= edge(PadButton::L1);
        prev_buttons_ = pad.buttons;

        sim::ControlInputs in;
        const PadState& drive = app.input().race_pad(0); // pedals and steering: the race layout
        in.throttle = drive.press(PadButton::Cross) / 255.0f;
        in.brake = drive.press(PadButton::Square) / 255.0f;
        in.steer = drive.axis_f(PadAxis::LX);
        if (std::fabs(in.steer) < 0.03f) in.steer = 0;
        in.handbrake = drive.down(PadButton::Circle);
        in.automatic = automatic_;
        if (autodrive_) {
            in.throttle = 0.8f;
            in.steer = 0.25f;
        }

        accum_ += std::min(dt, 0.25);
        const double tick = 1.0 / sim::kDefaultTickHz;
        while (accum_ >= tick) {
            accum_ -= tick;
            in.shift_up = shift_up_;
            in.shift_down = shift_down_;
            shift_up_ = shift_down_ = false;
            car_->step(in, plane_, tick);
            input_ = in;
            if (++trail_tick_ % 6 == 0) {
                trail_.push_back(car_->state().position);
                if (trail_.size() > 400) trail_.pop_front();
            }
        }
    }

    void render(App& app, Ui& ui) override {
        (void)app;
        if (mode_ == Mode::List || !car_) {
            render_list(ui);
            return;
        }
        render_view(ui);
        render_telemetry(ui);
        ui.footer("Cross: gas  Square: brake  LX: steer  R1/L1: shift  O: handbrake  Select: AT/MT  Start: reset  "
                  "Tri: cars");
    }

private:
    enum class Mode { List, Drive };
    struct Entry {
        std::string label, name;
    };

    void load(App& app) {
        loaded_ = true;
        if (const char* a = std::getenv("GT3R_SIM_AUTODRIVE")) {
            autodrive_ = true;
            prerun_ = std::atof(a); // seconds simulated at start (screenshots)
        }
        entries_.push_back({"", "Test car (built-in, R34-like 4WD)"});
        if (app.vfs() && db_.load(*app.vfs())) {
            have_db_ = true;
            for (const auto& c : db_.cars()) {
                if (c.label.rfind("sample", 0) == 0) continue;
                entries_.push_back({c.label, ascii(c.name) + "  [" + c.label + "]"});
            }
        } else if (app.vfs()) {
            error_ = "database: " + db_.error();
        }
        int def = 0;
        for (size_t i = 0; i < entries_.size(); ++i)
            if (entries_[i].label == "r34_gtr_v_99") def = int(i);
        sel_ = def;
        start(def);
    }

    void start(int idx) {
        const Entry& e = entries_[size_t(idx)];
        std::string err;
        if (e.label.empty()) car_ = sim::Vehicle::create(sim::CarSpec::test_car());
        else car_ = sim::Vehicle::create(db_, e.label, &err);
        if (!car_) {
            error_ = err;
            mode_ = Mode::List;
            return;
        }
        error_.clear();
        name_ = e.name;
        reset();
        mode_ = Mode::Drive;
        if (autodrive_) {
            sim::ControlInputs in;
            in.throttle = 0.8f;
            in.steer = 0.25f;
            for (int i = 0; i < int(prerun_ * sim::kDefaultTickHz); ++i) {
                car_->step(in, plane_, 1.0 / sim::kDefaultTickHz);
                if (i % 6 == 0) trail_.push_back(car_->state().position);
            }
            input_ = in;
        }
    }

    void reset() {
        car_->reset({0, 0, 0}, {});
        trail_.clear();
        accum_ = 0;
    }

    void render_list(Ui& ui) {
        ui.header("PHYSICS TEST", "Choose a car (Cross)  " + std::to_string(entries_.size()) + " cars");
        const int rows = 24;
        int first = ui.list_window(int(entries_.size()), sel_, rows);
        for (int r = 0; r < rows && first + r < int(entries_.size()); ++r) {
            int i = first + r;
            ui.text(12, 30 + r * 12.0f, entries_[size_t(i)].name, i == sel_ ? colors::kAccent : colors::kText);
        }
        if (!error_.empty()) ui.text(12, 330, error_, colors::kBad);
        ui.footer("Up/Down: select  Cross: drive  Triangle: back");
    }

    void line(Ui& ui, float x0, float y0, float x1, float y1, Color c) {
        SDL_SetRenderDrawColor(ui.renderer(), c.r, c.g, c.b, c.a);
        SDL_RenderLine(ui.renderer(), x0, y0, x1, y1);
    }

    void render_view(Ui& ui) {
        const sim::VehicleState& st = car_->state();
        const sim::VehicleSetup& su = car_->setup();
        const float vx0 = 4, vy0 = 14, vw = 352, vh = 330;
        const float cx = vx0 + vw * 0.5f, cy = vy0 + vh * 0.6f, scale = 9.0f; // px per metre
        ui.fill(vx0, vy0, vw, vh, Color{20, 24, 22});
        Vec3 fwd = rotate(st.orientation, {0, 0, -1});
        fwd.y = 0;
        float fl = fwd.length();
        fwd = fl > 1e-4f ? fwd * (1 / fl) : Vec3{0, 0, -1};
        Vec3 right{-fwd.z, 0, fwd.x};
        auto to_screen = [&](const Vec3& w, float& sx, float& sy) {
            Vec3 p = w - st.position;
            sx = cx + p.dot(right) * scale;
            sy = cy - p.dot(fwd) * scale;
        };
        auto clip = [&](float x, float y) { return x >= vx0 && x <= vx0 + vw && y >= vy0 && y <= vy0 + vh; };
        // World grid every 5 m (drawn as clipped segments).
        const float step = 5, reach = 40;
        float gx0 = std::floor((st.position.x - reach) / step) * step, gz0 = std::floor((st.position.z - reach) / step) * step;
        for (float g = 0; g <= 2 * reach; g += step) {
            for (int axis = 0; axis < 2; ++axis) {
                Vec3 a = axis ? Vec3{gx0 + g, 0, gz0} : Vec3{gx0, 0, gz0 + g};
                Vec3 b = axis ? Vec3{gx0 + g, 0, gz0 + 2 * reach} : Vec3{gx0 + 2 * reach, 0, gz0 + g};
                const int segs = 48;
                for (int s = 0; s < segs; ++s) {
                    Vec3 p0 = a + (b - a) * (float(s) / segs), p1 = a + (b - a) * (float(s + 1) / segs);
                    float x0, y0, x1, y1;
                    to_screen(p0, x0, y0);
                    to_screen(p1, x1, y1);
                    if (clip(x0, y0) && clip(x1, y1)) line(ui, x0, y0, x1, y1, Color{45, 55, 50});
                }
            }
        }
        // Trail.
        for (size_t i = 1; i < trail_.size(); ++i) {
            float x0, y0, x1, y1;
            to_screen(trail_[i - 1], x0, y0);
            to_screen(trail_[i], x1, y1);
            if (clip(x0, y0) && clip(x1, y1)) line(ui, x0, y0, x1, y1, Color{90, 120, 200});
        }
        // Body outline (car points up).
        float hl = su.length * 0.5f * scale, hw = su.width * 0.5f * scale;
        float ox = 0, oy = 0; // car origin is the screen centre
        Color body{200, 200, 210};
        line(ui, cx - hw + ox, cy - hl + oy, cx + hw, cy - hl, body);
        line(ui, cx + hw, cy - hl, cx + hw, cy + hl, body);
        line(ui, cx + hw, cy + hl, cx - hw, cy + hl, body);
        line(ui, cx - hw, cy + hl, cx - hw, cy - hl, body);
        line(ui, cx - hw * 0.6f, cy - hl + 4, cx + hw * 0.6f, cy - hl + 4, colors::kAccent); // nose
        // Wheels: rectangle rotated by steer, colour by slip; force arrows.
        const auto& tel = car_->telemetry();
        for (int i = 0; i < 4; ++i) {
            const auto& w = st.wheels[size_t(i)];
            const auto& ws = su.wheels[size_t(i)];
            float wx = cx + ws.rest_center.x * scale, wy = cy + ws.rest_center.z * scale;
            float a = w.steer_angle, ca = std::cos(a), sa = std::sin(a);
            float r = ws.radius * scale, hwid = ws.width * 0.5f * scale;
            float slip = std::min(1.0f, std::fabs(w.slip_ratio) / 0.25f + std::fabs(w.slip_angle) / 0.3f);
            Color c = !w.grounded ? colors::kDim
                                  : Color{u8(80 + 175 * slip), u8(220 - 150 * slip), 90};
            // Corners in wheel frame: forward = -y on screen.
            float px[4] = {-hwid, hwid, hwid, -hwid}, py[4] = {-r, -r, r, r};
            float sx[4], sy[4];
            for (int k = 0; k < 4; ++k) {
                sx[k] = wx + px[k] * ca - py[k] * sa;
                sy[k] = wy + px[k] * sa + py[k] * ca;
            }
            for (int k = 0; k < 4; ++k) line(ui, sx[k], sy[k], sx[(k + 1) % 4], sy[(k + 1) % 4], c);
            // Tyre force (wheel frame -> screen): fx along the wheel, fy to the right.
            float fscale = 12.0f / 4000.0f;
            float fxw = tel.fx[size_t(i)] * fscale, fyw = tel.fy[size_t(i)] * fscale;
            float dx = fyw * ca + fxw * sa, dy = fyw * sa - fxw * ca;
            line(ui, wx, wy, wx + dx, wy + dy, Color{255, 120, 220});
        }
        // Velocity vector.
        Vec3 v = st.velocity;
        float vxs = v.dot(right) * 1.5f, vys = -v.dot(fwd) * 1.5f;
        line(ui, cx, cy, cx + vxs, cy + vys, Color{120, 220, 255});
        ui.frame(vx0, vy0, vw, vh, colors::kHighlight);
        ui.text(vx0 + 4, vy0 + 4, "grid 5 m   pink: tyre force   cyan: velocity", colors::kDim);
    }

    void render_telemetry(Ui& ui) {
        const sim::VehicleState& st = car_->state();
        const sim::VehicleSetup& su = car_->setup();
        const sim::CarSpec& sp = car_->spec();
        const auto& tel = car_->telemetry();
        float x = 364, y = 14;
        ui.text(x, y, name_.substr(0, 34), colors::kAccent);
        y += 11;
        const char* drive[] = {"FR", "FF", "4WD", "MR", "RR"};
        ui.text(x, y,
                fmt("%.0fkg %.0fkW@%.0f", sp.mass, sp.peak_power_kw(), sp.peak_power_rpm()) + " " +
                    drive[int(sp.drivetrain.drive)] + " " + sp.tire[0].compound,
                colors::kDim);
        y += 16;
        std::string gear = st.gear < 0 ? "R" : st.gear == 0 ? "N" : std::to_string(st.gear);
        ui.text(x, y, fmt("%5.1f km/h", st.speed_kmh), colors::kText);
        ui.text(x + 92, y, gear + (input_.automatic ? " AT" : " MT"), colors::kAccent);
        ui.text(x + 140, y, std::string(tel.abs_active ? "ABS " : "") + (tel.tcs_active ? "TCS " : "") +
                                (tel.shifting ? "SHIFT" : ""),
                colors::kBad);
        y += 12;
        float rpm_frac = st.engine_rpm / su.max_rpm;
        ui.bar(x, y, 196, 8, rpm_frac, st.rev_limiter ? colors::kBad : colors::kGood);
        float red = x + 196 * su.rev_limit_rpm / su.max_rpm;
        ui.fill(red, y - 1, 1, 10, colors::kBad);
        ui.text(x + 202, y, fmt("%5.0frpm", st.engine_rpm), colors::kText);
        y += 12;
        auto labelled_bar = [&](const char* name, float v, Color c) {
            ui.text(x, y, name, colors::kDim);
            ui.bar(x + 56, y, 120, 7, v, c);
            ui.text(x + 182, y, fmt("%.2f", v), colors::kDim);
            y += 10;
        };
        labelled_bar("gas", input_.throttle, colors::kGood);
        labelled_bar("eff.gas", st.throttle, colors::kGood);
        labelled_bar("brake", input_.brake, colors::kBad);
        labelled_bar("clutch", tel.clutch, colors::kAccent);
        ui.text(x, y, "steer", colors::kDim);
        ui.fill(x + 56, y, 120, 7, colors::kPanel);
        float sw = 60 * std::clamp(input_.steer, -1.0f, 1.0f);
        ui.fill(sw >= 0 ? x + 116 : x + 116 + sw, y, std::fabs(sw), 7, colors::kAccent);
        ui.text(x + 182, y, fmt("%+.1fdeg", tel.steer_angle * 57.2958f), colors::kDim);
        y += 12;
        ui.text(x, y, fmt("crank %4.0fNm  ", tel.engine_torque) + (tel.clutch_locked ? "clutch locked" : "clutch slip"),
                colors::kDim);
        y += 10;
        ui.text(x, y, fmt("drag %4.0fN  downforce %4.0fN", tel.drag_force, tel.downforce), colors::kDim);
        y += 14;

        // Per-wheel table.
        ui.text(x, y, "   load   slip  angle  rad/s mm", colors::kDim);
        y += 10;
        const char* names[] = {"FL", "FR", "RL", "RR"};
        for (int i = 0; i < 4; ++i) {
            const auto& w = st.wheels[size_t(i)];
            ui.text(x, y, names[i], w.grounded ? colors::kText : colors::kDim);
            ui.text(x + 24, y,
                    fmt("%4.1fk %+5.2f %+5.1f", w.load / 1000, w.slip_ratio, w.slip_angle * 57.2958f) +
                        fmt(" %5.0f %3.0f", w.angular_velocity, w.suspension * 1000),
                    colors::kText);
            y += 10;
        }
        y += 4;
        // Wheel loads as bars.
        for (int i = 0; i < 4; ++i) {
            float f = st.wheels[size_t(i)].load / (sp.mass * 9.81f * 0.5f);
            ui.bar(x + (i % 2) * 100, y + (i / 2) * 10, 90, 7, std::min(f, 1.0f), colors::kAccent);
        }
        y += 24;
        // G-meter.
        float gx = x + 40, gy = y + 36, gr = 34;
        ui.frame(gx - gr, gy - gr, gr * 2, gr * 2, colors::kHighlight);
        ui.fill(gx - gr, gy, gr * 2, 1, colors::kHighlight);
        ui.fill(gx, gy - gr, 1, gr * 2, colors::kHighlight);
        float ax = tel.accel_local.x / 9.81f, az = tel.accel_local.z / 9.81f; // z: + = backwards
        float dx = std::clamp(ax, -1.5f, 1.5f) * gr / 1.5f, dy = std::clamp(az, -1.5f, 1.5f) * gr / 1.5f;
        ui.fill(gx + dx - 2, gy + dy - 2, 5, 5, colors::kAccent);
        ui.text(gx + gr + 8, gy - 12, fmt("lat  %+.2fg", ax), colors::kText);
        ui.text(gx + gr + 8, gy, fmt("long %+.2fg", -az), colors::kText);
        ui.text(gx + gr + 8, gy + 12, fmt("yaw %+.0fdeg/s", st.angular_velocity.y * 57.2958f), colors::kDim);
        if (autodrive_) ui.text(x, gy + gr + 6, "autodrive (GT3R_SIM_AUTODRIVE)", colors::kDim);
    }

    bool loaded_ = false;
    bool have_db_ = false;
    bool autodrive_ = false;
    double prerun_ = 0;
    data::CarDatabase db_;
    std::vector<Entry> entries_;
    int sel_ = 0;
    Mode mode_ = Mode::List;
    std::unique_ptr<sim::Vehicle> car_;
    std::string name_, error_;
    FlatPlane plane_;
    double accum_ = 0;
    bool automatic_ = true;
    bool shift_up_ = false, shift_down_ = false;
    u16 prev_buttons_ = 0;
    sim::ControlInputs input_;
    std::deque<Vec3> trail_;
    int trail_tick_ = 0;
};

} // namespace

GT3R_REGISTER_SCREEN_NO_DISC(sim, "sim", "Physics test",
                             "Drive a car on a flat plane with live physics telemetry.", 75,
                             [] { return std::make_unique<SimScreen>(); });

} // namespace gt3r
