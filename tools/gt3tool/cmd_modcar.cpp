// gt3tool mod-car commands: check car definitions (mods/<id>/cars/*.ini, see
// docs/MODDING.md "Adding cars") against the disc and test-drive them.
//
//   gt3tool <iso> modcar <file.ini | mod dir | mods dir> ...
//
// For each car: load errors and warnings, the resulting catalogue figures and
// physics spec, then a straight-line run on a flat plane (0-60 mph, 0-100
// km/h, quarter mile, top speed) next to the [reference] figures in the file.

#include "tool.h"

#include "base/config.h"
#include "base/file.h"
#include "game/data/car_database.h"
#include "game/data/mod_cars.h"
#include "game/gt_mode.h"
#include "sim/car_spec.h"
#include "sim/vehicle.h"

#include <SDL3/SDL_filesystem.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace gt3r::tool {

namespace {

struct Flat : sim::TrackQuery {
    sim::SurfaceHit probe(float, float, float y_max) const override {
        sim::SurfaceHit h;
        h.hit = y_max >= 0;
        return h;
    }
};

std::vector<std::string> list_dir(const std::string& dir) {
    std::vector<std::string> out;
    SDL_EnumerateDirectory(
        dir.c_str(),
        [](void* user, const char*, const char* name) {
            static_cast<std::vector<std::string>*>(user)->push_back(name);
            return SDL_ENUM_CONTINUE;
        },
        &out);
    std::sort(out.begin(), out.end());
    return out;
}

bool is_ini(const std::string& n) { return n.size() > 4 && to_lower(n.substr(n.size() - 4)) == ".ini"; }

// A .ini file, a mod (has cars/), or a mods folder (holds mods).
void collect(const std::string& path, std::vector<std::string>& out) {
    if (path.rfind("disc:", 0) == 0) {
        out.push_back(path);
        return;
    }
    if (!host_is_directory(path)) {
        out.push_back(path);
        return;
    }
    if (host_is_directory(path + "/cars")) {
        for (const std::string& n : list_dir(path + "/cars"))
            if (is_ini(n)) out.push_back(path + "/cars/" + n);
        return;
    }
    for (const std::string& n : list_dir(path)) {
        if (is_ini(n)) out.push_back(path + "/" + n);
        else if (host_is_directory(path + "/" + n + "/cars")) collect(path + "/" + n, out);
    }
}

struct Run {
    double t_roll = -1, t60 = -1, t100 = -1, t400 = -1, trap_kmh = 0, top_kmh = 0, t_top = 0;
    double spin_kmh = 0, spin_at_kmh = 0; // peak driven-wheel rim speed over road speed before 60 mph (TCS on)
};

Run straight_line(const sim::CarSpec& spec) {
    auto car = sim::Vehicle::create(spec);
    Flat g;
    car->reset({}, {});
    sim::ControlInputs in;
    in.throttle = 1;
    Run r;
    const double h = 1.0 / sim::kDefaultTickHz;
    sim::Vec3 start = car->state().position;
    float best = 0;
    for (int i = 0; i < int(120 / h); ++i) {
        car->step(in, g, h);
        double t = (i + 1) * h;
        float kmh = car->state().speed_kmh;
        if (r.t_roll < 0 && (car->state().position - start).length() >= 0.3048f) r.t_roll = t;
        if (r.t60 < 0)
            for (int w = 0; w < 4; ++w)
                if (double d = car->state().wheels[w].angular_velocity * car->setup().wheels[w].radius * 3.6f - kmh; d > r.spin_kmh)
                    r.spin_kmh = d, r.spin_at_kmh = kmh;
        if (r.t60 < 0 && kmh >= 96.56f) r.t60 = t;
        if (r.t100 < 0 && kmh >= 100) r.t100 = t;
        if (r.t400 < 0 && (car->state().position - start).length() >= 402.34f) r.t400 = t, r.trap_kmh = kmh;
        if (kmh > best + 0.05f) best = kmh, r.t_top = t;
        if (t - r.t_top > 10 && r.t400 > 0) break; // no longer gaining
    }
    r.top_kmh = best;
    return r;
}

// Lift-off from 100 km/h on the flat: average deceleration 100->50 and 50->10
// km/h (g), and where the car ends up after 25 s.
struct Coast {
    double g_high = 0, g_low = 0, end_kmh = 0;
};

Coast coast_down(const sim::CarSpec& spec) {
    auto car = sim::Vehicle::create(spec);
    Flat g;
    car->reset({}, {});
    sim::ControlInputs in;
    in.throttle = 1;
    const double h = 1.0 / sim::kDefaultTickHz;
    for (int i = 0; i < int(60 / h) && car->state().speed_kmh < 100; ++i) car->step(in, g, h);
    in.throttle = 0;
    Coast c;
    double t = 0, t50 = -1, t10 = -1;
    for (int i = 0; i < int(25 / h); ++i) {
        car->step(in, g, h);
        t += h;
        float v = car->state().speed_kmh;
        if (t50 < 0 && v <= 50) t50 = t;
        if (t10 < 0 && v <= 10) t10 = t;
        if (std::getenv("GT3R_MODCAR_DEBUG") && i % 30 == 0) {
            const auto& tl = car->telemetry();
            std::printf("     t %.1f  %.1f km/h  motor %.0f rpm  T %.0f N*m  clutch %.2f locked %d  gear %d\n", t, v,
                        car->state().engine_rpm, tl.engine_torque, tl.clutch, int(tl.clutch_locked), car->state().gear);
        }
    }
    if (t50 > 0) c.g_high = 50 / 3.6 / t50 / 9.81;
    if (t10 > 0 && t50 > 0) c.g_low = 40 / 3.6 / (t10 - t50) / 9.81;
    c.end_kmh = car->state().speed_kmh;
    return c;
}

// Skidpad, like Car and Driver's 300-ft circle: a controller holds the circle's curvature with
// the road-wheel angle while the speed ramps up slowly; the result is the highest lateral
// acceleration held for a second.
double skidpad_g(const sim::CarSpec& spec, double radius = 45.72) {
    auto car = sim::Vehicle::create(spec);
    Flat g;
    car->reset({}, {});
    const double h = 1.0 / sim::kDefaultTickHz;
    sim::ControlInputs in;
    double steer = 0, thr_i = 0.2, best = 0, held = 0, held_acc = 0;
    int held_n = 0;
    double v_target = 30 / 3.6;
    for (int i = 0; i < int(240 / h); ++i) {
        const auto& st = car->state();
        double v = std::hypot(st.velocity.x, st.velocity.z);
        double r = st.angular_velocity.y;
        double k = v > 1 ? std::fabs(r) / v : 0;
        if (v > 3) steer = std::clamp(steer + 0.6 * (1 / radius - k) * h * 60 * 0.02, 0.0, 0.6);
        else steer = 0.05;
        double e = v_target - v;
        thr_i = std::clamp(thr_i + 0.15 * e * h, 0.0, 1.0);
        in.throttle = float(std::clamp(thr_i + 0.4 * e, 0.0, 1.0));
        in.brake = 0;
        in.steer_angle = float(steer);
        car->step(in, g, h);
        if (i * h > 6) v_target += 1.0 / 3.6 * h; // +1 km/h per second once settled
        bool on_circle = v > 5 && std::fabs(k * radius - 1) < 0.04;
        double a = v * std::fabs(r) / 9.81;
        if (on_circle) {
            held += h, held_acc += a, ++held_n;
            if (held >= 1.0) {
                best = std::max(best, held_acc / held_n);
                held = 0, held_acc = 0, held_n = 0;
            }
        } else {
            held = 0, held_acc = 0, held_n = 0;
            if (i * h > 20 && steer >= 0.59) break; // out of lock: past the limit
        }
        if (v_target - v > 4 && i * h > 20) break; // can no longer hold the speed on the circle
    }
    return best;
}

// 60-0 mph stop with full brakes (ABS), feet.
double brake_60_0_ft(const sim::CarSpec& spec) {
    auto car = sim::Vehicle::create(spec);
    Flat g;
    car->reset({}, {});
    const double h = 1.0 / sim::kDefaultTickHz;
    sim::ControlInputs in;
    in.throttle = 1;
    for (int i = 0; i < int(60 / h) && car->state().speed_kmh < 100; ++i) car->step(in, g, h);
    in.throttle = 0;
    in.brake = 1;
    double dist = 0;
    bool started = false;
    for (int i = 0; i < int(20 / h); ++i) {
        float v = car->state().speed_kmh;
        if (!started && v <= 96.56f) started = true;
        car->step(in, g, h);
        if (started) dist += car->state().speed_kmh / 3.6 * h;
        if (car->state().speed_kmh < 0.5) break;
    }
    return dist * 3.28084;
}

std::string secs(double t) {
    char b[32];
    std::snprintf(b, sizeof(b), t < 0 ? "  -  " : "%5.2f", t);
    return b;
}

int cmd_modcar(Disc& disc, const Args& a) {
    std::vector<std::string> files;
    for (const std::string& p : a) {
        if (p.rfind("arcade:", 0) == 0) { // every arcade car of a class: arcade:CLASS_S
            game::GtData gd;
            Volume& vol = disc.volume();
            if (!gd.load([&](const std::string& path) -> std::optional<Bytes> {
                    const Volume::Entry* e = vol.find(path);
                    if (!e) return std::nullopt;
                    return vol.read(*e);
                })) {
                std::fprintf(stderr, "error: %s\n", gd.error().c_str());
                return 1;
            }
            auto it = gd.arcade_classes.find(p.substr(7));
            if (it == gd.arcade_classes.end()) continue;
            for (const auto& c : it->second) files.push_back("disc:" + c.label);
            continue;
        }
        collect(p, files);
    }
    if (files.empty()) {
        std::fprintf(stderr, "no car definitions found\n");
        return 1;
    }
    data::CarDatabase db;
    if (!db.load(disc.volume())) {
        std::fprintf(stderr, "error: %s\n", db.error().c_str());
        return 1;
    }
    int bad = 0;
    for (const std::string& f : files) {
        std::printf("== %s\n", f.c_str());
        if (f.rfind("disc:", 0) == 0) { // a disc car, for comparison
            std::string serr;
            auto spec = sim::build_car_spec(db, f.substr(5), &serr);
            if (!spec) {
                std::printf("   error: %s\n", serr.c_str());
                continue;
            }
            if (std::getenv("GT3R_MODCAR_DEBUG")) { // at rest: where the car frame and wheels sit
            auto car = sim::Vehicle::create(*spec);
            Flat g;
            car->reset({}, {});
            sim::ControlInputs idle;
            for (int i = 0; i < 180; ++i) car->step(idle, g, 1.0 / 60);
            const auto& st = car->state();
            const auto& su = car->setup();
            std::printf("   at rest: origin y %.4f, body lift (ride height) %.3f, wheel FL hub %.3f (rest %.3f, susp %.4f static %.4f), tyre r %.3f\n",
                        st.position.y, su.wheels[0].ride_height, st.position.y + su.wheels[0].rest_center.y + (st.wheels[0].suspension - su.wheels[0].static_suspension),
                        su.wheels[0].rest_center.y, st.wheels[0].suspension, su.wheels[0].static_suspension, spec->axle[0].tire_radius);
        }
        Run r = straight_line(*spec);
            std::printf("   %s: %.0f kg, %.0f kW, %.2f kg/PS\n", spec->name.c_str(), spec->mass, spec->peak_power_kw(),
                        (spec->mass - 120) / std::max(1.0f, spec->peak_power_kw() * 1.35962f));
            std::printf("   0-60 mph  %s s, %s s with a 1 ft rollout\n", secs(r.t60).c_str(), secs(r.t60 - r.t_roll).c_str());
            std::printf("   1/4 mile  %s s @ %.0f mph\n", secs(r.t400).c_str(), r.trap_kmh / 1.609344);
            std::printf("   top speed %.0f km/h\n", r.top_kmh);
            std::printf("   skidpad   %.2f g (300 ft)\n", skidpad_g(*spec));
            std::printf("   60-0 mph  %.0f ft\n", brake_60_0_ft(*spec));
            continue;
        }
        auto bytes = read_host_file(f);
        if (!bytes) {
            std::printf("   error: cannot read\n");
            ++bad;
            continue;
        }
        std::string text(reinterpret_cast<const char*>(bytes->data()), bytes->size()), err;
        std::vector<std::string> warns;
        auto def = data::parse_mod_car(text, &err, &warns);
        for (const std::string& w : warns) std::printf("   warning: %s\n", w.c_str());
        if (def) def->file = f;
        if (!def || !data::add_mod_car(db, *def, &err)) {
            std::printf("   error: %s\n", err.c_str());
            ++bad;
            continue;
        }
        db.rebuild();
        const data::Car* c = db.car(def->label);
        auto s = db.summarize(*c);
        std::printf("   %s %s (%u) [%s], %u Cr, donor %s, shell %s\n", s.maker.c_str(), s.name.c_str(), s.year,
                    def->label.c_str(), s.price, def->donor.c_str(), s.model.c_str());
        std::printf("   %s %s, %u PS (%.0f kW), %.1f kgfm, %u kg, %u x %u x %u mm, wb %u, front %u%%\n", s.drive.c_str(),
                    s.aspiration.c_str(), s.power_ps, s.power_ps / 1.35962f, s.torque_kgfm, s.weight_kg, s.length_mm,
                    s.width_mm, s.height_mm, s.wheelbase_mm, s.front_weight_pct);
        std::string serr;
        auto spec = sim::build_car_spec(db, def->label, &serr);
        if (!spec) {
            std::printf("   error: spec: %s\n", serr.c_str());
            ++bad;
            continue;
        }
        float total = spec->gearbox.ratios.empty() ? 0 : spec->gearbox.ratios[0] * spec->gearbox.final_drive;
        std::printf("   spec: %.0f kg, %s, peak %.0f N*m, %.0f kW @ %.0f rpm, limit %.0f rpm, %zu gear(s), 1st x final %.2f, "
                    "tyres r %.3f/%.3f m, CdA %.2f m^2\n",
                    spec->mass, spec->engine.electric ? "electric" : "engine", spec->peak_torque(), spec->peak_power_kw(),
                    spec->peak_power_rpm(), spec->engine.rev_limit_rpm, spec->gearbox.ratios.size(), total,
                    spec->axle[0].tire_radius, spec->axle[1].tire_radius, spec->aero.drag_area);
        if (spec->engine.electric)
            std::printf("   regen: %.0f N*m, %.0f kW\n", spec->engine.regen_torque, spec->engine.regen_power / 1000);
        Run r = straight_line(*spec);
        IniDocument ini = IniDocument::parse(text);
        auto ref = [&](const char* k) {
            auto v = ini.get("reference", k);
            if (!v) return std::string();
            std::string t(trim(*v));
            if (size_t p = t.find(" ;"); p != std::string::npos) t = std::string(trim(t.substr(0, p)));
            return "   (real " + t + ")";
        };
        std::printf("   0-60 mph  %s s, %s s with a 1 ft rollout like US magazine tests%s\n", secs(r.t60).c_str(),
                    secs(r.t60 - r.t_roll).c_str(), ref("zero_to_60_s").c_str());
        std::printf("   launch    wheelspin peaks %.0f km/h over road speed at %.0f km/h (TCS on)\n", r.spin_kmh, r.spin_at_kmh);
        std::printf("   0-100 km/h%s s%s\n", secs(r.t100).c_str(), ref("zero_to_100_s").c_str());
        std::printf("   1/4 mile  %s s @ %.0f mph (rollout %s s)%s\n", secs(r.t400).c_str(), r.trap_kmh / 1.609344,
                    secs(r.t400 - r.t_roll).c_str(), ref("quarter_mile_s").c_str());
        std::printf("   top speed %.0f km/h (%.0f mph)%s\n", r.top_kmh, r.top_kmh / 1.609344, ref("top_speed_kmh").c_str());
        std::printf("   skidpad   %.2f g (300 ft)%s\n", skidpad_g(*spec), ref("skidpad_g").c_str());
        std::printf("   60-0 mph  %.0f ft%s\n", brake_60_0_ft(*spec), ref("brake_60_0_ft").c_str());
        Coast cd = coast_down(*spec);
        std::printf("   lift-off  %.2f g (100-50 km/h), %.2f g (50-10), %.1f km/h after 25 s\n", cd.g_high, cd.g_low, cd.end_kmh);
    }
    for (const std::string& e : db.mod_car_errors()) std::printf("error: %s\n", e.c_str());
    return bad ? 1 : 0;
}

} // namespace

GT3TOOL_COMMAND(modcar, "modcar", "<car.ini | mod dir | mods dir | disc:<label>>...", "check and test-drive mod car definitions (disc cars for comparison)", 1,
                cmd_modcar);

} // namespace gt3r::tool
