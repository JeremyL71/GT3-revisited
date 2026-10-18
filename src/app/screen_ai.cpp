// AI test: opponents from a real event race each other on its course, seen
// from above with the AI lines and live timing (src/ai/, docs/dev/AI.md).
//
// Controls: list: Up/Down event, Cross start. Race: L1/R1 select car,
// Square zoom (whole track / follow), Select lines on/off, Circle pause,
// Start restart, Triangle events.
// GT3R_AI_EVENT=<label> picks the event; GT3R_AI_PRERUN=<seconds> simulates
// ahead at start (for --shot).

#include "ai/ai_race.h"
#include "app/app.h"
#include "app/screen_registry.h"
#include "game/data/car_database.h"
#include "track/track.h"

#include <SDL3/SDL_render.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

namespace gt3r {

namespace {

using sim::Vec3;

std::string fmt_time(double t) {
    if (t < 0) return "  --.---";
    char b[32];
    std::snprintf(b, sizeof b, "%d:%06.3f", int(t) / 60, std::fmod(t, 60.0));
    return b;
}

Vec3 rotate(const sim::Quat& q, const Vec3& v) {
    Vec3 u{q.x, q.y, q.z};
    Vec3 t = u.cross(v) * 2.0f;
    return v + t * q.w + u.cross(t);
}

const Color kCarColors[] = {{255, 196, 40}, {90, 200, 255}, {240, 90, 80}, {120, 230, 120},
                            {220, 140, 255}, {255, 150, 70}, {200, 200, 200}, {80, 140, 255}};
const Color kLineColors[8] = {{0, 0, 0},       {90, 120, 255}, {255, 110, 110}, {220, 220, 80},
                              {220, 160, 60},  {80, 80, 80},   {90, 230, 120},  {160, 160, 220}};

class AiScreen : public Screen {
public:
    bool wants_menu_input() const override { return true; }

    void update(App& app, double dt) override {
        if (!loaded_) load(app);
        const MenuInput& m = app.menu();
        if (mode_ == Mode::List) {
            int n = int(events_.size());
            if (n) sel_ = (sel_ + m.vertical() + n) % n;
            if (n && m.pressed(PadButton::Cross)) start(app, sel_);
            if (m.pressed(PadButton::Triangle)) {
                if (race_) mode_ = Mode::Race;
                else app.pop();
            }
            return;
        }
        if (m.pressed(PadButton::Triangle)) {
            mode_ = Mode::List;
            return;
        }
        if (!race_) return;
        if (m.pressed(PadButton::Start)) start(app, cur_);
        if (m.pressed(PadButton::Circle)) paused_ = !paused_;
        if (m.pressed(PadButton::Square)) follow_ = !follow_;
        if (m.pressed(PadButton::Select)) show_lines_ = !show_lines_;
        if (m.pressed(PadButton::R1)) focus_ = (focus_ + 1) % race_->count();
        if (m.pressed(PadButton::L1)) focus_ = (focus_ + race_->count() - 1) % race_->count();
        if (paused_) return;
        accum_ += std::min(dt, 0.1);
        const double tick = 1.0 / sim::kDefaultTickHz;
        while (accum_ >= tick) {
            accum_ -= tick;
            race_->step();
        }
    }

    void render(App& app, Ui& ui) override {
        (void)app;
        if (mode_ == Mode::List || !race_) {
            render_list(ui);
            return;
        }
        render_map(ui);
        render_table(ui);
        ui.footer("L1/R1: car  Square: zoom  Select: lines  O: pause  Start: restart  Tri: events");
    }

private:
    enum class Mode { List, Race };
    struct EventEntry {
        std::string label, course;
        int laps = 0, cars = 0;
    };

    void load(App& app) {
        loaded_ = true;
        if (!app.vfs() || !db_.load(*app.vfs())) {
            error_ = app.vfs() ? "database: " + db_.error() : "no disc";
            return;
        }
        const char* want = std::getenv("GT3R_AI_EVENT");
        int def = -1;
        for (const auto& ev : db_.events()) {
            int cars = 0;
            for (const auto& e : ev.entries) cars += !e.empty() && e != "-";
            if (cars < 2 || ev.course.empty() || ev.course.rfind("L_", 0) == 0) continue;
            if ((want && ev.label == want) || (!want && def < 0 && ev.label == "Gsunday_e_0001")) def = int(events_.size());
            events_.push_back({ev.label, ev.course, ev.laps, cars});
        }
        if (def < 0) def = 0;
        sel_ = def;
        if (!events_.empty()) {
            start(app, def);
            if (const char* pre = std::getenv("GT3R_AI_PRERUN"))
                for (int i = 0; race_ && i < int(std::atof(pre) * sim::kDefaultTickHz); ++i) race_->step();
        }
    }

    void start(App& app, int idx) {
        cur_ = idx;
        const EventEntry& ee = events_[size_t(idx)];
        const data::Event* ev = db_.event(ee.label);
        race_.reset();
        error_.clear();
        try {
            track_ = track::Track::load(*app.vfs(), ee.course);
        } catch (const std::exception& e) {
            error_ = ee.course + ": " + e.what();
            mode_ = Mode::List;
            return;
        }
        std::vector<ai::AiRace::Entry> entries;
        for (const auto& label : ev->entries) {
            if (label.empty() || label == "-") continue;
            ai::DriverParams p = ai::DriverParams::builtin(0);
            ai::apply_event(p, *ev, 0);
            entries.push_back({label, p});
            if (entries.size() == 8) break;
        }
        // Car 0 stands in for the player (rubber-band reference).
        race_ = std::make_unique<ai::AiRace>(*track_, db_, entries, ai::RubberBand::from_event(*ev), 0);
        if (!race_->error().empty()) error_ = race_->error();
        lines_.clear();
        auto course = std::make_unique<ai::AiCourse>(*track_);
        for (int l = 1; l <= 7; ++l) {
            std::vector<Vec3> pts;
            if (const auto* line = course->line(l)) {
                float a = line->points.front().progress, len = track_->length();
                float span = line->lap ? len : course->ahead(a, line->points.back().progress);
                for (float d = 0; d <= span; d += 6) pts.push_back(course->evaluate(l, a + d));
            }
            lines_.push_back(pts);
        }
        // Map bounds.
        bx0_ = bz0_ = 1e30f, bx1_ = bz1_ = -1e30f;
        for (const auto& w : track_->walls())
            for (const auto& p : w) bx0_ = std::min(bx0_, p.x), bx1_ = std::max(bx1_, p.x), bz0_ = std::min(bz0_, p.z), bz1_ = std::max(bz1_, p.z);
        focus_ = 0;
        accum_ = 0;
        mode_ = Mode::Race;
    }

    void render_list(Ui& ui) {
        ui.header("AI TEST", "Choose an event (Cross)  " + std::to_string(events_.size()) + " events with opponents");
        const int rows = 24;
        int first = ui.list_window(int(events_.size()), sel_, rows);
        for (int r = 0; r < rows && first + r < int(events_.size()); ++r) {
            const auto& e = events_[size_t(first + r)];
            const Color c = first + r == sel_ ? colors::kAccent : colors::kText;
            const float y = 30 + r * 12.0f;
            ui.text(12, y, Ui::clip(e.label, 180), c);
            ui.text(200, y, Ui::clip(e.course, 130), c);
            ui.text_right(360, y, std::to_string(e.laps) + " laps", c);
            ui.text(376, y, std::to_string(e.cars) + " cars", c);
        }
        if (!error_.empty()) ui.text(12, 330, error_, colors::kBad);
        ui.footer("Up/Down: select  Cross: race  Triangle: back");
    }

    void line(Ui& ui, float x0, float y0, float x1, float y1, Color c) {
        SDL_SetRenderDrawColor(ui.renderer(), c.r, c.g, c.b, c.a);
        SDL_RenderLine(ui.renderer(), x0, y0, x1, y1);
    }

    void render_map(Ui& ui) {
        const float vx0 = 4, vy0 = 14, vw = 400, vh = 330;
        ui.fill(vx0, vy0, vw, vh, Color{18, 22, 20});
        float cx, cz, scale;
        if (follow_) {
            Vec3 p = race_->car(focus_).vehicle->state().position;
            cx = p.x, cz = p.z, scale = 2.5f;
        } else {
            cx = 0.5f * (bx0_ + bx1_), cz = 0.5f * (bz0_ + bz1_);
            scale = std::min(vw / std::max(bx1_ - bx0_, 1.0f), vh / std::max(bz1_ - bz0_, 1.0f)) * 0.95f;
        }
        auto sx = [&](float x) { return vx0 + vw * 0.5f + (x - cx) * scale; };
        auto sy = [&](float z) { return vy0 + vh * 0.5f + (z - cz) * scale; };
        auto inside = [&](float x, float y) { return x >= vx0 && x <= vx0 + vw && y >= vy0 && y <= vy0 + vh; };
        auto seg = [&](const Vec3& a, const Vec3& b, Color c) {
            float x0 = sx(a.x), y0 = sy(a.z), x1 = sx(b.x), y1 = sy(b.z);
            if (inside(x0, y0) && inside(x1, y1)) line(ui, x0, y0, x1, y1, c);
        };
        for (const auto& w : track_->walls())
            for (size_t i = 1; i < w.size(); ++i)
                seg({w[i - 1].x, 0, w[i - 1].z}, {w[i].x, 0, w[i].z}, Color{70, 80, 75});
        if (show_lines_)
            for (size_t l = 0; l < lines_.size(); ++l) {
                if (l + 1 == ai::AiCourse::kSideMap || l + 1 == ai::AiCourse::kPitStop) continue;
                for (size_t i = 1; i < lines_[l].size(); ++i) seg(lines_[l][i - 1], lines_[l][i], kLineColors[l + 1]);
            }
        for (int i = 0; i < race_->count(); ++i) {
            const auto& c = race_->car(i);
            const auto& st = c.vehicle->state();
            Vec3 f = rotate(st.orientation, {0, 0, -1}), r = rotate(st.orientation, {1, 0, 0});
            float hl = std::max(c.vehicle->setup().length * 0.5f, 3.0f / scale), hw = std::max(c.vehicle->setup().width * 0.5f, 1.5f / scale);
            Vec3 p = st.position, q[4] = {p + f * hl + r * hw, p + f * hl - r * hw, p - f * hl - r * hw, p - f * hl + r * hw};
            Color col = kCarColors[i % 8];
            for (int k = 0; k < 4; ++k) seg(q[k], q[(k + 1) % 4], col);
            seg(p, p + f * (hl * 1.8f), col);
            if (i == focus_) {
                const auto& d = c.driver->debug();
                seg(p, d.target, Color{255, 255, 255, 120});
                float x = sx(p.x), y = sy(p.z);
                if (inside(x, y)) ui.frame(x - 6, y - 6, 12, 12, colors::kText);
            }
        }
        ui.frame(vx0, vy0, vw, vh, colors::kHighlight);
        char b[96];
        std::snprintf(b, sizeof b, "%s  %s  t %s%s", events_[size_t(cur_)].label.c_str(), track_->label().c_str(),
                      fmt_time(race_->time()).c_str(), paused_ ? "  PAUSED" : "");
        ui.text(vx0 + 4, vy0 + 4, b, colors::kDim);
        if (show_lines_)
            ui.text(vx0 + 4, vy0 + vh - 12, "lines: green main(6) blue A(1) red B(2) yellow pit(3)", colors::kDim);
    }

    void render_table(Ui& ui) {
        float x = 410, y = 14;
        // Columns: position, car, laps, best lap.
        const float cx = x + 16, lx = x + 112, bx = x + 120;
        ui.text(x + 4, y, "P", colors::kDim);
        ui.text(cx, y, "car", colors::kDim);
        ui.text_right(lx, y, "lap", colors::kDim);
        ui.text(bx, y, "best", colors::kDim);
        y += 11;
        int pos = 1;
        for (int i : race_->ranking()) {
            const auto& c = race_->car(i);
            std::string name = c.label.rfind("sun_e_", 0) == 0 || c.label.rfind("clb_e_", 0) == 0 ? c.label.substr(6) : c.label;
            const Color tc = i == focus_ ? colors::kAccent : colors::kText;
            ui.fill(x - 2, y + 1, 4, 6, kCarColors[i % 8]);
            ui.text(x + 4, y, std::to_string(pos++), tc);
            ui.text(cx, y, Ui::clip(name, lx - cx - 20), tc);
            ui.text_right(lx, y, std::to_string(c.timer->laps_completed()), tc);
            ui.text(bx, y, fmt_time(c.timer->best_lap_time()), tc);
            y += 10;
        }
        y += 8;
        const auto& c = race_->car(focus_);
        const auto& d = c.driver->debug();
        const auto& st = c.vehicle->state();
        auto row = [&](const char* fmt, double a, double b2 = 0, double c2 = 0) {
            char buf[96];
            std::snprintf(buf, sizeof buf, fmt, a, b2, c2);
            ui.text(x, y, buf, colors::kText);
            y += 10;
        };
        ui.text(x, y, c.label.substr(0, 27), colors::kAccent);
        y += 11;
        const char* modes[] = {"straight", "arc", "arc", "brake", "brake(arc)"};
        row("%.0f km/h  gear %.0f  dt x%.2f", st.speed_kmh, st.gear, c.time_scale);
        ui.text(x, y, "last lap " + fmt_time(c.timer->last_lap_time()), colors::kText);
        y += 10;
        ui.text(x, y, std::string("mode ") + modes[std::clamp(d.mode, 0, 4)] + "  line " + std::to_string(d.line) +
                          (d.blend > 0 ? " <" + std::to_string(d.prev_line) : "") + "  rec " + std::to_string(d.state),
                colors::kText);
        y += 10;
        row("target %.0f  next %.0f km/h", d.target_speed * 3.6, d.next_speed * 3.6);
        row("brake dist %.0f / %.0f m", d.brake_distance, d.distance_to_point);
        row("look %.0f m  err %+.1f deg", d.lookahead, d.heading_error * 57.2958);
        row("slip %+.1f deg  sat %.2f", d.slip_wanted * 57.2958, d.saturation);
        ui.text(x, y, "thr", colors::kDim);
        ui.bar(x + 28, y, 80, 7, d.throttle, colors::kGood);
        ui.text(x + 112, y, "brk", colors::kDim);
        ui.bar(x + 140, y, 80, 7, d.brake, colors::kBad);
        y += 10;
        row("contact %.2f  zones %02X", d.contact, d.zones);
        row("respawns %.0f  car contacts %.0f", c.respawns, race_->contacts());
        if (!error_.empty()) ui.text(x, 330, error_.substr(0, 28), colors::kBad);
    }

    bool loaded_ = false;
    data::CarDatabase db_;
    std::vector<EventEntry> events_;
    int sel_ = 0, cur_ = 0;
    Mode mode_ = Mode::List;
    std::unique_ptr<track::Track> track_;
    std::unique_ptr<ai::AiRace> race_;
    std::vector<std::vector<Vec3>> lines_;
    float bx0_ = 0, bz0_ = 0, bx1_ = 1, bz1_ = 1;
    int focus_ = 0;
    bool follow_ = false, show_lines_ = true, paused_ = false;
    double accum_ = 0;
    std::string error_;
};

} // namespace

GT3R_REGISTER_SCREEN(ai, "ai", "AI test", "Opponents of a real event racing each other, with AI lines and timing.", 76,
                     [] { return std::make_unique<AiScreen>(); });

} // namespace gt3r
