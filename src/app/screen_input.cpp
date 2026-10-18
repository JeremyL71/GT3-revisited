// Controller setup: live view of each player's virtual DualShock 2 and
// interactive rebinding. Shows pressure levels so analog-trigger-to-pressure
// mappings can be checked at a glance. Two pages per player: the menu buttons
// (the DualShock 2 itself) and the race controls (the race pad's actions,
// input/bindings.h make_race_preset).

#include "app/app.h"
#include "app/screen_registry.h"

#include <algorithm>
#include <cmath>

namespace gt3r {

namespace {

struct Row {
    PadTarget target;
};

class InputScreen : public Screen {
public:
    InputScreen() {
        for (size_t i = 0; i < kPadButtonCount; ++i) {
            PadTarget t;
            t.kind = PadTarget::Kind::Button;
            t.button = static_cast<PadButton>(i);
            menu_rows_.push_back({t});
        }
        for (size_t i = 0; i < kPadAxisCount; ++i) {
            PadTarget t;
            t.kind = PadTarget::Kind::Axis;
            t.axis = static_cast<PadAxis>(i);
            menu_rows_.push_back({t});
        }
        // Race actions in driving order.
        auto button = [](PadButton b) {
            PadTarget t;
            t.kind = PadTarget::Kind::Button;
            t.button = b;
            return Row{t};
        };
        PadTarget steer;
        steer.kind = PadTarget::Kind::Axis;
        steer.axis = PadAxis::LX;
        race_rows_ = {button(PadButton::Cross),  button(PadButton::Square), Row{steer},
                      button(PadButton::Left),   button(PadButton::Right),  button(PadButton::Circle),
                      button(PadButton::R2),     button(PadButton::L2),     button(PadButton::Triangle),
                      button(PadButton::R1),     button(PadButton::L1),     button(PadButton::Select),
                      button(PadButton::Start),  button(PadButton::R3)};
    }

    bool wants_menu_input() const override { return state_ == State::Idle; }

    void update(App& app, double dt) override {
        InputManager& in = app.input();
        if (state_ == State::WaitRelease) {
            const PadState& p = in.pad(player_);
            bool centered = true;
            for (u8 a : p.axes) centered &= std::abs(int(a) - 128) < 40;
            if (p.buttons == 0 && centered) {
                in.begin_capture();
                state_ = State::Capturing;
                timer_ = 6.0;
            }
            return;
        }
        if (state_ == State::Capturing) {
            timer_ -= dt;
            if (auto src = in.poll_capture()) {
                if (race_) in.bind_race(player_, rows()[sel_ - 1].target, *src, !append_);
                else in.bind(player_, rows()[sel_ - 1].target, *src, !append_);
                finish(app);
            } else if (timer_ <= 0) {
                in.cancel_capture();
                finish(app);
            }
            return;
        }

        const MenuInput& m = app.menu();
        if (m.pressed(PadButton::L2) || m.pressed(PadButton::R2)) race_ = !race_, sel_ = 0;
        int n = int(rows().size()) + 1;
        sel_ = (sel_ + m.vertical() + n) % n;
        if (m.pressed(PadButton::L1) || m.pressed(PadButton::R1)) player_ = (player_ + 1) % InputManager::kMaxPlayers;
        if (race_ && sel_ == 0 && m.horizontal()) {
            const auto& names = race_layout_names();
            int idx = 0;
            for (size_t i = 0; i < names.size(); ++i)
                if (names[i] == in.race_layout(player_)) idx = int(i);
            // From a custom profile, Left/Right first loads the layout it was made from.
            if (!in.is_race_custom(player_)) idx = (idx + m.horizontal() + int(names.size())) % int(names.size());
            in.set_race_layout(player_, names[idx]);
            app.save_config();
        } else if (sel_ == 0 && m.horizontal()) {
            const auto& names = preset_names();
            int idx = 0;
            for (size_t i = 0; i < names.size(); ++i)
                if (names[i] == in.preset(player_)) idx = int(i);
            idx = (idx + m.horizontal() + int(names.size())) % int(names.size());
            in.apply_preset(player_, names[idx]);
            app.save_config();
        }
        if (sel_ > 0 && (m.pressed(PadButton::Cross) || m.pressed(PadButton::Square))) {
            append_ = m.pressed(PadButton::Square);
            state_ = State::WaitRelease;
        }
        if (sel_ > 0 && m.pressed(PadButton::Select)) {
            if (race_) in.clear_race_target(player_, rows()[sel_ - 1].target);
            else in.clear_target(player_, rows()[sel_ - 1].target);
            app.save_config();
        }
        if (m.held(PadButton::Start)) in.set_rumble(player_, PadRumble{true, 200});
        else if (rumbling_) in.set_rumble(player_, PadRumble{});
        rumbling_ = m.held(PadButton::Start);
        if (m.pressed(PadButton::Triangle)) {
            app.save_config();
            app.pop();
        }
    }

    void render(App& app, Ui& ui) override {
        InputManager& in = app.input();
        ui.header(race_ ? "CONTROLLER SETUP - RACE CONTROLS" : "CONTROLLER SETUP - MENU BUTTONS",
                  "Player " + std::to_string(player_ + 1) + "  (L1/R1: player  L2/R2: page)");
        ui.text(8, 26, "Device: " + in.device_name(player_), colors::kDim);

        float y = 42;
        if (sel_ == 0) ui.fill(4, y - 2, Ui::kWidth - 8, 12, colors::kHighlight);
        if (race_) {
            std::string layout = in.race_layout(player_) == "gt3" ? "GT3 original (the menu buttons)" : "modern";
            if (in.is_race_custom(player_)) layout = "custom (Left/Right to load a layout)";
            ui.text(8, y, "Layout:", colors::kText);
            ui.text(100, y, "< " + layout + " >", colors::kAccent);
        } else {
            std::string preset = in.is_custom(player_) ? "custom (Left/Right to load a preset)" : in.preset(player_);
            ui.text(8, y, "Preset:", colors::kText);
            ui.text(100, y, "< " + preset + " >", colors::kAccent);
        }

        const PadState& pad = race_ ? in.race_pad(player_) : in.pad(player_);
        const Profile& prof = race_ ? in.race_profile(player_) : in.profile(player_);
        const std::vector<Row>& rs = rows();
        for (size_t r = 0; r < rs.size(); ++r) {
            y = 58 + r * 12.0f;
            const PadTarget& t = rs[r].target;
            bool selected = int(r) + 1 == sel_;
            if (selected) ui.fill(4, y - 2, Ui::kWidth - 8, 12, colors::kHighlight);
            if (race_) {
                ui.text(8, y, race_action_name(t), colors::kText);
                bool pressure = t.kind == PadTarget::Kind::Button && is_pressure_sensitive(t.button) &&
                                (t.button == PadButton::Cross || t.button == PadButton::Square || t.button == PadButton::Triangle);
                if (t.kind == PadTarget::Kind::Button) {
                    ui.bar(90, y, 80, 7, pad.press(t.button) / 255.0f, pad.down(t.button) ? colors::kGood : colors::kDim);
                    if (pressure) ui.text(174, y, std::to_string(pad.press(t.button)), colors::kDim);
                } else {
                    float v = pad.axis_f(t.axis);
                    ui.fill(90, y, 80, 7, colors::kPanel);
                    float cx = 130, w = 40 * std::clamp(v, -1.0f, 1.0f);
                    ui.fill(w >= 0 ? cx : cx + w, y, std::fabs(w), 7, colors::kAccent);
                    ui.frame(90, y, 80, 7, colors::kHighlight);
                }
            } else if (t.kind == PadTarget::Kind::Button) {
                ui.text(8, y, pad_button_name(t.button), colors::kText);
                float v = pad.press(t.button) / 255.0f;
                ui.bar(90, y, 80, 7, v, pad.down(t.button) ? colors::kGood : colors::kDim);
                if (is_pressure_sensitive(t.button)) ui.text(174, y, std::to_string(pad.press(t.button)), colors::kDim);
            } else {
                ui.text(8, y, pad_axis_name(t.axis), colors::kText);
                float v = pad.axis_f(t.axis);
                ui.fill(90, y, 80, 7, colors::kPanel);
                float cx = 130, w = 40 * std::clamp(v, -1.0f, 1.0f);
                ui.fill(w >= 0 ? cx : cx + w, y, std::fabs(w), 7, colors::kAccent);
                ui.frame(90, y, 80, 7, colors::kHighlight);
                ui.text(174, y, std::to_string(pad.axis(t.axis)), colors::kDim);
            }
            ui.text(206, y, bindings_for(app, prof, t), selected ? colors::kText : colors::kDim);
        }

        if (state_ != State::Idle) {
            ui.fill(120, 140, 400, 70, colors::kPanel);
            ui.frame(120, 140, 400, 70, colors::kAccent);
            const PadTarget& tt = rows()[sel_ - 1].target;
            std::string what = race_ ? std::string(race_action_name(tt)) : tt.to_string();
            if (state_ == State::WaitRelease) ui.text_center(160, "Release all inputs...", colors::kText);
            else {
                ui.text_center(160, std::string(append_ ? "Add input for " : "Press input for ") + what, colors::kAccent);
                ui.text_center(176, "Key, button, trigger, stick or wheel axis", colors::kDim);
                ui.text_center(190, "Cancels in " + std::to_string(int(std::ceil(timer_))) + "s", colors::kDim);
            }
        }
        ui.footer("Cross: rebind  Square: add  Select: clear  Start: test rumble  Triangle: back");
    }

private:
    enum class State { Idle, WaitRelease, Capturing };

    std::string bindings_for(App& app, const Profile& prof, const PadTarget& t) const {
        std::string out;
        for (const Binding& b : prof.bindings) {
            bool match = b.target.kind == t.kind &&
                         (t.kind == PadTarget::Kind::Button ? b.target.button == t.button : b.target.axis == t.axis);
            if (!match) continue;
            if (!out.empty()) out += ", ";
            if (b.target.kind == PadTarget::Kind::Axis && b.target.half != InputSource::Half::Full)
                out += b.target.half == InputSource::Half::Positive ? "+" : "-";
            out += app.input().describe(player_, b.source);
        }
        if (out.size() > 52) out = out.substr(0, 49) + "...";
        return out.empty() ? "(unbound)" : out;
    }

    void finish(App& app) {
        state_ = State::Idle;
        app.menu().suppress();
        app.save_config();
    }

    const std::vector<Row>& rows() const { return race_ ? race_rows_ : menu_rows_; }

    std::vector<Row> menu_rows_, race_rows_;
    bool race_ = false; // the race controls page
    int sel_ = 1;
    int player_ = 0;
    State state_ = State::Idle;
    bool append_ = false;
    bool rumbling_ = false;
    double timer_ = 0;
};

} // namespace

std::unique_ptr<Screen> make_input_screen() { return std::make_unique<InputScreen>(); }
GT3R_REGISTER_PLAYER_SCREEN_NO_DISC(input, "input", "Controller setup",
                     "Bind keyboards, gamepads and wheels to menu buttons and race controls, per player.", 4,
                     [] { return make_input_screen(); });

} // namespace gt3r
