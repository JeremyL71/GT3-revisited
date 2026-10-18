// On-screen touch controls (input/touch_controls.h): layout hit-testing, pad
// output, visibility rules and the merge into InputManager.

#include "test_framework.h"

#include "base/config.h"
#include "input/input_manager.h"
#include "input/touch_controls.h"

#include <cmath>

using namespace gt3r;

namespace {

using Widget = TouchControls::Widget;

const Widget* find_widget(const TouchControls& t, Widget::Kind kind, PadButton b = PadButton::Count) {
    for (const Widget& w : t.widgets())
        if (w.kind == kind && (b == PadButton::Count || w.button == b)) return &w;
    return nullptr;
}

TouchControls driving(TouchConfig cfg = {}) {
    TouchControls t;
    t.configure(cfg);
    t.set_layout(TouchLayout::Driving);
    return t;
}

} // namespace

TEST(touch_hidden_until_touched) {
    TouchControls t = driving();
    CHECK(!t.visible());
    t.update(0.016, 0);
    CHECK(!t.pad().connected);
    const Widget* gas = find_widget(t, Widget::Kind::Pedal, PadButton::Cross);
    CHECK(gas != nullptr);
    // The first touch both shows the controls and acts.
    CHECK(t.finger_down(1, gas->x + gas->w / 2, gas->y + gas->h / 2, 1.0f));
    CHECK(t.visible());
    t.update(0.016, 0);
    CHECK(t.pad().connected);
    CHECK(t.pad().down(PadButton::Cross));
    // Gamepad / keyboard input hides them and releases everything.
    t.note_other_input();
    CHECK(!t.visible());
    t.update(0.016, 0);
    CHECK(!t.pad().down(PadButton::Cross));
    CHECK(!t.pad().connected);
}

TEST(touch_modes) {
    TouchConfig cfg;
    cfg.mode = TouchConfig::Mode::On;
    TouchControls on = driving(cfg);
    CHECK(on.visible());
    cfg.mode = TouchConfig::Mode::Off;
    TouchControls off = driving(cfg);
    off.note_touch();
    CHECK(!off.visible());
    CHECK(!off.finger_down(1, 600, 300, 1.0f));
    TouchControls none;
    none.note_touch();
    CHECK(!none.visible()); // TouchLayout::None
}

TEST(touch_pedal_pressure_from_position) {
    TouchControls t = driving();
    const Widget gas = *find_widget(t, Widget::Kind::Pedal, PadButton::Cross);
    t.finger_down(1, gas.x + gas.w / 2, gas.y + gas.h - 1, 1.0f); // bottom edge
    t.update(0.016, 0);
    u8 low = t.pad().press(PadButton::Cross);
    t.finger_move(1, gas.x + gas.w / 2, gas.y + 2, 1.0f); // top
    t.update(0.016, 0);
    u8 high = t.pad().press(PadButton::Cross);
    CHECK(low > 60 && low < 120); // pedal_min 0.35
    CHECK(high == 255);
    CHECK(t.pad().down(PadButton::Cross));

    TouchConfig full;
    full.pedals = TouchConfig::Pedals::Full;
    TouchControls f = driving(full);
    f.finger_down(1, gas.x + gas.w / 2, gas.y + gas.h - 1, 0.2f);
    f.update(0.016, 0);
    CHECK(f.pad().press(PadButton::Cross) == 255);

    TouchConfig area;
    area.pedals = TouchConfig::Pedals::Area;
    TouchControls a = driving(area);
    a.finger_down(1, gas.x + gas.w / 2, gas.y + 2, 0.6f);
    a.update(0.016, 0);
    CHECK(std::abs(int(a.pad().press(PadButton::Cross)) - 153) <= 1);
}

TEST(touch_slide_between_pedals) {
    TouchControls t = driving();
    const Widget gas = *find_widget(t, Widget::Kind::Pedal, PadButton::Cross);
    const Widget brake = *find_widget(t, Widget::Kind::Pedal, PadButton::Square);
    t.finger_down(7, brake.x + brake.w / 2, brake.y + brake.h / 2, 1.0f);
    t.update(0.016, 0);
    CHECK(t.pad().down(PadButton::Square));
    CHECK(!t.pad().down(PadButton::Cross));
    t.finger_move(7, gas.x + gas.w / 2, gas.y + gas.h / 2, 1.0f);
    t.update(0.016, 0);
    CHECK(t.pad().down(PadButton::Cross));
    CHECK(!t.pad().down(PadButton::Square));
    t.finger_up(7);
    t.update(0.016, 0);
    CHECK(t.pad().buttons == 0);
}

TEST(touch_drag_steering_and_multitouch) {
    TouchConfig cfg;
    TouchControls t = driving(cfg);
    const Widget gas = *find_widget(t, Widget::Kind::Pedal, PadButton::Cross);
    const float range = cfg.steer_range * 640.0f;
    CHECK(t.finger_down(1, 120, 260, 1.0f)); // steering zone (left half)
    CHECK(t.finger_down(2, gas.x + 5, gas.y + 5, 1.0f));
    t.update(0.016, 0);
    CHECK(t.pad().axis(PadAxis::LX) == 128); // origin = centre
    t.finger_move(1, 120 + range * 0.5f, 250, 1.0f);
    t.update(0.016, 0);
    CHECK(std::fabs(t.pad().axis_f(PadAxis::LX) - 0.5f) < 0.02f);
    CHECK(t.pad().down(PadButton::Cross)); // other finger still on the gas
    t.finger_move(1, 120 - range * 3, 250, 1.0f); // past full lock
    t.update(0.016, 0);
    CHECK(t.pad().axis(PadAxis::LX) == 0);
    // The steering finger never turns into a button press.
    t.finger_move(1, gas.x + 5, gas.y + 5, 1.0f);
    t.update(0.016, 0);
    CHECK(t.pad().axis(PadAxis::LX) == 255);
    t.finger_up(1);
    t.update(0.016, 0);
    CHECK(t.pad().axis(PadAxis::LX) == 128);
}

TEST(touch_tilt_steering) {
    TouchConfig cfg;
    cfg.steering = TouchConfig::Steering::Tilt;
    cfg.mode = TouchConfig::Mode::On;
    TouchControls t = driving(cfg);
    t.update(0.016, 0.02f); // inside the deadzone
    CHECK(t.pad().axis(PadAxis::LX) == 128);
    t.update(0.016, -1.0f);
    CHECK(t.pad().axis(PadAxis::LX) == 0);
    t.update(0.016, 0.52f);
    CHECK(std::fabs(t.pad().axis_f(PadAxis::LX) - 0.5f) < 0.02f);
    // A finger in the (unused) steering zone does nothing in tilt mode.
    t.finger_down(1, 100, 300, 1.0f);
    t.update(0.016, 0.0f);
    CHECK(t.pad().axis(PadAxis::LX) == 128);
}

TEST(touch_layout_switch_releases) {
    TouchControls t = driving();
    const Widget* start = find_widget(t, Widget::Kind::Button, PadButton::Start);
    CHECK(start != nullptr);
    t.finger_down(3, start->x, start->y, 1.0f);
    t.update(0.016, 0);
    CHECK(t.pad().down(PadButton::Start));
    t.set_layout(TouchLayout::Menu); // pause menu opened
    t.update(0.016, 0);
    CHECK(!t.pad().down(PadButton::Start));
    t.finger_up(3); // the late "up" is harmless
}

TEST(touch_menu_dpad_and_face_buttons) {
    TouchControls t;
    t.set_layout(TouchLayout::Menu);
    const Widget pad = *find_widget(t, Widget::Kind::DPad);
    const float cx = pad.x + pad.w / 2, cy = pad.y + pad.h / 2;
    CHECK(t.finger_down(1, cx, cy - pad.h * 0.4f, 1.0f));
    t.update(0.016, 0);
    CHECK(t.pad().down(PadButton::Up));
    t.finger_move(1, cx + pad.w * 0.4f, cy + 2, 1.0f);
    t.update(0.016, 0);
    CHECK(t.pad().down(PadButton::Right));
    CHECK(!t.pad().down(PadButton::Up));
    t.finger_move(1, cx, cy, 1.0f); // centre: dead zone
    t.update(0.016, 0);
    CHECK(t.pad().buttons == 0);
    t.finger_up(1);

    const Widget cross = *find_widget(t, Widget::Kind::Button, PadButton::Cross);
    CHECK(t.hit(cross.x, cross.y));
    CHECK(t.finger_down(2, cross.x, cross.y, 1.0f));
    t.update(0.016, 0);
    CHECK(t.pad().down(PadButton::Cross));
    CHECK(t.pad().press(PadButton::Cross) == 255);
    // Taps away from the controls go to the screen underneath.
    CHECK(!t.hit(320, 180));
    CHECK(!t.finger_down(3, 320, 180, 1.0f));
}

TEST(touch_left_handed_mirrors) {
    TouchConfig cfg;
    cfg.left_handed = true;
    TouchControls t = driving(cfg);
    const Widget* gas = find_widget(t, Widget::Kind::Pedal, PadButton::Cross);
    const Widget* steer = find_widget(t, Widget::Kind::Steering);
    CHECK(gas->x < 200);
    CHECK(steer->x >= 319);
}

TEST(touch_config_roundtrip) {
    IniDocument ini = IniDocument::parse(
        "[touch]\nmode = on\nsteering = tilt\npedals = area\npedal_min = 0.2\nsteer_range = 9\nopacity = 0.8\n"
        "size = 1.2\nleft_handed = on\n");
    TouchConfig c = TouchConfig::load(ini);
    CHECK(c.mode == TouchConfig::Mode::On);
    CHECK(c.steering == TouchConfig::Steering::Tilt);
    CHECK(c.pedals == TouchConfig::Pedals::Area);
    CHECK(std::fabs(c.pedal_min - 0.2f) < 1e-6f);
    CHECK(c.steer_range == 0.5f); // clamped
    CHECK(c.left_handed);
    IniDocument out;
    c.save(out);
    TouchConfig d = TouchConfig::load(out);
    CHECK(d.mode == c.mode && d.steering == c.steering && d.pedals == c.pedals && d.left_handed);
    CHECK(std::fabs(d.size - 1.2f) < 1e-3f && std::fabs(d.opacity - 0.8f) < 1e-3f);
    TouchConfig def = TouchConfig::load(IniDocument{});
    CHECK(def.mode == TouchConfig::Mode::Auto && def.steering == TouchConfig::Steering::Drag);
}

TEST(touch_overlay_merges_into_player_pad) {
    InputManager input;
    PadState ov;
    ov.connected = true;
    ov.buttons = u16(1u << unsigned(PadButton::Cross));
    ov.pressure[size_t(PadButton::Cross)] = 100;
    ov.axes[size_t(PadAxis::LX)] = 0;
    input.set_overlay(0, ov);
    input.update(0.016);
    const PadState& p = input.pad(0);
    CHECK(p.down(PadButton::Cross));
    CHECK(p.press(PadButton::Cross) == 100);
    CHECK(p.axis(PadAxis::LX) == 0);
    CHECK(!input.pad(1).down(PadButton::Cross));
    input.set_overlay(0, PadState{}); // disconnected overlay: ignored
    input.update(0.016);
    CHECK(!input.pad(0).down(PadButton::Cross));
    CHECK(input.pad(0).axis(PadAxis::LX) == 128);
}

// Race controls per player: the layout and custom race bindings survive a save / load, and
// the overlay (GT3 meaning) reaches the race pad too.
TEST(input_race_layout_per_player) {
    IniDocument cfg = IniDocument::parse("[race]\nbuttons = ours\n[player2]\npreset = classic\n");
    InputManager input;
    input.load(cfg);
    CHECK(input.race_layout(0) == "modern" && input.race_layout(1) == "modern"); // [race] buttons = ours
    input.set_race_layout(1, "gt3");
    CHECK(input.race_uses_menu_pad(1) && !input.race_uses_menu_pad(0));
    CHECK(&input.race_profile(1) == &input.profile(1));
    PadTarget view;
    view.kind = PadTarget::Kind::Button;
    view.button = PadButton::R1;
    InputSource y;
    CHECK(InputSource::parse("pad.north", y));
    InputSource rb;
    CHECK(InputSource::parse("pad.rightshoulder", rb));
    input.bind_race(0, view, rb, true); // player 1 only
    CHECK(input.is_race_custom(0) && !input.is_race_custom(1));
    IniDocument out;
    input.save(out);
    CHECK(out.get("player1", "race") == "modern" && out.get("player2", "race") == "gt3");
    CHECK(out.get("player1.race", "R1") == "pad.rightshoulder");
    CHECK(!out.find("player2.race"));
    InputManager again;
    again.load(out);
    CHECK(again.is_race_custom(0) && again.race_layout(1) == "gt3" && again.race_uses_menu_pad(1));
    bool north_view = false;
    for (const Binding& b : again.race_profile(0).bindings) north_view |= b.source == y && b.target == view;
    CHECK(!north_view); // replaced
    // Old configs: [race] buttons = auto picks GT3's layout for the classic preset.
    InputManager old;
    old.load(IniDocument::parse("[player1]\npreset = classic\n"));
    CHECK(old.race_layout(0) == "gt3" && old.race_layout(1) == "modern");

    PadState ov;
    ov.connected = true;
    ov.buttons = u16(1u << unsigned(PadButton::Cross));
    ov.pressure[size_t(PadButton::Cross)] = 90;
    input.set_overlay(0, ov);
    input.update(0.016);
    CHECK(input.race_pad(0).press(PadButton::Cross) == 90);
}

TEST(touch_tap_shorter_than_a_frame) {
    TouchControls t;
    t.set_layout(TouchLayout::Menu);
    const Widget cross = *find_widget(t, Widget::Kind::Button, PadButton::Cross);
    t.finger_down(1, cross.x, cross.y, 1.0f);
    t.finger_up(1); // down and up between two frames
    t.update(0.016, 0);
    CHECK(t.pad().down(PadButton::Cross)); // reported once...
    t.update(0.016, 0);
    CHECK(!t.pad().down(PadButton::Cross)); // ...then released
}

TEST(touch_controls_use_side_bars) {
    // 20:9 phone: the 16:9 game area leaves ~80 UI units on each side.
    TouchControls t = driving();
    t.set_screen(-80, 0, 720, 360);
    const Widget gas = *find_widget(t, Widget::Kind::Pedal, PadButton::Cross);
    CHECK(gas.x + gas.w > 690); // anchored to the right edge of the screen
    const Widget steer = *find_widget(t, Widget::Kind::Steering);
    CHECK(steer.x == -80.0f);
    CHECK(t.finger_down(1, -60, 300, 1.0f)); // a touch in the left bar steers
    t.finger_move(1, -60 + 0.14f * 640 * 0.5f, 300, 1.0f);
    t.update(0.016, 0);
    CHECK(std::fabs(t.pad().axis_f(PadAxis::LX) - 0.5f) < 0.02f);
    // A smaller "screen" than the game area is ignored.
    t.set_screen(10, 10, 600, 300);
    CHECK(find_widget(t, Widget::Kind::Pedal, PadButton::Cross)->x + gas.w <= 640.0f);
}

TEST(touch_controls_avoid_cutout_insets) {
    // Phone with a display cutout on the left (27 UI units) and small
    // rounded-corner insets elsewhere.
    TouchControls t;
    t.set_layout(TouchLayout::Menu);
    t.set_screen(-70, 0, 710, 360);
    const float pad_x = find_widget(t, Widget::Kind::DPad)->x;
    t.set_insets(27, 11, 11, 11);
    CHECK(find_widget(t, Widget::Kind::DPad)->x >= -70 + 27 + 4 - 0.01f); // clear of the cutout
    CHECK(find_widget(t, Widget::Kind::DPad)->x > pad_x);
    CHECK(find_widget(t, Widget::Kind::Button, PadButton::L1)->x >= -70 + 31 - 0.01f);
    // Left-handed: the pedals move to the cutout side and keep clear of it.
    TouchConfig lh;
    lh.left_handed = true;
    TouchControls d = driving(lh);
    d.set_screen(-70, 0, 710, 360);
    d.set_insets(27, 11, 11, 11);
    const Widget gas = *find_widget(d, Widget::Kind::Pedal, PadButton::Cross);
    CHECK(gas.x >= -70 + 31 - 0.01f && gas.x < 0);
    // Right-handed: the pedals keep their normal margin from the right edge.
    TouchControls r = driving();
    r.set_screen(-70, 0, 710, 360);
    const float gx = find_widget(r, Widget::Kind::Pedal, PadButton::Cross)->x;
    r.set_insets(27, 11, 11, 11);
    CHECK(std::fabs(find_widget(r, Widget::Kind::Pedal, PadButton::Cross)->x - (gx - 1)) < 0.01f); // 11 + 4 vs 14
}

TEST(touch_safe_area) {
    TouchControls t = driving();
    TouchRect hidden = t.safe_area();
    CHECK(hidden.x == 0 && hidden.y == 0 && hidden.w == 640 && hidden.h == 360);
    t.note_touch();
    TouchRect a = t.safe_area();
    const Widget brake = *find_widget(t, Widget::Kind::Pedal, PadButton::Square);
    CHECK(a.x == 0);                      // the steering zone doesn't count
    CHECK(a.x + a.w < brake.x);           // right edge clear of the pedal cluster (incl. handbrake)
    CHECK(a.y > 30 && a.y < 60);          // below the pause/camera row
    CHECK(a.y + a.h == 360);
    // With wide side bars most of the cluster leaves the game area.
    t.set_screen(-80, 0, 720, 360);
    TouchRect wide = t.safe_area();
    CHECK(wide.x + wide.w > a.x + a.w + 60);
    // Menu layout: D-pad on the left, face buttons on the right.
    t.set_layout(TouchLayout::Menu);
    t.set_screen(0, 0, 640, 360);
    TouchRect m = t.safe_area();
    CHECK(m.x > 100 && m.x + m.w < 540);
}

TEST(touch_gt3_race_buttons) {
    // The driving layout presses the race pad's buttons in GT3's meaning.
    TouchControls t = driving();
    CHECK(find_widget(t, Widget::Kind::Button, PadButton::R2)->label == "UP");
    CHECK(find_widget(t, Widget::Kind::Button, PadButton::L2)->label == "DOWN");
    CHECK(find_widget(t, Widget::Kind::Button, PadButton::R1)->label == "CAM");
    CHECK(find_widget(t, Widget::Kind::Button, PadButton::Select)->label == "RST");
    CHECK(find_widget(t, Widget::Kind::Button, PadButton::Triangle) == nullptr);
}
