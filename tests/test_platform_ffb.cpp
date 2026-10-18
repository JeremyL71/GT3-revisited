// Controller feedback shaping (input/force_feedback.h). Device output itself
// needs hardware; these check the effect values the devices are sent.

#include "test_framework.h"

#include "base/config.h"
#include "input/force_feedback.h"
#include "input/input_manager.h"

#include <cmath>

using namespace gt3r;

TEST(ffb_dualsense_brake_trigger) {
    ForceFeedbackConfig c;
    ForceFeedbackState s;
    s.speed_kmh = 100;
    ffb::TriggerEffect e = ffb::brake_trigger_effect(s, c);
    CHECK(e[0] == 0x01);                 // continuous resistance
    CHECK(e[2] == u8(std::lround(0.6f * 255))); // trigger_force at speed
    s.speed_kmh = 0;
    CHECK(ffb::brake_trigger_effect(s, c)[2] < e[2]); // lighter when stopped
    s.abs_active = true;
    ffb::TriggerEffect abs = ffb::brake_trigger_effect(s, c);
    CHECK(abs[0] == 0x06 && abs[1] > 0 && abs[2] > 0); // vibration while ABS works
    c.triggers = false;
    CHECK(ffb::brake_trigger_effect(s, c) == ffb::trigger_off());
    c.triggers = true;
    c.enabled = false;
    CHECK(ffb::brake_trigger_effect(s, c)[0] == 0x05);
}

TEST(ffb_xbox_impulse_pulses_only_with_abs) {
    ForceFeedbackState s;
    CHECK(ffb::impulse_brake(s, 0.01) == 0.0f);
    s.abs_active = true;
    int on = 0;
    for (int i = 0; i < 100; ++i) on += ffb::impulse_brake(s, i / 100.0) > 0 ? 1 : 0;
    CHECK(on > 40 && on < 60); // ~50 % duty
}

TEST(ffb_wheel_spring_and_force) {
    ForceFeedbackConfig c;
    ForceFeedbackState s;
    int stopped = ffb::wheel_spring_coefficient(s, c);
    s.speed_kmh = 200;
    int fast = ffb::wheel_spring_coefficient(s, c);
    CHECK(stopped > 0 && fast > stopped * 4);
    CHECK(fast <= 32767);
    s.steer_force = 0.5f;
    CHECK(std::abs(ffb::wheel_constant_level(s, c) - 16384) <= 1);
    c.invert = true;
    CHECK(ffb::wheel_constant_level(s, c) < 0);
    c.invert = false;
    s.steer_force = 3.0f;
    CHECK(ffb::wheel_constant_level(s, c) == 32767);
    c.wheel = false;
    CHECK(ffb::wheel_constant_level(s, c) == 0 && ffb::wheel_spring_coefficient(s, c) == 0);
}

TEST(ffb_config_roundtrip) {
    IniDocument ini = IniDocument::parse("[ffb]\nenabled = off\nwheel = off\nstrength = 5\nspring = 0.3\ninvert = on\n");
    ForceFeedbackConfig c = ForceFeedbackConfig::load(ini);
    CHECK(!c.enabled && !c.wheel && c.triggers && c.invert);
    CHECK(c.strength == 2.0f); // clamped
    IniDocument out;
    c.save(out);
    ForceFeedbackConfig d = ForceFeedbackConfig::load(out);
    CHECK(!d.enabled && !d.wheel && d.invert && std::fabs(d.spring - 0.3f) < 1e-3f);
}

TEST(ffb_input_manager_without_devices) {
    InputManager input; // no devices: must be a harmless no-op
    ForceFeedbackState s;
    s.abs_active = true;
    input.set_force_feedback(0, s);
    input.update(0.016);
    input.update(0.5); // times out
    CHECK(input.ffb_config().enabled);
}
