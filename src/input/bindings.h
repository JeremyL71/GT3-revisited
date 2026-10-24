#pragma once

#include "input/virtual_pad.h"

#include <string>
#include <vector>

namespace gt3r {

// What a binding reads from. Values are normalized: digital 0/1, full
// axes -1..1, half axes / triggers 0..1.
struct InputSource {
    enum class Kind : u8 {
        None,
        Key,           // code = SDL_Scancode
        PadButton,     // code = SDL_GamepadButton
        PadAxis,       // code = SDL_GamepadAxis
        JoyButton,     // code = button index (wheels, pedals, arcade sticks)
        JoyAxis,       // code = axis index
        JoyHat,        // code = hat index, dir = hat direction mask
        Tilt,          // gamepad accelerometer roll (or device tilt on mobile)
        Mouse,         // code = SDL mouse button
    };
    enum class Half : u8 { Full, Positive, Negative };

    Kind kind = Kind::None;
    int code = 0;
    u8 hat_dir = 0;
    Half half = Half::Full;

    bool is_analog() const { return kind == Kind::PadAxis || kind == Kind::JoyAxis || kind == Kind::Tilt; }
    std::string to_string() const;
    static bool parse(std::string_view token, InputSource& out);
    bool operator==(const InputSource&) const = default;
};

// Shapes a raw source value before it reaches the pad.
struct BindingOptions {
    float deadzone = 0.0f;   // inner deadzone (0..1)
    float saturation = 1.0f; // value at which output reaches full
    float curve = 1.0f;      // response exponent: >1 finer control near rest
    float scale = 1.0f;
    bool invert = false;
    bool pedal = false;      // map a full -1..1 axis to 0..1 (wheel pedals)
    float ramp = 0.0f;       // seconds to reach full value (keyboard "analog")
    float threshold = 0.0f;  // digital press threshold for analog -> button (0 = auto)
};

// Where a binding writes to.
struct PadTarget {
    enum class Kind : u8 { None, Button, Axis };
    Kind kind = Kind::None;
    PadButton button = PadButton::Cross;
    PadAxis axis = PadAxis::LX;
    InputSource::Half half = InputSource::Half::Full; // for axes: LX+ / LX-

    std::string to_string() const;
    static bool parse(std::string_view token, PadTarget& out);
    bool operator==(const PadTarget&) const = default;
};

struct Binding {
    PadTarget target;
    InputSource source;
    BindingOptions options;

    // "Cross = pad.righttrigger curve=1.5" -> one Binding per '|' alternative.
    static std::vector<Binding> parse_line(std::string_view target, std::string_view sources);
    std::string source_with_options() const;
};

struct Profile {
    std::string name;
    std::vector<Binding> bindings;

    void add(std::string_view target, std::string_view sources);
    void remove_target(const PadTarget& target);
    // No binding reads a controller (pad, joystick or tilt): only keys and mouse buttons drive it.
    bool keyboard_only() const;
};

// Built-in presets for the virtual DualShock 2 (menus, and races in GT3's layout).
// "racing" and "classic" are the DualShock 2 layout 1:1 (triggers -> L2/R2; "racing" also
// binds the back paddles); "wheel" is a typical USB wheel; "keyboard" is keyboard only. All
// presets also include keyboard bindings so a pad is never required.
// "quest" (VR builds' default) is "racing" with B as back (Triangle) and Y as Circle.
Profile make_preset(std::string_view name);
const std::vector<std::string_view>& preset_names();
std::string_view default_preset(); // "racing"; "quest" in VR builds

// Race controls. While driving, the race reads a second virtual pad per player whose buttons
// always mean what they meant in GT3 (Cross accelerate, Square brake, Triangle reverse, R2/L2
// shift, R1 view, L1 look back, Select respawn, ...); see race_action_name(). A race layout
// says which physical inputs drive each of those actions:
//   "modern" (default)  a current racing-game layout: triggers throttle/brake, Cross/Square
//                       shift, Circle handbrake, Triangle view, R3 look back, L1 reverse,
//                       Select respawn; WASD-style keyboard driving
//   "gt3"               the original: the race reads the menu pad itself (identity), so every
//                       button does what it did on a DualShock 2
// make_race_preset("gt3", ...) returns an empty profile named "gt3" (= use the menu pad).
Profile make_race_preset(std::string_view layout, std::string_view device_preset);
const std::vector<std::string_view>& race_layout_names();
// The race action a race-pad button / axis stands for ("Accelerate" for Cross); empty = unused.
std::string_view race_action_name(const PadTarget& target);

} // namespace gt3r
