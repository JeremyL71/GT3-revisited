#include "input/input_manager.h"

#include "base/config.h"
#include "base/log.h"
#include "platform/device_tilt.h"

#include <SDL3/SDL_joystick.h>
#include <SDL3/SDL_keyboard.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_sensor.h>

#include <algorithm>
#include <cmath>

namespace gt3r {

namespace {

constexpr float kAxisMax = 32767.0f;
constexpr float kCaptureAxisDelta = 0.6f;
constexpr float kTiltRange = 0.785398f; // 45 degrees of roll = full lock

float normalize_axis(Sint16 v) { return std::clamp(static_cast<float>(v) / kAxisMax, -1.0f, 1.0f); }

std::string section_name(int player, const char* suffix = "") {
    return "player" + std::to_string(player + 1) + suffix;
}

bool contains_ci(const std::string& hay, const std::string& needle) {
    return to_lower(hay).find(to_lower(needle)) != std::string::npos;
}

Profile load_profile(const IniDocument& cfg, const std::string& section) {
    Profile custom;
    custom.name = "custom";
    if (const IniDocument::Section* b = cfg.find(section))
        for (const auto& [target, sources] : b->entries) custom.add(target, sources);
    return custom;
}

void save_profile(IniDocument& cfg, const std::string& section, const Profile* prof) {
    auto& sections = cfg.sections();
    std::erase_if(sections, [&](const IniDocument::Section& s) { return s.name == section; });
    if (!prof) return;
    // Group alternatives for the same target onto one line.
    IniDocument::Section& out = cfg.get_or_add(section);
    for (const Binding& b : prof->bindings) {
        std::string key = b.target.to_string();
        auto it = std::find_if(out.entries.begin(), out.entries.end(), [&](auto& kv) { return kv.first == key; });
        if (it == out.entries.end())
            out.entries.emplace_back(key, b.source_with_options());
        else
            it->second += " | " + b.source_with_options();
    }
}

void bind_into(Profile& prof, const PadTarget& target, const InputSource& source, bool replace) {
    // Clear both the full axis and its halves when replacing an axis binding.
    if (replace) {
        prof.remove_target(target);
        if (target.kind == PadTarget::Kind::Axis && target.half == InputSource::Half::Full) {
            PadTarget t = target;
            t.half = InputSource::Half::Positive;
            prof.remove_target(t);
            t.half = InputSource::Half::Negative;
            prof.remove_target(t);
        }
    }
    Binding b;
    b.target = target;
    b.source = source;
    // A full stick axis bound to a full pad axis drops the captured half.
    if (target.kind == PadTarget::Kind::Axis && target.half == InputSource::Half::Full && source.is_analog())
        b.source.half = InputSource::Half::Full;
    if (source.kind == InputSource::Kind::PadAxis || source.kind == InputSource::Kind::JoyAxis)
        if (target.kind == PadTarget::Kind::Axis) b.options.deadzone = 0.08f;
    prof.bindings.push_back(b);
    prof.name = "custom";
}

// [playerN] race; before it existed, [race] buttons = gt3 | ours | auto (auto: GT3's layout
// with the classic preset) chose the race buttons for everyone.
std::string initial_race_layout(const IniDocument& cfg, const std::string& sec, const std::string& preset) {
    if (auto r = cfg.get(sec, "race")) return to_lower(*r) == "gt3" ? "gt3" : "modern";
    std::string b = to_lower(cfg.get("race", "buttons").value_or("auto"));
    return b == "gt3" || (b == "auto" && preset == "classic") ? "gt3" : "modern";
}

} // namespace

InputManager::InputManager() {
    for (int i = 0; i < kMaxPlayers; ++i) {
        players_[i].profile = make_preset(default_preset());
        players_[i].preset = std::string(default_preset());
        players_[i].race_profile = make_race_preset("modern", default_preset());
        players_[i].keyboard = (i == 0);
    }
    int count = 0;
    if (SDL_JoystickID* ids = SDL_GetJoysticks(&count)) {
        for (int i = 0; i < count; ++i) add_device(ids[i]);
        SDL_free(ids);
    }
    // Mobile devices: use the device's own accelerometer for tilt steering.
    if (SDL_SensorID* sensors = SDL_GetSensors(&count)) {
        for (int i = 0; i < count && !device_accel_; ++i)
            if (SDL_GetSensorTypeForID(sensors[i]) == SDL_SENSOR_ACCEL) device_accel_ = SDL_OpenSensor(sensors[i]);
        SDL_free(sensors);
    }
    assign_devices();
}

InputManager::~InputManager() {
    for (Device& d : devices_) {
        ffb_.stop(d.id);
        ffb_.forget(d.id); // haptics before their joystick
        if (d.gamepad) SDL_CloseGamepad(d.gamepad);
        else if (d.joystick) SDL_CloseJoystick(d.joystick);
    }
    if (device_accel_) SDL_CloseSensor(device_accel_);
}

void InputManager::load(const IniDocument& cfg) {
    ffb_.configure(ForceFeedbackConfig::load(cfg));
    for (int i = 0; i < kMaxPlayers; ++i) {
        Player& p = players_[i];
        std::string sec = section_name(i);
        p.preset = cfg.get(sec, "preset").value_or(std::string(default_preset()));
        p.device_filter = cfg.get(sec, "device").value_or("auto");
        if (auto kb = cfg.get(sec, "keyboard")) p.keyboard = to_lower(*kb) == "on" || *kb == "1";
        p.profile = make_preset(p.preset);
        p.custom_bindings = false;
        if (Profile custom = load_profile(cfg, section_name(i, ".bindings")); !custom.bindings.empty()) {
            p.profile = std::move(custom);
            p.custom_bindings = true;
        }
        p.ramp_values.assign(p.profile.bindings.size(), 0.0f);
        p.race_layout = initial_race_layout(cfg, sec, p.preset);
        p.race_profile = make_race_preset(p.race_layout, p.preset);
        p.race_custom = false;
        if (Profile custom = load_profile(cfg, section_name(i, ".race")); !custom.bindings.empty()) {
            p.race_profile = std::move(custom);
            p.race_custom = true;
        }
        p.race_ramps.assign(p.race_profile.bindings.size(), 0.0f);
    }
    assign_devices();
}

void InputManager::save(IniDocument& cfg) const {
    ffb_.config().save(cfg);
    for (int i = 0; i < kMaxPlayers; ++i) {
        const Player& p = players_[i];
        std::string sec = section_name(i);
        cfg.set(sec, "preset", p.preset);
        cfg.set(sec, "device", p.device_filter);
        cfg.set(sec, "keyboard", p.keyboard ? "on" : "off");
        cfg.set(sec, "race", p.race_layout);
        save_profile(cfg, section_name(i, ".bindings"), p.custom_bindings ? &p.profile : nullptr);
        save_profile(cfg, section_name(i, ".race"), p.race_custom ? &p.race_profile : nullptr);
    }
}

void InputManager::apply_preset(int player, std::string_view preset) {
    Player& p = players_[player];
    p.preset = std::string(preset);
    p.profile = make_preset(preset);
    p.custom_bindings = false;
    p.ramp_values.assign(p.profile.bindings.size(), 0.0f);
    if (!p.race_custom) set_race_layout(player, p.race_layout); // the race layout of this device kind
    assign_devices();
}

void InputManager::bind(int player, const PadTarget& target, const InputSource& source, bool replace) {
    Player& p = players_[player];
    bind_into(p.profile, target, source, replace);
    p.custom_bindings = true;
    p.ramp_values.assign(p.profile.bindings.size(), 0.0f);
}

void InputManager::clear_target(int player, const PadTarget& target) {
    Player& p = players_[player];
    p.profile.remove_target(target);
    p.profile.name = "custom";
    p.custom_bindings = true;
    p.ramp_values.assign(p.profile.bindings.size(), 0.0f);
}

void InputManager::set_race_layout(int player, std::string_view layout) {
    Player& p = players_[player];
    p.race_profile = make_race_preset(layout, p.preset);
    p.race_layout = p.race_profile.name;
    p.race_custom = false;
    p.race_ramps.assign(p.race_profile.bindings.size(), 0.0f);
}

bool InputManager::race_uses_menu_pad(int player) const {
    const Player& p = players_[player];
    return !p.race_custom && p.race_layout == "gt3";
}

const Profile& InputManager::race_profile(int player) const {
    return race_uses_menu_pad(player) ? players_[player].profile : players_[player].race_profile;
}

void InputManager::bind_race(int player, const PadTarget& target, const InputSource& source, bool replace) {
    Player& p = players_[player];
    if (race_uses_menu_pad(player)) p.race_profile = p.profile; // the original layout, as it stands
    bind_into(p.race_profile, target, source, replace);
    p.race_custom = true;
    p.race_ramps.assign(p.race_profile.bindings.size(), 0.0f);
}

void InputManager::clear_race_target(int player, const PadTarget& target) {
    Player& p = players_[player];
    if (race_uses_menu_pad(player)) p.race_profile = p.profile;
    p.race_profile.remove_target(target);
    p.race_profile.name = "custom";
    p.race_custom = true;
    p.race_ramps.assign(p.race_profile.bindings.size(), 0.0f);
}

void InputManager::add_device(SDL_JoystickID id) {
    if (find_device(id)) return;
#if GT3R_XR
    // A Meta headset's own controllers and buttons, as Android gamepads: the Touch controllers
    // come through OpenXR instead (xr::Session's virtual gamepad).
    if (SDL_GetJoystickVendorForID(id) == 0x2833) {
        LOG_INFO("Input device ignored: %s (headset)", SDL_GetJoystickNameForID(id) ? SDL_GetJoystickNameForID(id) : "?");
        return;
    }
#endif
    Device d;
    d.id = id;
    if (SDL_IsGamepad(id)) {
        d.gamepad = SDL_OpenGamepad(id);
        d.joystick = d.gamepad ? SDL_GetGamepadJoystick(d.gamepad) : nullptr;
        if (d.gamepad && SDL_GamepadHasSensor(d.gamepad, SDL_SENSOR_ACCEL))
            d.has_accel = SDL_SetGamepadSensorEnabled(d.gamepad, SDL_SENSOR_ACCEL, true);
    } else {
        d.joystick = SDL_OpenJoystick(id);
    }
    if (!d.joystick) {
        LOG_WARN("Failed to open input device %u: %s", unsigned(id), SDL_GetError());
        return;
    }
    const char* name = d.gamepad ? SDL_GetGamepadName(d.gamepad) : SDL_GetJoystickName(d.joystick);
    d.name = name ? name : "Unknown device";
    LOG_INFO("Input device connected: %s (%s)", d.name.c_str(), d.gamepad ? "gamepad" : "joystick");
    devices_.push_back(std::move(d));
}

void InputManager::remove_device(SDL_JoystickID id) {
    auto it = std::find_if(devices_.begin(), devices_.end(), [&](const Device& d) { return d.id == id; });
    if (it == devices_.end()) return;
    LOG_INFO("Input device disconnected: %s", it->name.c_str());
    ffb_.forget(id); // closes its haptic before the joystick
    if (it->gamepad) SDL_CloseGamepad(it->gamepad);
    else SDL_CloseJoystick(it->joystick);
    devices_.erase(it);
}

const InputManager::Device* InputManager::find_device(SDL_JoystickID id) const {
    for (const Device& d : devices_)
        if (d.id == id) return &d;
    return nullptr;
}

void InputManager::assign_devices() {
    std::vector<SDL_JoystickID> taken;
    auto is_taken = [&](SDL_JoystickID id) { return std::find(taken.begin(), taken.end(), id) != taken.end(); };
    // Named filters first, so "auto" players don't steal a named device.
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = 0; i < kMaxPlayers; ++i) {
            Player& p = players_[i];
            bool is_auto = p.device_filter == "auto" || p.device_filter.empty();
            if ((pass == 0) == is_auto) continue;
            p.device = 0;
            bool wants_wheel = p.preset == "wheel";
            for (const Device& d : devices_) {
                if (is_taken(d.id)) continue;
                bool match = is_auto ? (wants_wheel ? !d.gamepad : d.gamepad != nullptr)
                                     : contains_ci(d.name, p.device_filter);
                if (match) {
                    p.device = d.id;
                    taken.push_back(d.id);
                    break;
                }
            }
            if (const Device* d = find_device(p.device); d && d->gamepad) SDL_SetGamepadPlayerIndex(d->gamepad, i);
        }
    }
}

void InputManager::set_device_filter(int player, std::string filter) {
    players_[player].device_filter = filter.empty() ? "auto" : std::move(filter);
    assign_devices();
}

std::vector<std::string> InputManager::device_names() const {
    std::vector<std::string> out;
    for (const Device& d : devices_) out.push_back(d.name);
    return out;
}

std::string InputManager::device_name(int player) const {
    const Device* d = find_device(players_[player].device);
    if (d) return d->name;
    return players_[player].keyboard ? "Keyboard" : "No device";
}

std::string InputManager::describe(int player, const InputSource& src) const {
    const Device* d = find_device(players_[player].device);
    if (src.kind == InputSource::Kind::PadButton && d && d->gamepad) {
        switch (SDL_GetGamepadButtonLabel(d->gamepad, static_cast<SDL_GamepadButton>(src.code))) {
        case SDL_GAMEPAD_BUTTON_LABEL_A: return "A";
        case SDL_GAMEPAD_BUTTON_LABEL_B: return "B";
        case SDL_GAMEPAD_BUTTON_LABEL_X: return "X";
        case SDL_GAMEPAD_BUTTON_LABEL_Y: return "Y";
        case SDL_GAMEPAD_BUTTON_LABEL_CROSS: return "Cross";
        case SDL_GAMEPAD_BUTTON_LABEL_CIRCLE: return "Circle";
        case SDL_GAMEPAD_BUTTON_LABEL_SQUARE: return "Square";
        case SDL_GAMEPAD_BUTTON_LABEL_TRIANGLE: return "Triangle";
        default: break;
        }
    }
    if (src.kind == InputSource::Kind::Key) return std::string("Key ") + SDL_GetScancodeName(static_cast<SDL_Scancode>(src.code));
    return src.to_string();
}

void InputManager::handle_event(const SDL_Event& e) {
    switch (e.type) {
    case SDL_EVENT_JOYSTICK_ADDED:
        add_device(e.jdevice.which);
        assign_devices();
        break;
    case SDL_EVENT_JOYSTICK_REMOVED:
        remove_device(e.jdevice.which);
        assign_devices();
        break;
    case SDL_EVENT_KEY_DOWN:
        if (size_t(e.key.scancode) < size_t(SDL_SCANCODE_COUNT)) {
            if (key_latch_.empty()) key_latch_.assign(SDL_SCANCODE_COUNT, false);
            key_latch_[size_t(e.key.scancode)] = true;
        }
        if (capturing_ && !e.key.repeat) {
            captured_ = InputSource{InputSource::Kind::Key, int(e.key.scancode)};
            capturing_ = false;
        }
        break;
    case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
        if (capturing_) {
            captured_ = InputSource{InputSource::Kind::PadButton, int(e.gbutton.button)};
            capturing_ = false;
        }
        break;
    case SDL_EVENT_JOYSTICK_BUTTON_DOWN:
        // Gamepads report through the gamepad event above.
        if (capturing_ && !SDL_IsGamepad(e.jbutton.which)) {
            captured_ = InputSource{InputSource::Kind::JoyButton, int(e.jbutton.button)};
            capturing_ = false;
        }
        break;
    case SDL_EVENT_JOYSTICK_HAT_MOTION:
        if (capturing_ && !SDL_IsGamepad(e.jhat.which) && e.jhat.value != SDL_HAT_CENTERED) {
            u8 v = e.jhat.value;
            u8 dir = (v & SDL_HAT_UP) ? SDL_HAT_UP : (v & SDL_HAT_RIGHT) ? SDL_HAT_RIGHT
                   : (v & SDL_HAT_DOWN) ? SDL_HAT_DOWN : SDL_HAT_LEFT;
            captured_ = InputSource{InputSource::Kind::JoyHat, int(e.jhat.hat), dir};
            capturing_ = false;
        }
        break;
    default: break;
    }
}

void InputManager::begin_capture() {
    capturing_ = true;
    captured_.reset();
    // Record resting positions so pedals that rest at +/-1 and slightly
    // off-center sticks don't register as input.
    capture_baseline_.clear();
    for (const Device& d : devices_) {
        std::vector<float> axes;
        if (d.gamepad)
            for (int a = 0; a < SDL_GAMEPAD_AXIS_COUNT; ++a)
                axes.push_back(normalize_axis(SDL_GetGamepadAxis(d.gamepad, SDL_GamepadAxis(a))));
        else
            for (int a = 0; a < SDL_GetNumJoystickAxes(d.joystick); ++a)
                axes.push_back(normalize_axis(SDL_GetJoystickAxis(d.joystick, a)));
        capture_baseline_.emplace_back(d.id, std::move(axes));
    }
}

std::optional<InputSource> InputManager::poll_capture() {
    auto out = captured_;
    captured_.reset();
    return out;
}

void InputManager::update_tilt(Player& p, const Device* dev) {
    float accel[3] = {0, 0, 0};
    bool ok = false;
    if (dev && dev->has_accel) {
        ok = SDL_GetGamepadSensorData(dev->gamepad, SDL_SENSOR_ACCEL, accel, 3);
    } else if (device_accel_ && SDL_GetSensorData(device_accel_, accel, 3)) {
        // Phone/tablet: the sensor doesn't rotate with the display.
        float raw[3] = {accel[0], accel[1], accel[2]};
        platform::accel_to_display(raw, accel, platform::display_rotation_degrees());
        ok = true;
    }
    if (!ok) return;
    // Roll around the axis pointing at the player (clockwise = right).
    float roll = platform::steering_roll(accel);
    p.tilt = std::clamp(roll / kTiltRange, -1.0f, 1.0f);
}

float InputManager::read_source(const Player& p, const Device* dev, const InputSource& src) const {
    float v = 0.0f;
    switch (src.kind) {
    case InputSource::Kind::None: return 0.0f;
    case InputSource::Kind::Key: {
        if (!p.keyboard) return 0.0f;
        int n = 0;
        const bool* keys = SDL_GetKeyboardState(&n);
        bool latched = size_t(src.code) < key_latch_.size() && key_latch_[size_t(src.code)];
        return ((src.code < n && keys[src.code]) || latched) ? 1.0f : 0.0f;
    }
    case InputSource::Kind::Mouse:
        if (!p.keyboard) return 0.0f;
        return (SDL_GetMouseState(nullptr, nullptr) & SDL_BUTTON_MASK(src.code)) ? 1.0f : 0.0f;
    case InputSource::Kind::PadButton:
        return dev && dev->gamepad && SDL_GetGamepadButton(dev->gamepad, SDL_GamepadButton(src.code)) ? 1.0f : 0.0f;
    case InputSource::Kind::PadAxis:
        if (!dev || !dev->gamepad) return 0.0f;
        v = normalize_axis(SDL_GetGamepadAxis(dev->gamepad, SDL_GamepadAxis(src.code)));
        break;
    case InputSource::Kind::JoyButton:
        return dev && SDL_GetJoystickButton(dev->joystick, src.code) ? 1.0f : 0.0f;
    case InputSource::Kind::JoyAxis:
        if (!dev || src.code >= SDL_GetNumJoystickAxes(dev->joystick)) return 0.0f;
        v = normalize_axis(SDL_GetJoystickAxis(dev->joystick, src.code));
        break;
    case InputSource::Kind::JoyHat:
        return dev && (SDL_GetJoystickHat(dev->joystick, src.code) & src.hat_dir) ? 1.0f : 0.0f;
    case InputSource::Kind::Tilt: v = p.tilt; break;
    }
    return v;
}

void InputManager::evaluate(Player& p, double dt) {
    const Device* dev = find_device(p.device);
    update_tilt(p, dev);
    evaluate_profile(p, dev, p.profile, p.ramp_values, p.state, dt);
    if (!p.race_custom && p.race_layout == "gt3") p.race_state = p.state;
    else evaluate_profile(p, dev, p.race_profile, p.race_ramps, p.race_state, dt);
}

void InputManager::evaluate_profile(const Player& p, const Device* dev, const Profile& prof,
                                    std::vector<float>& ramps, PadState& s, double dt) const {
    float button_val[kPadButtonCount] = {};
    bool button_digital[kPadButtonCount] = {};
    float axis_val[kPadAxisCount] = {};
    if (ramps.size() != prof.bindings.size()) ramps.assign(prof.bindings.size(), 0.0f);

    for (size_t bi = 0; bi < prof.bindings.size(); ++bi) {
        const Binding& b = prof.bindings[bi];
        const BindingOptions& o = b.options;
        float v = read_source(p, dev, b.source);

        if (b.source.is_analog()) {
            if (o.invert) v = -v;
            if (o.pedal) v = (v + 1.0f) * 0.5f;
            else if (b.source.half == InputSource::Half::Positive) v = std::max(v, 0.0f);
            else if (b.source.half == InputSource::Half::Negative) v = std::max(-v, 0.0f);
            // Radial shaping on magnitude, sign preserved.
            float m = std::fabs(v);
            float range = std::max(o.saturation - o.deadzone, 1e-4f);
            m = m <= o.deadzone ? 0.0f : std::min((m - o.deadzone) / range, 1.0f);
            if (o.curve != 1.0f) m = std::pow(m, o.curve);
            v = std::copysign(m, v);
        } else if (o.invert) {
            v = 1.0f - v;
        }
        v *= o.scale;

        if (o.ramp > 0.0f) {
            float& cur = ramps[bi];
            float rate = static_cast<float>(dt) / o.ramp;
            cur = v > cur ? std::min(v, cur + rate) : std::max(v, cur - rate * 2.0f); // release faster
            v = cur;
        }

        if (b.target.kind == PadTarget::Kind::Button) {
            size_t i = static_cast<size_t>(b.target.button);
            float bv = std::clamp(v, 0.0f, 1.0f);
            button_val[i] = std::max(button_val[i], bv);
            float thr = o.threshold > 0.0f ? o.threshold : (b.source.is_analog() ? 0.12f : 0.0f);
            if (bv > thr && (b.source.is_analog() || o.ramp > 0.0f || bv >= 1.0f)) button_digital[i] = true;
        } else if (b.target.kind == PadTarget::Kind::Axis) {
            size_t i = static_cast<size_t>(b.target.axis);
            if (b.target.half == InputSource::Half::Negative) v = -std::fabs(v);
            else if (b.target.half == InputSource::Half::Positive) v = std::fabs(v);
            axis_val[i] += v;
        }
    }

    if (const PadState& ov = p.overlay; ov.connected) {
        for (size_t i = 0; i < kPadButtonCount; ++i) {
            if (!(ov.buttons & (1u << i))) continue;
            button_digital[i] = true;
            button_val[i] = std::max(button_val[i], float(ov.pressure[i]) / 255.0f);
        }
        for (size_t i = 0; i < kPadAxisCount; ++i) axis_val[i] += ov.axis_f(static_cast<PadAxis>(i));
    }

    s.connected = dev != nullptr || p.keyboard || p.overlay.connected;
    s.buttons = 0;
    for (size_t i = 0; i < kPadButtonCount; ++i) {
        bool pressed = button_digital[i];
        if (pressed) s.buttons |= u16(1u << i);
        if (is_pressure_sensitive(static_cast<PadButton>(i)))
            s.pressure[i] = pressed ? u8(std::max(1.0f, std::round(button_val[i] * 255.0f))) : 0;
        else
            s.pressure[i] = pressed ? 255 : 0;
    }
    for (size_t i = 0; i < kPadAxisCount; ++i) {
        float v = std::clamp(axis_val[i], -1.0f, 1.0f);
        s.axes[i] = u8(std::lround((v + 1.0f) * 127.5f));
    }
}

void InputManager::update(double dt) {
    if (capturing_) {
        // Analog capture: the axis that moved furthest from its resting value.
        float best = kCaptureAxisDelta;
        for (const auto& [id, base] : capture_baseline_) {
            const Device* d = find_device(id);
            if (!d) continue;
            for (size_t a = 0; a < base.size(); ++a) {
                float v = d->gamepad ? normalize_axis(SDL_GetGamepadAxis(d->gamepad, SDL_GamepadAxis(a)))
                                     : normalize_axis(SDL_GetJoystickAxis(d->joystick, int(a)));
                float delta = v - base[a];
                if (std::fabs(delta) <= best) continue;
                best = std::fabs(delta);
                InputSource s;
                s.kind = d->gamepad ? InputSource::Kind::PadAxis : InputSource::Kind::JoyAxis;
                s.code = int(a);
                bool trigger = d->gamepad && (a == SDL_GAMEPAD_AXIS_LEFT_TRIGGER || a == SDL_GAMEPAD_AXIS_RIGHT_TRIGGER);
                s.half = trigger ? InputSource::Half::Full
                                 : (delta > 0 ? InputSource::Half::Positive : InputSource::Half::Negative);
                captured_ = s;
            }
        }
        if (captured_) capturing_ = false;
    }
    for (Player& p : players_) evaluate(p, dt);
    std::fill(key_latch_.begin(), key_latch_.end(), false);

    time_ += dt;
    for (Player& p : players_) {
        if (const Device* d = find_device(p.device)) {
            if (p.ffb_age < 0.25) ffb_.apply(d->id, d->gamepad, d->joystick, brake_trigger(p), p.ffb, time_);
            else ffb_.stop(d->id);
        }
        p.ffb_age += dt;
    }
}

void InputManager::set_force_feedback(int player, const ForceFeedbackState& state) {
    players_[player].ffb = state;
    players_[player].ffb_age = 0;
}

int InputManager::brake_trigger(const Player& p) const {
    const Profile& race = !p.race_custom && p.race_layout == "gt3" ? p.profile : p.race_profile;
    for (const Binding& b : race.bindings)
        if (b.target.kind == PadTarget::Kind::Button && b.target.button == PadButton::Square &&
            b.source.kind == InputSource::Kind::PadAxis &&
            (b.source.code == SDL_GAMEPAD_AXIS_LEFT_TRIGGER || b.source.code == SDL_GAMEPAD_AXIS_RIGHT_TRIGGER))
            return b.source.code;
    return -1;
}

void InputManager::set_rumble(int player, const PadRumble& r) {
    Player& p = players_[player];
    p.rumble = r;
    const Device* d = find_device(p.device);
    if (!d) return;
    // DS2: large motor -> low-frequency rumble, small (on/off) -> high frequency.
    Uint16 low = Uint16(r.large * 257);
    Uint16 high = r.small ? 0xA000 : 0;
    if (d->gamepad) SDL_RumbleGamepad(d->gamepad, low, high, 250);
    else SDL_RumbleJoystick(d->joystick, low, high, 250);
}

void InputManager::set_trigger_rumble(int player, float left, float right) {
    const Device* d = find_device(players_[player].device);
    if (!d || !d->gamepad) return;
    SDL_RumbleGamepadTriggers(d->gamepad, Uint16(std::clamp(left, 0.0f, 1.0f) * 65535),
                              Uint16(std::clamp(right, 0.0f, 1.0f) * 65535), 250);
}

} // namespace gt3r
