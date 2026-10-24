#pragma once

#include "input/bindings.h"
#include "input/force_feedback.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gamepad.h>
#include <SDL3/SDL_haptic.h>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace gt3r {

class IniDocument;

// Owns physical input devices and turns them into virtual DualShock 2
// states, one per player. Configuration lives in the main config file:
//
//   [player1]
//   preset = racing          ; starting point (racing/classic/wheel/keyboard)
//   device = auto            ; auto, or part of a device name ("DualSense")
//   keyboard = on
//   phone_vibration = off    ; Android: rumble on the device's own motor (see below)
//
//   race = modern            ; race controls layout: modern | gt3 (the original)
//
//   [player1.bindings]       ; optional; replaces the preset when present
//   Cross = pad.south | key.Return
//
//   [player1.race]           ; optional; replaces the race layout when present
//   Cross = pad.righttrigger curve=1.4 | key.W ramp=0.12   ; Cross = accelerate
//
// See docs/INPUT.md for the full binding syntax.

// Phone vibration ([player1] phone_vibration, off by default): player 1's rumble goes to the
// phone's or tablet's own vibration motor only while the device driving them has no rumble motor
// of its own (built-in handheld controls, touch, a controller without motors). A controller that
// can rumble always rumbles itself and the phone stays still.
bool use_phone_vibrator(bool enabled, int player, bool device_rumbles);
// The DS2 rumble as one motor's strength 0..1: the large motor's level, or the small (on/off)
// motor's 0.6 when stronger.
float phone_vibration_strength(const PadRumble& r);

class InputManager {
public:
    static constexpr int kMaxPlayers = 2; // GT3 supports 2-player split screen

    InputManager();
    ~InputManager();

    void load(const IniDocument& config);
    void save(IniDocument& config) const;

    void handle_event(const SDL_Event& e);
    void update(double dt); // call once per frame

    const PadState& pad(int player) const { return players_[player].state; }
    // The pad a race reads while driving: buttons in GT3's race meaning (race_action_name),
    // driven by the player's race layout. With the "gt3" layout it is pad() itself.
    const PadState& race_pad(int player) const { return players_[player].race_state; }
    // Extra input merged into a player's pad on the next update(): buttons
    // are OR'ed (pressure: the stronger wins), axes are added. Used by the
    // on-screen touch controls; a state with connected=false is ignored.
    void set_overlay(int player, const PadState& overlay) { players_[player].overlay = overlay; }
    // Device/controller roll for this player, -1..1 = full lock (the `tilt` source).
    float tilt(int player) const { return players_[player].tilt; }
    void set_rumble(int player, const PadRumble& rumble);
    // Phone vibration (see use_phone_vibrator). Available on Android, where SDL exposes the
    // device's vibrator as a haptic device.
    static bool phone_vibration_available();
    bool phone_vibration() const { return phone_vibration_; }
    void set_phone_vibration(bool on);
    // Impulse triggers on Xbox One/Series pads (0..1 each). No-op elsewhere.
    void set_trigger_rumble(int player, float left, float right);
    // Driving feedback for this frame (call every frame while driving):
    // DualSense brake-trigger resistance and ABS pulse, Xbox impulse-trigger
    // ABS pulse, wheel spring + steering force. Effects stop by themselves
    // ~0.25 s after the last call. See docs/INPUT.md "Force feedback".
    void set_force_feedback(int player, const ForceFeedbackState& state);
    const ForceFeedbackConfig& ffb_config() const { return ffb_.config(); }
    void set_ffb_config(const ForceFeedbackConfig& c) { ffb_.configure(c); }

    const Profile& profile(int player) const { return players_[player].profile; }
    void apply_preset(int player, std::string_view preset);
    const std::string& preset(int player) const { return players_[player].preset; }
    bool is_custom(int player) const { return players_[player].custom_bindings; }
    // Editing any binding turns the player's profile into a custom one that
    // is saved to [playerN.bindings].
    void bind(int player, const PadTarget& target, const InputSource& source, bool replace);
    void clear_target(int player, const PadTarget& target);
    // Race controls (see make_race_preset). Editing a race binding makes the race profile a
    // custom one, saved to [playerN.race]; under "gt3" it starts from the menu bindings.
    const std::string& race_layout(int player) const { return players_[player].race_layout; }
    void set_race_layout(int player, std::string_view layout);
    bool race_uses_menu_pad(int player) const;
    bool is_race_custom(int player) const { return players_[player].race_custom; }
    const Profile& race_profile(int player) const; // the effective one (gt3: the menu profile)
    void bind_race(int player, const PadTarget& target, const InputSource& source, bool replace);
    void clear_race_target(int player, const PadTarget& target);
    std::string device_name(int player) const;
    // [playerN] device: "auto" or part of a device name. Reassigns devices.
    const std::string& device_filter(int player) const { return players_[player].device_filter; }
    void set_device_filter(int player, std::string filter);
    // [playerN] keyboard: whether the keyboard also drives this player.
    bool keyboard_enabled(int player) const { return players_[player].keyboard; }
    void set_keyboard(int player, bool on) { players_[player].keyboard = on; }
    // Player 1 with the keyboard off but a profile only the keyboard can drive (the keyboard
    // preset, or custom bindings with no controller source): the keyboard still drives them, so
    // the game can't be left with no way to control it.
    bool keyboard_fallback(int player) const;
    // The keyboard drives this player: enabled, or the fallback above.
    bool keyboard_active(int player) const { return players_[player].keyboard || keyboard_fallback(player); }
    // Names of the connected gamepads, wheels and joysticks.
    std::vector<std::string> device_names() const;
    // Human label for a source on this player's device (e.g. "Cross" on a
    // PlayStation pad, "A" on Xbox).
    std::string describe(int player, const InputSource& src) const;

    // Remap support: arms capture; the next deliberate input on any device
    // is returned once by poll_capture().
    void begin_capture();
    void cancel_capture() { capturing_ = false; }
    bool capturing() const { return capturing_; }
    std::optional<InputSource> poll_capture();

private:
    struct Device {
        SDL_JoystickID id = 0;
        SDL_Gamepad* gamepad = nullptr;   // set when SDL recognizes a gamepad
        SDL_Joystick* joystick = nullptr; // always set
        std::string name;
        bool has_accel = false;
    };
    struct Player {
        Profile profile;
        std::string preset = "racing";
        std::string device_filter = "auto";
        bool keyboard = true;
        bool custom_bindings = false;
        SDL_JoystickID device = 0; // 0 = none
        PadState state;
        std::vector<float> ramp_values; // parallel to profile.bindings
        std::string race_layout = "modern";
        bool race_custom = false;
        Profile race_profile; // empty "gt3" profile = the race reads `state`
        std::vector<float> race_ramps; // parallel to race_profile.bindings
        PadState race_state;
        PadRumble rumble;
        float tilt = 0.0f;
        PadState overlay; // set_overlay()
        ForceFeedbackState ffb;
        double ffb_age = 1e9; // seconds since set_force_feedback()
    };

    void add_device(SDL_JoystickID id);
    void remove_device(SDL_JoystickID id);
    void assign_devices();
    const Device* find_device(SDL_JoystickID id) const;
    float read_source(const Player& p, const Device* dev, const InputSource& src) const;
    void evaluate(Player& p, double dt);
    void evaluate_profile(const Player& p, const Device* dev, const Profile& prof, std::vector<float>& ramps,
                          PadState& out, double dt) const;
    void update_tilt(Player& p, const Device* dev);
    int brake_trigger(const Player& p) const; // SDL trigger axis bound to the brake, or -1
    void vibrate_phone(float strength);       // 0 stops it

    std::vector<Device> devices_;
    std::array<Player, kMaxPlayers> players_;
    SDL_Sensor* device_accel_ = nullptr; // phone/tablet accelerometer
    ForceFeedback ffb_;
    double time_ = 0;
    bool phone_vibration_ = false;
    SDL_Haptic* vibrator_ = nullptr; // the device's own motor, opened on first use
    bool vibrator_tried_ = false;
    float vibrator_level_ = 0;      // strength last sent (0 = stopped)
    double vibrator_sent_ = -1e9;   // time_ it was sent
    // Keys pressed since the last update(), so a press and release within one
    // frame (fast taps, Android's back key, injected events) still registers.
    std::vector<bool> key_latch_;

    bool capturing_ = false;
    std::optional<InputSource> captured_;
    std::vector<std::pair<SDL_JoystickID, std::vector<float>>> capture_baseline_;
};

} // namespace gt3r
