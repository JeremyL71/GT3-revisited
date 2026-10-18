// gt3tool input: what SDL sees of controllers on this machine (diagnostics for "my pad does
// nothing"). Lists every joystick / gamepad as it connects, with SDL's driver details, and
// prints button and axis changes for the given number of seconds.

#include "tool.h"

#include <SDL3/SDL.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace gt3r;

namespace {

void describe(SDL_JoystickID id) {
    const char* name = SDL_GetJoystickNameForID(id);
    const char* path = SDL_GetJoystickPathForID(id);
    char guid[64] = {};
    SDL_GUIDToString(SDL_GetJoystickGUIDForID(id), guid, sizeof guid);
    std::printf("  id %u: \"%s\"  gamepad=%s  type=%d  vendor=%04x product=%04x\n    guid %s\n    path %s\n",
                unsigned(id), name ? name : "?", SDL_IsGamepad(id) ? "yes" : "no", int(SDL_GetJoystickTypeForID(id)),
                SDL_GetJoystickVendorForID(id), SDL_GetJoystickProductForID(id), guid, path ? path : "-");
    if (SDL_IsGamepad(id))
        if (char* m = SDL_GetGamepadMappingForID(id)) {
            std::printf("    mapping %.120s\n", m);
            SDL_free(m);
        }
}

int cmd_input(Disc&, const tool::Args& a) {
    double seconds = a.empty() ? 10 : std::atof(a[0].c_str());
    int v = SDL_GetVersion();
    std::printf("SDL %d.%d.%d\n", SDL_VERSIONNUM_MAJOR(v), SDL_VERSIONNUM_MINOR(v), SDL_VERSIONNUM_MICRO(v));
    if (!SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_GAMEPAD | SDL_INIT_EVENTS)) {
        std::printf("SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    int n = 0;
    SDL_JoystickID* ids = SDL_GetJoysticks(&n);
    std::printf("joysticks at start: %d\n", n);
    for (int i = 0; i < n; ++i) describe(ids[i]);
    SDL_free(ids);
    std::printf("watching for %.0f s (press buttons, move sticks)...\n", seconds);
    std::fflush(stdout);
    Uint64 end = SDL_GetTicks() + Uint64(seconds * 1000);
    float last_axis[8][8] = {};
    while (SDL_GetTicks() < end) {
        SDL_Event e;
        while (SDL_WaitEventTimeout(&e, 100)) {
            switch (e.type) {
            case SDL_EVENT_JOYSTICK_ADDED:
                std::printf("[%.1f s] joystick added\n", SDL_GetTicks() / 1000.0);
                describe(e.jdevice.which);
                SDL_OpenJoystick(e.jdevice.which);
                break;
            case SDL_EVENT_GAMEPAD_ADDED:
                std::printf("[%.1f s] gamepad added: %s\n", SDL_GetTicks() / 1000.0, SDL_GetGamepadNameForID(e.gdevice.which));
                SDL_OpenGamepad(e.gdevice.which);
                break;
            case SDL_EVENT_JOYSTICK_REMOVED:
                std::printf("[%.1f s] joystick removed (id %u)\n", SDL_GetTicks() / 1000.0, unsigned(e.jdevice.which));
                break;
            case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
                std::printf("[%.1f s] gamepad %u button %s\n", SDL_GetTicks() / 1000.0, unsigned(e.gbutton.which),
                            SDL_GetGamepadStringForButton(SDL_GamepadButton(e.gbutton.button)));
                break;
            case SDL_EVENT_GAMEPAD_AXIS_MOTION: {
                float val = e.gaxis.value / 32767.0f;
                float& prev = last_axis[e.gaxis.which % 8][e.gaxis.axis % 8];
                if (std::fabs(val - prev) > 0.25f) {
                    prev = val;
                    std::printf("[%.1f s] gamepad %u axis %s = %.2f\n", SDL_GetTicks() / 1000.0, unsigned(e.gaxis.which),
                                SDL_GetGamepadStringForAxis(SDL_GamepadAxis(e.gaxis.axis)), val);
                }
                break;
            }
            case SDL_EVENT_JOYSTICK_AXIS_MOTION:
                { // raw axes of every device (also mapped gamepads): which raw axis is which
                    float val = e.jaxis.value / 32767.0f;
                    float& prev = last_axis[e.jaxis.which % 8][e.jaxis.axis % 8];
                    if (std::fabs(val - prev) > 0.5f) {
                        prev = val;
                        std::printf("[%.1f s] joystick %u raw axis %d = %.2f\n", SDL_GetTicks() / 1000.0, unsigned(e.jaxis.which),
                                    int(e.jaxis.axis), val);
                    }
                }
                break;
            case SDL_EVENT_JOYSTICK_HAT_MOTION:
                if (!SDL_IsGamepad(e.jhat.which) && e.jhat.value)
                    std::printf("[%.1f s] joystick %u raw hat %d = %d\n", SDL_GetTicks() / 1000.0, unsigned(e.jhat.which),
                                int(e.jhat.hat), int(e.jhat.value));
                break;
            case SDL_EVENT_JOYSTICK_BUTTON_DOWN:
                // raw buttons of every device (also mapped gamepads)
                std::printf("[%.1f s] joystick %u raw button %d\n", SDL_GetTicks() / 1000.0, unsigned(e.jbutton.which),
                            int(e.jbutton.button));
                break;
            default: break;
            }
            std::fflush(stdout);
        }
    }
    ids = SDL_GetJoysticks(&n);
    std::printf("joysticks at end: %d\n", n);
    SDL_free(ids);
    SDL_Quit();
    return 0;
}

} // namespace

GT3TOOL_COMMAND(input, "input", "[seconds]", "list the controllers SDL sees and their button presses (diagnostics)", 0,
                cmd_input);
