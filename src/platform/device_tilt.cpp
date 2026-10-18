#include "platform/device_tilt.h"

#include <SDL3/SDL_video.h>

#include <cmath>

namespace gt3r::platform {

namespace {

int orientation_degrees(SDL_DisplayOrientation o) {
    switch (o) {
    case SDL_ORIENTATION_LANDSCAPE: return 90;
    case SDL_ORIENTATION_PORTRAIT_FLIPPED: return 180;
    case SDL_ORIENTATION_LANDSCAPE_FLIPPED: return 270;
    default: return 0; // portrait or unknown
    }
}

} // namespace

int display_rotation_degrees() {
    SDL_DisplayID d = SDL_GetPrimaryDisplay();
    if (!d) return 0;
    SDL_DisplayOrientation cur = SDL_GetCurrentDisplayOrientation(d);
    SDL_DisplayOrientation nat = SDL_GetNaturalDisplayOrientation(d);
    if (cur == SDL_ORIENTATION_UNKNOWN || nat == SDL_ORIENTATION_UNKNOWN) return 0;
    return (orientation_degrees(cur) - orientation_degrees(nat) + 360) % 360;
}

void accel_to_display(const float in[3], float out[3], int rotation_degrees) {
    // Same remapping as Android's Surface.ROTATION_* (SDL derives its
    // orientation values from it): the display frame is the device frame
    // turned by the display rotation.
    const float x = in[0], y = in[1];
    switch (((rotation_degrees % 360) + 360) % 360) {
    case 90: out[0] = -y, out[1] = x; break;
    case 180: out[0] = -x, out[1] = -y; break;
    case 270: out[0] = y, out[1] = -x; break;
    default: out[0] = x, out[1] = y; break;
    }
    out[2] = in[2];
}

float steering_roll(const float a[3]) {
    // Turning clockwise moves the "up" reading towards the display's -X.
    return std::atan2(-a[0], std::sqrt(a[1] * a[1] + a[2] * a[2]));
}

} // namespace gt3r::platform
