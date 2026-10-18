#pragma once

#include <optional>
#include <string_view>

namespace gt3r::platform {

// [video] render_scale: "auto" (default) or 0.5..1.0. Auto keeps desktop at
// native resolution; on phones and tablets it caps the 3D resolution at about
// kAutoTargetLines lines of the 16:9 game area (the PS2 drew 448).
constexpr int kAutoTargetLines = 900;

// Parses a config value; nullopt means "auto" (or an unparsable value).
std::optional<float> parse_render_scale(std::string_view value);

// Scale for "auto" given the window size in physical pixels.
float auto_render_scale(int pixel_w, int pixel_h, bool mobile);

// True on Android and iOS builds.
bool is_mobile();

} // namespace gt3r::platform
