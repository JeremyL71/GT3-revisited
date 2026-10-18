#pragma once

#include <optional>

namespace gt3r::platform {

// What actually hides part of a phone screen, in window pixels: the display
// cutout (camera hole/notch) per edge, and the radius of the rounded screen
// corners. SDL_GetWindowSafeArea also counts the system-gesture zones (a
// tall band along the top on some devices even in immersive mode), which is
// too cautious for laying out a full-screen UI.
struct DisplayInsets {
    int left = 0, top = 0, right = 0, bottom = 0;
    int corner_radius = 0; // 0: square corners (or unknown)
};

// Android: queried from the activity (API 28+ cutout, API 31+ corners).
// nullopt elsewhere or when the system does not report it.
std::optional<DisplayInsets> query_display_insets();

} // namespace gt3r::platform
