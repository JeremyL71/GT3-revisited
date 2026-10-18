#pragma once

struct SDL_Window;

namespace gt3r::platform {

// macOS: marks the window's Metal layers non-opaque, so the compositor always composites the
// window instead of switching fullscreen frames straight to the display and back. No-op elsewhere.
void keep_composited(SDL_Window* window);

} // namespace gt3r::platform
