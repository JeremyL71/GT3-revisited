#pragma once

#include <functional>
#include <string>

struct SDL_Window;

namespace gt3r::platform {

// macOS: the system's open / save panel as a sheet on `window`, for the save page's file
// dialogs. SDL's own dialog hands the app to the Dock when it closes and asks for it back,
// which macOS no longer grants every time: the game then stayed without keyboard focus (no
// keys, no controller) until a click. `done` gets the chosen file's path, or "" when cancelled,
// on the main thread. `folder`: where the panel starts ("" = where it was last); `name`: the
// save panel's file name. False (and `done` never called) without a Cocoa window, and
// elsewhere.
bool show_file_sheet(SDL_Window* window, bool save, const std::string& folder, const std::string& name,
                     std::function<void(std::string path)> done);
// macOS: gives `window` the keyboard back after a file sheet on it ended: the sheet (an
// out-of-process panel) leaves no key window, the window looking active while SDL has no
// keyboard focus and drops keys and a controller's presses. The window is made key again
// (and the app active, if it is not). No-op elsewhere.
void refocus_after_dialog(SDL_Window* window);
// macOS: closes the sheet open over `window` (a file dialog shown on it) as its Cancel button
// does; the dialog's callback then runs as for a cancel. False without one, and elsewhere.
bool cancel_window_sheet(SDL_Window* window);

} // namespace gt3r::platform
