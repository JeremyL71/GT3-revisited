#pragma once

// Screens register themselves so new viewers/tools can be added in their own
// file without touching the hub or command-line handling:
//
//   GT3R_REGISTER_SCREEN(car_viewer, "cars", "Car viewer",
//                        "Inspect car models.", 50, [] { return make_car_viewer(); });
//
// `id` is used by `gt3r --screen <id>`; entries appear by `order`. Player
// screens (GT3R_REGISTER_PLAYER_SCREEN*) are on the home menu under the game;
// everything else is a tool listed under Developer options.

#include <functional>
#include <memory>
#include <vector>

namespace gt3r {

class Screen;

struct ScreenEntry {
    const char* id;
    const char* title;
    const char* help;
    int order;
    std::function<std::unique_ptr<Screen>()> factory;
    bool needs_disc = true; // false: can be opened with --screen before a disc is chosen
    bool player = false;    // true: on the home menu; false: under Developer options
};

std::vector<ScreenEntry>& screen_registry();

struct ScreenRegistrar {
    explicit ScreenRegistrar(ScreenEntry e) { screen_registry().push_back(std::move(e)); }
};

} // namespace gt3r

#define GT3R_REGISTER_SCREEN(name, id, title, help, order, factory) \
    static ::gt3r::ScreenRegistrar gt3r_screen_##name { ::gt3r::ScreenEntry{id, title, help, order, factory} }

// For screens that work without game data (renderer tests, input setup).
#define GT3R_REGISTER_SCREEN_NO_DISC(name, id, title, help, order, factory) \
    static ::gt3r::ScreenRegistrar gt3r_screen_##name { ::gt3r::ScreenEntry{id, title, help, order, factory, false} }

// Part of the game for players (home menu), with and without game data.
#define GT3R_REGISTER_PLAYER_SCREEN(name, id, title, help, order, factory) \
    static ::gt3r::ScreenRegistrar gt3r_screen_##name { ::gt3r::ScreenEntry{id, title, help, order, factory, true, true} }
#define GT3R_REGISTER_PLAYER_SCREEN_NO_DISC(name, id, title, help, order, factory) \
    static ::gt3r::ScreenRegistrar gt3r_screen_##name { ::gt3r::ScreenEntry{id, title, help, order, factory, false, true} }
