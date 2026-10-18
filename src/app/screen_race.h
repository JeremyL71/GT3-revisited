#pragma once

// The race screen (screen_race.cpp) started the way the interactive menus start it: a
// menu::flow::GameRequest (mode, event, car, paint, gearbox, ...). The screen pops itself
// when the race is left, after posting a menu::flow::GameResult.

#include <memory>

namespace gt3r {

class Screen;
namespace menu::flow {
struct GameRequest;
}

std::unique_ptr<Screen> make_menu_race_screen(const menu::flow::GameRequest& request);

} // namespace gt3r
