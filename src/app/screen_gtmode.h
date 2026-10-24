#pragma once

// GT Mode screens other code can open (implemented in screen_gtmode*.cpp).

#include <memory>
#include <string>

namespace gt3r {

class Screen;
namespace sim {
struct CarSpec;
}
namespace menu::flow {
struct GameResult;
}

// GT3's car settings menu (menu/setting.mbl) for the GT-mode ride car, as the
// pre-race quick menu's "Setting" opens it. `event` = the race code whose
// entry rules every change must keep (may be empty). Committed changes are
// stored in the garage car and saved; the screen pops itself when the menu
// is left. Also `gt3r --screen gtsetting` (event from GT3R_GTSETTING_EVENT).
std::unique_ptr<Screen> make_gt_setting_screen(std::string event);

// The records menu (menu/records.mbl, the quick menus' "Records") for a ranking:
// "lic:<license test>", "mt:M400" / "mt:M1000" / "mt:MMAX", "at:<arcade time
// trial code>", "ct:<course>" (a free run's course). Pops itself when left.
std::unique_ptr<Screen> make_records_screen(std::string key);

// The GT-mode presentations queued so far (e.g. a licence run's record name entry,
// game::add_license_record), shown one after the other; pops itself after the last.
std::unique_ptr<Screen> make_presentation_screen();

// The save page in Save game mode (menu/flow/save_page.h), as a championship's result
// quick menu "Save game" opens it: a new or existing save gets the game with the round `r`
// recorded; the running game and its save are untouched. Pops itself when the page ends.
std::unique_ptr<Screen> make_game_save_screen(const menu::flow::GameResult& r);

// The physics spec of the GT-mode ride car with its current parts, settings
// and condition (nullptr without a session / ride car), e.g. to rebuild the
// player's car after the settings menu.
std::shared_ptr<const sim::CarSpec> gt_ride_car_spec();

} // namespace gt3r
