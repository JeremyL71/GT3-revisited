#pragma once

// Binds GT Mode (gt_mode.h) to the interactive menus through
// menu::flow::GtModeHooks: the GT mode menu's actions, its
// "pre-check" popups, the code-filled text widgets (credits, days, prices,
// license times, status page...), the garage list, race requests and
// results. docs/dev/GT_MODE.md describes the mapping.
//
// install_gt_mode_menu() is called once by the app (src/app/screen_gtmode.cpp).
// The game is loaded from the current save of the save library (save_library.h,
// <user data>/saves/) when the GT-mode control opens (new game when there is none) and
// saved into it after every race; the save page (menu/flow/save_page.h) loads, saves,
// renames, copies, deletes, imports and exports saves.
//
// Environment (development / screenshots):
//   GT3R_GTMODE_SAVE=<path>   play this save file (its folder is the library; the
//                             current save is not remembered)
//   GT3R_GTMODE_NOSAVE=1      never write the save

#include "game/gt_mode.h"
#include "menu/flow/game_hooks.h"

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gt3r::game {

class GtModeSession;
class SaveLibrary;

void install_gt_mode_menu();
// A license test run judged by the race (medal 0 gold .. 2 bronze, -1 none): written to the
// save at once, so the medal is kept even if the game ends before the race screen returns
// to the menus (the menus record it again from the GameResult, with the presentations).
void save_license_run(const std::string& code, int medal, double seconds);
// A licence test run with a medal (race code, seconds): entered in the test's records at
// once; when it makes the ranking the name entry is queued as a presentation (run it with
// GameFlow::start_presentations()). Returns the rank (-1 = not ranked / no session).
int add_license_record(const std::string& code, double seconds);
// A GT race's prize page (the event's cup for places 1-3, else the consolation prize; place
// 0-based), queued for GameFlow::start_presentations() as the original shows it
// straight after the results. False when the race has none (no trophy class / money,
// championship rounds). Post the result with GameResult::prize_shown.
bool queue_race_prize(const std::string& code, int place);
// A finished GT single race (not a championship round): its place, prize and best result
// written to the save at once (a copy of the game; the menus record it from the GameResult
// when the race screen closes), as the original records a race before its replay.
void save_race_run(const menu::flow::GameResult& r);
// A championship round's "Save game" (result quick menu): a snapshot save
// (begin_snapshot_save) of the game with this round recorded (a copy: the menus record the
// round from the GameResult when the race screen closes); end_race_game_save() drops it.
// False without a session.
bool begin_race_game_save(const menu::flow::GameResult& r);
void end_race_game_save();
// The running session (nullptr until the GT-mode menus opened with disc data).
GtModeSession* gt_mode_session();
// The car a "car_render_*" widget shows on a page: the dealer's focus car with
// the chosen paint, the focused garage car, or the ride car (shops). False = none.
// `dirt` (0..0.8): the car's dirt (the showroom floor reflects 1 - dirt of it).
bool car_for_widget(const std::string& page, const std::string& widget_class, std::string& car, int& paint,
                    std::string* wheel = nullptr, float* dirt = nullptr);
// The prize model (".prm" path in the VOL) of the presentation on `page`: the
// license card on "license_get", the championship trophy on "trophy_get"; ""
// when none (or not yet faded in). `serial` changes with every new presentation
// (restart the animation); `fade` is its opacity 0..1.
std::string presentation_model(const std::string& page, int* serial = nullptr, float* fade = nullptr);
// The car an arcade picker entry shows ("nsx_s_zero_97" picture -> base car
// label for the 3D model), "" when unknown.
std::string arcade_entry_car(const std::string& entry);
// Arcade garage classes: the car page entries "hg<i>" (home garage) / "gg<i>" (guest
// garage), i = the garage list index. Returns the garage car (nullptr for other entries).
const GarageCar* arcade_garage_car(const std::string& entry);
// The race information's Cars Permitted list (page 00cmn/car_reguration) for a race code:
// each car of the event's regulation as "<name> <second name>", a string listed once (at
// its last place); empty when the race has no list ("No Limitations").
std::vector<std::string> permitted_car_names(const GtMode& gt, const std::string& code);
// Prepares the car settings menu (menu/setting.mbl) for the ride car: a working
// copy for the sliders; `event` (race code, may be empty) is checked on every
// change (settings that would break its rules are refused). False without a
// GT-mode session or ride car.
bool begin_setting(const std::string& event);
// Prepares the records menu (menu/records.mbl) for a ranking key: "lic:<test>",
// "mt:M400" / "mt:M1000" / "mt:MMAX", "at:<arcade time trial code>".
bool begin_records(const std::string& key);
// Creates the GT-mode session (disc data + save) if none exists yet, e.g. when
// the settings menu is opened on its own. `read` loads disc files.
bool ensure_gt_mode_session(const std::function<std::optional<Bytes>(const std::string&)>& read);
// The save library (GT3R_GTMODE_SAVE's folder, or <user data>/saves/ with the saves of
// earlier versions migrated on first use) and the save to play: GT3R_GTMODE_SAVE, the
// remembered current save, the newest save, or <library>/gtmode.sav for a new game.
SaveLibrary& save_library();
std::string current_save_path();
// The next save page in SaveGame mode saves `s` instead of the running game (e.g. a
// championship between rounds, with the round just finished), and the running game and
// its current save stay as they are. Cleared when that page closes.
void begin_snapshot_save(GameState s);
// The snapshot waiting for the save page (nullptr = none; tests).
const GameState* snapshot_save();

class GtModeSession {
public:
    GtModeSession(std::shared_ptr<const GtData> data, std::string save_path);

    GtMode& gt() { return gt_; }
    const GtMode& gt() const { return gt_; }
    const std::string& save_path() const { return save_path_; }
    // The save the game is written to from now on (the library's current save).
    void set_save_path(std::string p) { save_path_ = std::move(p); }
    bool save(std::string* error = nullptr);
    bool load(std::string* error = nullptr);

    // Menu state the original keeps in its session block.
    std::string focus_car;     // dealer car (set_car_code)
    int garage_cursor = 0;     // garage list cursor (hm/hm_gr), index into the display order
    int sort_key = 0;          // garage sort (0 date .. 8 aspiration)
    int color_cursor = 0;      // dealer car page: paint index
    int garage_focus = -1;     // garage car chosen (set_garage_car)
    int focus_goods = -1;      // tune-shop goods (set_focus_parts)
    float part_power_before = 0; // tune shop: the ride car's power before the last part was fitted
    int regulated_cursor = 0;  // 00cmn/car_reguration: the permitted-car list cursor
    std::string focus_wheel;   // wheel shop (set_focus_wheel_code)
    std::string focus_wheel_preview; // wheel shop: the wheel under the cursor (shown on the car)
    int wheel_cursor = 0;      // hm/hm_wheel: wheel inventory cursor (-1 = stock)
    std::string race_code;     // set_race_code*
    bool champ = false;        // set_race_code_champ
    std::string last_page;
    // Car settings menu.
    std::optional<GarageCar> setting_work;          // working copy (sliders edit it)
    std::string setting_event;                      // race whose rules apply ("" = none)
    std::map<std::string, int> setting_entry;       // slider positions when the page opened (pinch_prev)
    bool setting_committed = false;                 // a commit hides pinch_prev until the page is rebuilt
    std::string records_key;                        // records menu: the ranking shown
    bool name_started = false;                      // name entry: a key was pressed (the default name is replaced)
    // Guest garage (a friend's save loaded for Trade and the arcade guest class) and
    // whether the garage pages show it (the trade session).
    std::unique_ptr<GtMode> guest;
    bool trade = false;
    GtMode& list_gt() { return trade && guest ? *guest : gt_; }
private:
    GtMode gt_;
    std::string save_path_;
};

} // namespace gt3r::game
