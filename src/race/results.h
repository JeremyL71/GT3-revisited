#pragma once

// End-of-race screens, as GT3 draws them (research summary in
// docs/dev/RACE.md "Results"):
//
// 1. ResultsOverlay: over the still-running race, from menu/<LANG>/result1.pmb
//    (page "result"): a "RACE RESULTS" title bar 5 s after the player
//    finishes, then one 600x36 rank bar per car (row i at 6.0 + 0.12 i s,
//    fading in and unrolling out of the bar above), then the "OFFICIAL TIMER
//    SEIKO" bar. Names and times in sw_16 at 87 %: the player's row
//    0xFFC800, others 0xCCCCCC; times "m'ss.mmm" for the winner, "+gap",
//    "+N lap(s)" or "Running...".
// 2. TimeTrialOverlay: 5 s after a time trial ends: rank in the records
//    ("1st".."12th", font rank) with a flickering "NEW RECORD"/"RECORD TIME",
//    and "Lap" + the best lap at y 270.
// 3. QuickMenu: the icon menu page that follows (qm_arc_result1.pmb and the
//    2P / time-trial variants): items in a band, the focused item's help
//    layer, and the bound result list widgets (qm_list_car_name1..6 /
//    qm_list_time1..6).
//
// All three draw the 640x448 menu frame onto a 4:3 rectangle (`dst`).

#include "formats/menu/mbl.h"
#include "menu/flow/game_hooks.h"
#include "menu/flow/menu_controller.h"
#include "menu/flow/menu_set.h"
#include "menu/menu_renderer.h"
#include "menu/menu_scene.h"
#include "text/game_font.h"

#include <SDL3/SDL_render.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace gt3r {
class Vfs;
} // namespace gt3r

namespace gt3r::race {

struct ResultRow {
    std::string name;
    std::string time; // time column text
    bool player = false;
    int car = 0;      // -1 player, -2 second player, >= 0 opponent index (for car icons)
};

// Time column text for a car: winner total time, "+gap" on the same lap,
// "+N lap(s)" when lapped, "Running..." when not finished. Times in seconds.
std::string result_time_text(bool winner, bool finished, double time, double winner_time, int laps_behind);

class ResultsOverlay {
public:
    bool load(const Vfs& vfs, SDL_Renderer* r, const std::string& lang = "US");
    // t = seconds since the player finished.
    // `icons`: when set, filled with each row's car-icon rectangle (UI units; empty while the row
    // is not shown) and the names move right to make room (the 2P panel's car pictures).
    void draw(SDL_Renderer* r, text::GameFonts& fonts, const std::vector<ResultRow>& rows, double t, const SDL_FRect& dst,
              std::vector<SDL_FRect>* icons = nullptr);
    static double fully_shown_time(size_t rows) { return 7.0 + 0.12 * double(rows); }

private:
    std::optional<menu::MenuScene> scene_;
    std::unique_ptr<menu::MenuRenderer> renderer_;
    std::vector<float> slide_; // per bar: current distance to the previous bar's centre
    bool drawer_set_ = false;
};

class TimeTrialOverlay {
public:
    // t = seconds since the run ended; record_rank < 0 = not in the records.
    void draw(SDL_Renderer* r, text::GameFonts& fonts, double t, double best_lap, int record_rank, bool speed_record,
              const SDL_FRect& dst);
};

// Licence test result (RaceLicenseDisplay): t = seconds since the run ended.
// Finished: after 0.1 s the medal word (GOLD / SILVER / BRONZE, or NO PRIZE) fades in
// over 0.5 s, centred at (320, 336) in sw_20 x 1.25 in the medal colour, with the time
// at (320, 112). Failed: "FAIL" (sw_20 x 1.5) after 1 s. The 3-D medal models
// (race/gold.prm ...) are not drawn.
// GT3's licence / time-trial result presenter. Frames
// (60 Hz) count from the frame the result is decided (f = 0); the live race keeps running
// behind it.
//   Medal: wait 6, black dim 0 -> 1/3 over 30 frames (the jingle at its end, frame 37), a
//   further 180 (gold) / 30 frames, then the model (weight over 60 frames, 2 s fly-in then
//   the loop), the word (alpha over ~31 frames, shadowed) and the time (over 60 frames);
//   a 300-frame hold, then 300 frames that Cross can skip.
//   No medal (slower than bronze / FAIL): after 60 frames a bluish wash (alpha 0 -> 0.5
//   over 30 frames) and the text in two copies 30 px above / below y 224 that merge over
//   ~31 frames (the time at y 140 for TIME EXPIRED); a 300-frame hold, then 48 frames
//   (licence) or 240 Cross-skippable frames (time trial).
class LicenseResultOverlay {
public:
    static constexpr int kDimStart = 7, kDimFrames = 30, kJingleFrame = kDimStart + kDimFrames;
    static int model_start(int medal) { return kJingleFrame + (medal == 0 ? 180 : 30); }
    static constexpr int kTextFrames = 31, kHold = 300;
    static constexpr int kFailDelay = 60, kWashFrames = 30;
    static int fail_text_start() { return kFailDelay + kWashFrames; }
    // Frame from which Cross may end the presentation (-1: never), and the frame it ends.
    static int skippable_from(int medal, bool time_trial);
    static int end_frame(int medal, bool time_trial);
    // The medal model's composite weight (0 before it starts).
    static float model_weight(int frame, int medal);

    // Medal: the dim (drawn before the model) and the word + time (after it).
    void draw_dim(SDL_Renderer* r, int frame, const SDL_FRect& dst);
    void draw_medal(SDL_Renderer* r, text::GameFonts& fonts, int frame, int medal, double time, const SDL_FRect& dst);
    // No medal: `text` ("FAIL", "TIME EXPIRED"); `time` < 0 = no time line.
    void draw_fail(SDL_Renderer* r, text::GameFonts& fonts, int frame, const std::string& text, double time,
                   const SDL_FRect& dst);
};

// What a quick-menu item does: the item's .mbl action string, as the original's quick-menu
// control turns it into its result code. Every action of the
// disc's quick-menu layouts has one (test race_quick_menu_actions_disc); None = unknown.
enum class QmAction {
    None,
    Start,       // qm_start / qm_try_again: run (again)
    FreeRun,     // qm_free_run: the course alone before a GT race
    Replay,      // qm_replay
    Demo,        // qm_demo: the computer drives the course
    SaveReplay,  // qm_save_replay
    SaveGhost,   // qm_save_ghost
    LoadGhost,   // qm_load_ghost
    Records,     // qm_records
    Logger,      // qm_logger: data logger
    Option,      // qm_option
    Setting,     // qm_setting
    NextSession, // qm_next_session: the next championship round
    SaveGame,    // qm_save_game: the save page (championships)
    Exit,        // qm_exit (also "Yes" on exit_sure)
    Yes,         // qm_yes (ghost_dlg)
    No,          // qm_no
};
QmAction quick_menu_action(std::string_view action);

// The race quick menus run on the same page manager as every other menu
// (menu::flow::MenuController over menu/<set>.mbl + menu/<LANG>/<set>1.pmb):
// the icons in a horizontal row in the "menu" window (4 px apart, centred when
// they fit, else scrolling with the focus, easing 30 % per frame), the focus
// pulse, the focused item's help layer, the confirm blink (page flags 0xC0),
// popups (exit_sure, ghost_dlg) and the menu sound effects ("select", "next",
// "return", ...; take_sounds()).
class QuickMenu {
public:
    QuickMenu();
    ~QuickMenu();
    // `pmb` e.g. "qm_arc_result1.pmb", "qm_arc_2p_result1.pmb", "qm_arc_tt_ltd1.pmb".
    bool load(const Vfs& vfs, SDL_Renderer* r, text::GameFonts& fonts, const std::string& pmb,
              const std::string& lang = "US");
    bool loaded() const { return ctl_ != nullptr; }
    void set_rows(std::vector<ResultRow> rows) { rows_ = std::move(rows); }
    // Code-filled text widgets by name (e.g. qm_license_name, qm_license_time_gold).
    void set_text(const std::string& widget, std::string text) { texts_[widget] = std::move(text); }
    // Code-filled text widgets by their game-code class (e.g. qm_rank_name1, qm_rank_time1:
    // the layouts name these widgets differently); only text elements take it, and a
    // class text wins over the result rows.
    void set_class_text(const std::string& cls, std::string text) { class_texts_[cls] = std::move(text); }
    // A page of the flow file whose own layout (menu/<LANG>/<set><N>.pmb, N = the page's
    // number) is drawn over the root page: the licence tests' description panels.
    bool add_page_layout(const Vfs& vfs, SDL_Renderer* r, const std::string& mbl_page, const std::string& lang = "US");

    // What the player chose: an item of the root page after its confirm feedback
    // ("start", "replay", "save_replay", "option", "exit", ...), or of a dialog page
    // ("yes1" / "no1" on "ghost_dlg", "yes1" on "exit_sure"); empty item = nothing.
    struct Choice {
        std::string item, page;
        std::string action; // the item's .mbl action (quick_menu_action())
        bool empty() const { return item.empty(); }
    };
    // Runs the menu for `dt` seconds of 60 Hz frames with the logical keys held
    // (menu::flow::MenuKeys bits). Keys already held when the menu opened count
    // only after they were released.
    Choice update(u16 held_keys, double dt);
    // One frame (tests, scripted choices).
    Choice tick(u16 held_keys);
    std::vector<std::string> take_sounds();

    void draw(SDL_Renderer* r, const SDL_FRect& dst);
    const std::string& focused() const;
    bool focus(const std::string& item); // false if the page has no such item
    // Opens a dialog page of the same layout ("exit_sure", "ghost_dlg") as a popup over
    // the root page, or "root" (closes any popup). False if there is no such page.
    bool open_page(const std::string& name);
    bool on_root() const;
    const std::string& page() const;
    bool busy() const; // a confirm feedback / page change is running
    // The flow file's link of an item on the current page ("exit" -> "exit_sure"), or "".
    std::string link(const std::string& item) const;
    // The flow file's action of an item on the current page ("start" -> "qm_start"), or "".
    std::string action(const std::string& item) const;

private:
    void reopen_root(const std::string& focus_item);

    std::unique_ptr<menu::flow::MenuSet> set_;
    std::unique_ptr<menu::flow::MenuController> ctl_;
    std::unique_ptr<menu::MenuRenderer> renderer_;
    std::vector<ResultRow> rows_;
    std::map<std::string, std::string> texts_;
    std::map<std::string, std::string> class_texts_;
    std::string set_name_;
    std::optional<menu::MenuScene> inset_;
    std::unique_ptr<menu::MenuRenderer> inset_renderer_;
    u16 ignore_ = 0xFFFF; // keys held since the menu opened
    double acc_ = 0;
    std::vector<std::string> sounds_;
};

// The result a race reports to the menus (GT mode records place, prize, trophy mark, best
// time from it). The original records it when the live race ends, before the replay
// (the place and prize are taken right after the race, then the replay runs; spec menus.md §11), so
// we latch it there: replays re-run the field and must not change it, and every way out
// (quick-menu EXIT, replay pause EXIT, leaving from a replay) reports the latched run. A new
// live run (Retry) drops it.
class RaceResultLatch {
public:
    // The live run ended: keep its result (the first capture wins).
    void capture(menu::flow::GameResult r) {
        if (!latched_) latched_ = std::move(r);
    }
    void reset() { latched_.reset(); }
    bool has() const { return latched_.has_value(); }
    // What to post when leaving: the latched run if any, else `current` (an unfinished live
    // run). Flags decided after the run (retiring from a championship, the prize page /
    // records already shown, the car's dirt) come from `current`.
    menu::flow::GameResult for_leave(menu::flow::GameResult current) const {
        if (!latched_) return current;
        menu::flow::GameResult r = *latched_;
        r.abandoned = current.abandoned;
        r.records_done = current.records_done;
        r.prize_shown = current.prize_shown;
        if (current.car_dirt >= 0) r.car_dirt = current.car_dirt;
        return r;
    }

private:
    std::optional<menu::flow::GameResult> latched_;
};

} // namespace gt3r::race
