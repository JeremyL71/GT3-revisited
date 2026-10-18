#pragma once

// The top-level menu flow of GT3 (US): what the original's main loop and
// per-mode menu code (GT mode, arcade, options) do around
// the page manager. Starts at the title menu (gt_mode.mbl page "root"),
// enters arcade / GT mode / options, walks the arcade set-up screens and
// raises a GameRequest where the original starts a race. Code-drawn pages
// (course / class / car selection) are reproduced as simple carousels over
// the pages' own "hidden" art.
//
// SDL-free and frame based like MenuController.

#include "menu/flow/game_hooks.h"
#include "menu/flow/menu_controller.h"
#include "menu/flow/options.h"
#include "menu/flow/save_page.h"
#include "menu/flow/screen_transition.h"
#include "menu/flow/theater_page.h"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gt3r::menu::flow {

class GameFlow {
public:
    GameFlow(FileReader reader, std::string lang);

    // Boot: the title menu, focus on the first item.
    void start();
    // Starts directly in another control (e.g. "setting" / "root": the car
    // settings menu opened from a race's quick menu). exited() turns true when
    // that control is left from its top page.
    void start(const std::string& set, const std::string& page);
    // Starts on the save page alone in Save game mode (a race's quick menu "Save game",
    // with game::begin_race_game_save's snapshot); exited() turns true when the page ends.
    void start_save_game();
    bool exited() const { return exited_; }
    void tick(MenuKeys held);
    // Touch: a tap at a point in menu layout coordinates (640x448, x right, y up
    // from the bottom-left). Tapping an item focuses it, tapping the focused
    // item confirms it; on code-drawn pickers (arcade course/car carousels) the
    // left / right thirds step the entry and the middle confirms. Returns true
    // when the tap was used (the caller should then not treat it as a key).
    bool tap(float x, float y);

    std::vector<ElementDraw> draw_list() const;
    // Text width in menu units for a pmb font spec ("sw_20") at its own size: places the
    // name entry's caret after the text. Without it, half the font size per character.
    std::function<float(const std::string& font, const std::string& text)> measure_text;
    std::vector<std::string> take_sounds();
    std::optional<GameRequest> take_request();
    // Menu background music change: a music.inf sequence name, or "" to stop.
    std::optional<std::string> take_music();
    // Background movie for "mpeg" widgets: a VOL path, or "" to stop.
    std::optional<std::string> take_movie();
    // A full-screen transition the screen should start now over the last
    // frame (the original's loading transition while the next mode loads).
    std::optional<TransitionType> take_transition();
    // True while switching controls: the screen keeps the frozen frame and
    // draws the transition instead of the menu.
    bool switching() const { return switch_frames_ > 0; }
    // Gameplay started by a request has returned: reopen the menu.
    void game_finished();
    bool waiting_for_game() const { return waiting_game_; }
    // Messages for actions this reimplementation does not support yet
    // (memory card, replays, movies...). Unknown actions are logged too.
    std::vector<std::string> take_log();
    // The game options (title Options menu): read from / written to [options] of this
    // document; `changed` runs after every change (persist and apply).
    void set_options(IniDocument* cfg, std::function<void()> changed) {
        options_ = cfg;
        options_changed_ = std::move(changed);
    }

    const std::string& control() const { return control_; }
    // True for the GT-mode control and its prize / license presentations
    // (the GtModeHooks apply).
    // Runs the GT-mode presentations queued so far (name entry, licence card, prize car,
    // trophies) on their own, e.g. from the race screen; exited() once the last one closes.
    // False (and exited) when nothing is queued.
    bool start_presentations();
    bool gt_active() const {
        return control_ == "gt_mode" || control_ == "setting" || control_ == "records" || presenting_;
    }
    // Attract mode (title idle -> demo races -> opening movie) is running.
    bool attract() const { return in_attract_; }
    MenuController* controller() { return ctl_.get(); }
    const MenuController* controller() const { return ctl_.get(); }
    const MenuSet* set() const { return set_; }
    const std::string& lang() const { return lang_; }

    // Code-drawn selection page state (arcade course/class/car pickers).
    // The original's layout: a strip of
    // `<entry>_mini` icons (every available entry, centred or scrolled so the
    // focused one is centred, the focused icon pulsing) and a sliding "cross"
    // (`cross_bar`) that carries the entry's art 680 px sideways on each move.
    struct Carousel {
        std::string page;
        std::vector<std::string> entries;
        int index = 0;
        bool wrap = false;       // class page wraps; course / car / time trial clamp
        float scroll = 0;        // strip scroll (screen x of the first icon's left edge, minus the strip x)
        float pulse = 1;         // focused icon brightness (PS2 units, 0.502 = normal)
        float cross_x = 0, cross_v = 0, cross_target = 0; // cross position (screen x), spring
        int age = 0;             // frames since the current entry's art appeared (24-frame fade-in)
        int rest = 0;            // frames since the last move (background movie after 6)
        int frames = 0;          // frames since the page opened (breadcrumb slide-in)
        // Car page steps: 1 car and paint, 2 AUTOMATIC / MANUAL, 3 RACING / DRIFT. The page
        // springs up (oy, screen px) so the step's button row sits in the cross band.
        int step = 1;
        float oy = 0, oy_v = 0;
        float strip_alpha = 1;
        bool has_cross = false;  // the page has a cross_bar (the 2 player car page has none)
        float paint_pos = 0;     // colour column position (eases towards the paint index)
        float blur = 0;          // background blur radius (px, the car page steps)
        float art_offset = 0;    // the current art's offset from its layout place at cross_target
        struct Old {
            std::string id;
            float offset;        // as art_offset
            float alpha;
            int frames;          // frames left (16)
        };
        std::vector<Old> old;
    };
    const Carousel* carousel() const { return carousel_.entries.empty() ? nullptr : &carousel_; }
    int car_paint() const { return car_paint_; }
    // Player 2's pad (2 Player Battle car page), set before tick().
    void set_player2_keys(MenuKeys held) { held2_ = held; }
    // 2 Player car page (ar/select_car_2p): each player's selection, run on its own pad.
    // States: 2 class row, 4 car + paint, 6 AT / MT,
    // 8 RACING / DRIFT, 9 ready.
    struct TwoPlayer {
        int state = 2;
        int cls = 0;                      // index into `classes`
        std::vector<std::string> cars;    // the chosen class's open cars
        int car = 0, paint = 0, transmission = 0, tire_mode = 0;
        int age = 0;                      // frames since the current car / class appeared
        float scroll = 0, pulse = 1;      // the row's MiniCursor
        float paint_pos = 0;
        MenuKeys prev = 0;                // key repeat
        int repeat = 0;
    };
    const TwoPlayer* two_player(int p) const { return two_page_ ? &two_[p] : nullptr; }
    static const std::vector<std::string>& two_player_classes();

    // Debugging / screenshots: opens a page of a menu set directly (GT3R_GT3MENU_OPEN).
    void debug_open(const std::string& set, const std::string& page) { open_control(set, page, -1); }

private:
    std::vector<ElementDraw> draw_list_impl() const;
    // Vertically flipped copies of elements (the name entry's TextBox draws its box and
    // caret gradients upside down relative to the pmb corner order).
    const Element* flipped(const Element* e) const;
    mutable std::map<const Element*, std::unique_ptr<Element>> flipped_;
    MenuSet& load_set(const std::string& name);
    void switch_control(const std::string& set, const std::string& page, int focus);
    void open_control(const std::string& set, const std::string& page, int focus);
    void handle(const MenuEvent& e);
    void handle_gt(const MenuEvent& e);
    void handle_arcade(const MenuEvent& e);
    void handle_option(const MenuEvent& e);
    void arcade_step(int step);
    void arcade_back();
    void arcade_start_race();
    void build_carousel(const std::string& page);
    void animate_carousel(int move);        // one frame; move = -1 / 0 / +1 (an accepted move)
    std::vector<std::string> carousel_parts(const std::string& id) const; // the entry's art (moves with the cross)
    // The hidden picture of a car page entry (GtModeHooks::arcade_image; a label that
    // starts with a digit is named "_<label>" in the layout), else the id itself.
    std::string entry_image(const std::string& id) const;
    std::string carousel_disabled() const; // why the shown carousel entry cannot be chosen, or ""
    void request_race(GameRequest r);
    // The save page (control "memorycard", page file_operation; save_page.h): when it
    // ends the flow switches to `ok` after a load / save, else to `back`.
    struct Target {
        std::string set, page;
        int focus = -1;
    };
    void open_save_page(SavePage::Mode m, Target back, Target ok, bool transition, std::string title = {});
    void switch_control_now(const Target& t);
    void log(std::string msg);

    FileReader reader_;
    std::string lang_;
    std::map<std::string, std::unique_ptr<MenuSet>> sets_;
    std::string control_;
    MenuSet* set_ = nullptr;
    std::unique_ptr<MenuController> ctl_;
    std::vector<std::string> sounds_;
    std::vector<std::string> log_;
    std::optional<GameRequest> request_;
    std::optional<TransitionType> transition_;
    std::string music_ = "-"; // current menu BGM ("-" = none chosen yet)
    std::optional<std::string> music_request_;
    std::string movie_;
    std::optional<std::string> movie_request_;

    // Pending control switch (runs after the transition).
    int switch_frames_ = 0;
    std::string next_set_, next_page_;
    int next_focus_ = -1;
    std::optional<GameRequest> pending_request_;
    bool waiting_game_ = false;
    std::string resume_set_, resume_page_;
    int resume_focus_ = -1;

    // Title / GT mode.
    int title_focus_ = 0;
    std::string race_code_;
    // Arcade.
    int arcade_game_ = 0;           // index on ar/top
    std::string arcade_action_;     // its action
    std::vector<std::string> arcade_steps_;
    int arcade_pos_ = -1;
    int difficulty_ = 0;
    std::string course_, car_class_, car_;
    int car_paint_ = 0; // arcade car pickers: colour index of the focused car (Up / Down)
    int transmission_ = 0, tire_mode_ = 0; // car page steps 2 / 3 (kept while the arcade runs)
    Carousel carousel_;
    IniDocument* options_ = nullptr;
    mutable std::vector<std::pair<std::string, std::string>> songs_; // Options music list (title, artist)
    mutable bool songs_loaded_ = false;
    void option_draws(std::vector<ElementDraw>& out) const;
    std::function<void()> options_changed_;
    SavePage save_page_;
    bool save_mode_ = false; // the "memorycard" control shows the save page
    bool name_mode_ = false; // the save page's name entry (Edit file name, a new save)
    std::string name_text_;
    MenuKeys held_name_ = 0;
    Target save_back_, save_ok_;
public:
    const SavePage* save_page() const {
        return control_ == "memorycard" && save_mode_ && !theater_mode_ ? &save_page_ : nullptr;
    }
    // The save page on its own (title Save game): saves the running game, or the state the
    // game side offered with game::begin_snapshot_save() (e.g. a championship between
    // rounds, without changing the running game), as a new save or over one (one
    // confirm), then reopens <set>/<page> (focus `focus`) either way.
    void open_save_game(const std::string& set, const std::string& page, int focus = -1) {
        open_save_page(SavePage::Mode::SaveGame, Target{set, page, focus}, Target{set, page, focus}, false);
    }
    // Replay Theater: the file list (Load Replay, Demonstration, the Analyzer's LOAD A / B).
    const TheaterPage* theater() const { return control_ == "memorycard" && theater_mode_ ? &theater_ : nullptr; }
    // The Analyzer (menu/logger.mbl page "top"): the loaded tracks and the focused button
    // (0 REPLAY, 1 GRAPH, 2 LOAD A, 3 LOAD B, 4 EXIT).
    struct LoggerTrack {
        std::string path; // "" = none loaded
        std::vector<std::pair<std::string, std::string>> texts; // TheaterEntry texts
    };
    const LoggerTrack& logger_track(int i) const { return logger_tracks_[i & 1]; }
    int logger_focus() const { return logger_focus_; }
private:
    // Replay Theater (docs/dev/RACE.md "Replay Theater").
    enum class TheaterFrom { Theater, LoggerA, LoggerB };
    void open_theater(TheaterPage::Mode m, TheaterFrom from);
    void theater_tick(MenuKeys held);
    void logger_tick(MenuKeys held);
    void logger_draws(std::vector<ElementDraw>& out) const;
    TheaterPage theater_;
    bool theater_mode_ = false;
    TheaterFrom theater_from_ = TheaterFrom::Theater;
    bool theater_resume_ = false; // gameplay started by the theater: resume without results
    LoggerTrack logger_tracks_[2];
    int logger_focus_ = 0;
    int logger_frames_ = 0;
    MenuKeys logger_prev_ = 0;
    u32 generation_ = 0; // bumped whenever a page is opened
    bool gt_attached_ = false; // GtModeHooks::attach ran
    MenuKeys held_ = 0;
    MenuKeys held2_ = 0;
    bool two_page_ = false;           // ar/select_car_2p is open
    TwoPlayer two_[2];
    std::vector<std::string> raw_cars_; // every car picture of the 2 Player car page
    void two_player_open();
    void two_player_frame();
    bool two_player_step(int p, MenuKeys fired);
    void two_player_draw(std::vector<ElementDraw>& out) const;        // keys held this tick (arcade Professional: L1 + R1 on Hard)
    bool presenting_ = false;  // a GT-mode presentation control is open
    bool standalone_shows_ = false; // start_presentations(): exit when the queue is empty
    MenuKeys injected_ = 0;    // keys pressed by a tap, applied on the next tick
    bool exited_ = false;      // a control started with start(set, page) was left
    bool standalone_save_ = false; // started with start_save_game()
    // Attract mode: idle frames on the title, demos run.
    int idle_frames_ = 0;
    bool in_attract_ = false;
    int attract_demos_ = 0;
    bool attract_movie_ = false;
    void start_attract_demo();
    bool show_presentation();  // opens the next one; false when none
    void after_game();         // presentations, then the next round or the menu
};

} // namespace gt3r::menu::flow
