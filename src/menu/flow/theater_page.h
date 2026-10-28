#pragma once

// The Replay Theater's file list, drawn on the original's DEMONSTRATION page
// (menu/memorycard.mbl page "demo_loader", memorycard1.pmb): one wide panel of 64 x 64
// file icons, 8 to a row, rows scrolling, and the focused file's info panel below
// ("info_replay" / "info_best_replay": Size, Date, Mode, File Name, Car, Power, Course,
// Record, Laps). The original page lists the disc's demonstration runs; Load Replay and
// the Analyzer's LOAD A / LOAD B go through the memory-card file pages there. Ours list
// the replay files in native storage on this same page instead: no slots, no capacity,
// no confirm or "working" windows. Confirm plays (or loads) the focused file at once;
// Start deletes a replay after one "Delete OK?" (the file page's confirm art); cancel
// (Triangle / Square) goes back.
//
// SDL-free and frame based like MenuController.

#include "menu/flow/menu_controller.h"
#include "menu/flow/menu_set.h"
#include "menu/menu_renderer.h"

#include <functional>
#include <string>
#include <vector>

namespace gt3r::menu::flow {

// One file of the list (filled by the app: replay files or the disc's demonstrations).
struct TheaterEntry {
    enum class Icon { Replay, License, Best };
    Icon icon = Icon::Replay;
    std::string id;   // replay file path, or the demonstration's event code
    bool deletable = false;
    bool playable = true; // false: listed only (a replay of an older version: it can be deleted)
    // Info panel texts by field: "course", "car", "time", "laps", "power", "mode", "date",
    // "size", "comment" (the widgets r_pt_<field> / b_pt_<field>).
    std::vector<std::pair<std::string, std::string>> texts;
    std::string text(const std::string& field) const {
        for (const auto& [k, v] : texts)
            if (k == field) return v;
        return {};
    }
};

// What the app provides (src/game/replay_theater.cpp).
struct TheaterBackend {
    // kind 0: saved replays (newest first), 1: the disc's demonstrations.
    std::function<std::vector<TheaterEntry>(int kind)> list;
    std::function<bool(const TheaterEntry&)> remove;
};
void set_theater_backend(TheaterBackend b);
const TheaterBackend& theater_backend();

class TheaterPage {
public:
    enum class Mode {
        LoadReplay,    // Replay Theater > Load Replay: title "Load replay"
        Demonstration, // Replay Theater > Demonstration: the disc's demonstrations
        LoadTrack,     // Analyzer LOAD A / LOAD B: title "Load track"
    };
    static constexpr int kCols = 8, kRows = 3;

    void open(Mode m, std::vector<TheaterEntry> entries, int cursor = 0);
    // One frame. 0 running, 1 the focused entry was chosen (chosen()), -1 left.
    int tick(MenuKeys held, std::vector<std::string>& sounds);
    // Code-built parts over the page's own layers: titles, icons, cursor, info panel and the
    // delete confirm (`confirm`: the file_operation page of the same set, for its art).
    void draw(const MenuSet::PageRef& ref, const MenuSet::PageRef& confirm, std::vector<ElementDraw>& out) const;

    Mode mode() const { return mode_; }
    int cursor() const { return cursor_; }
    int top_row() const { return top_; }
    bool confirming() const { return confirm_; }
    const std::vector<TheaterEntry>& entries() const { return entries_; }
    const TheaterEntry* chosen() const {
        return cursor_ >= 0 && cursor_ < int(entries_.size()) ? &entries_[size_t(cursor_)] : nullptr;
    }

private:
    Mode mode_ = Mode::LoadReplay;
    std::vector<TheaterEntry> entries_;
    int cursor_ = 0, top_ = 0;
    bool confirm_ = false;
    int choice_ = 0; // confirm: 0 OK, 1 CANCEL
    int frames_ = 0;
    MenuKeys prev_ = 0;
    int repeat_ = 0;
};

} // namespace gt3r::menu::flow
