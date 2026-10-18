#pragma once

// The save page: GT-mode saves and car setting files as plain native files, drawn on
// the original's file list page (menu/memorycard.mbl "file_operation": two icon panels,
// the focused file's info panel, the operation menu and the confirm / error windows).
// The original goes through two PS2 memory-card slots (slot choice, "Load game?", a
// "working" window, "Complete"); this page has none of that: the left panel lists the
// files (any number, rows scrolling), the right panel the files from elsewhere
// (the platform's file dialog: other saves, PS2 saves), and only an overwrite or a delete
// asks once. Modes (docs/dev/GT_MODE.md "Saves"):
//   LoadGame     title Load: a save -> Load / Edit file name / Copy / Delete / Export;
//                "New Save" starts a new game; Import a save from a file.
//   SaveGame     title / home Save: "New Save" (name entry, then saved), or a save ->
//                Overwrite ("Save" on the current one) / Edit file name / Copy / Delete /
//                Export.
//   LoadGuest    Trade, arcade Load Garage: a save (or a file from elsewhere) is loaded
//                at once as the guest garage.
//   Files        title File Operation: Edit file name / Copy / Delete / Export; Import.
//   LoadSettings / SaveSettings   the car's setting files in the current game (Load or
//                Overwrite / Edit file name / Copy / Delete; Save on "New Save").
// The save being played is never deleted (no Delete on it).
//
// SDL-free and frame based like MenuController; the file dialog is the app's
// (set_file_dialogs).

#include "menu/flow/menu_controller.h"
#include "menu/flow/menu_set.h"
#include "menu/menu_renderer.h"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace gt3r::menu::flow {

// The platform file dialogs (registered by the app; SDL_ShowOpenFileDialog /
// SDL_ShowSaveFileDialog, the system picker on Android). show() starts one (async); poll()
// returns its answer once: the chosen path, or "" when cancelled; nullopt while it is open.
struct FileDialogs {
    enum class Kind { OpenSave, ExportPs2 };
    std::function<void(Kind kind, const std::string& default_name)> show;
    std::function<std::optional<std::string>()> poll;
};
void set_file_dialogs(FileDialogs d);
const FileDialogs& file_dialogs();

class SavePage {
public:
    enum class Mode { LoadGame, SaveGame, LoadGuest, Files, LoadSettings, SaveSettings };
    struct File {
        enum class Kind { New, Game, Setting, Open };
        Kind kind = Kind::Game;
        std::string id;       // backend id (a save's path, a setting file's index)
        bool own = true;      // settings modes: a setting file of the car (others are dimmed)
        bool current = false; // the save being played
        bool broken = false;  // unreadable (the "Corrupt File" icon)
        std::string name;     // file name (Kind::New: the name a new save starts with)
        // Info panel texts by widget name ("g_pt_money", "s_pt_car", ...).
        std::vector<std::pair<std::string, std::string>> texts;
    };
    enum Op { OpLoad, OpSave, OpOverwrite, OpDelete, OpCopy, OpRename, OpExport, OpImport, OpNewGame };
    // Backend results.
    enum Result { Ok = 0, ReadError = 1, BadData = 2, WriteError = 3 };
    struct Backend {
        // The left panel's files, "New Save" first where the mode has one.
        std::function<std::vector<File>(Mode m)> list;
        // Runs an operation. `arg`: the name (OpRename, OpSave / OpNewGame on "New Save"),
        // the chosen path (OpExport, OpImport, OpLoad on the "Open" entry).
        std::function<int(Mode m, Op op, const File& f, const std::string& arg)> run;
    };
    static constexpr int kNameMax = 16; // file name length (the original's comment)
    static constexpr int kCols = 4, kRows = 3;

    // `title`: the page title ("" = the mode's own).
    void open(Mode m, Backend b, std::string title = {});
    // One frame. 0 running, 1 ended after a load / save / new game, -1 left.
    int tick(MenuKeys held, std::vector<std::string>& sounds);
    // Code-built parts over the page's own layers (panel titles, icons, cursor, info,
    // operation menu, windows). The panel widgets of `out` (drawn by the controller) get
    // their texts and brightness here.
    void draw(const MenuSet::PageRef& ref, std::vector<ElementDraw>& out) const;

    // Edit file name / a new save's name: while true the flow runs the name entry page and
    // then calls name_done (ok = confirmed).
    bool wants_name() const { return phase_ == Phase::NameEntry; }
    const std::string& name_start() const { return name_; }
    void name_done(bool ok, const std::string& name);

    enum class Phase { List, Operation, NameEntry, Dialog, Confirm, Error };
    Phase phase() const { return phase_; }
    Mode mode() const { return mode_; }
    int panel() const { return panel_; }
    int cursor() const { return cursor_; }
    int top_row() const { return top_; }
    const std::vector<File>& files(int panel = 0) const { return panel ? extra_ : files_; }
    const File* focused() const;
    // The focused file's operations, top first (the operation menu).
    std::vector<Op> ops() const;
    int op() const { return op_; }
    int result() const { return result_; }

private:
    void reload(bool to_first);
    void run(Op op, const std::string& arg);
    void move(int dc, int dr, std::vector<std::string>& sounds);
    Mode mode_ = Mode::LoadGame;
    Backend backend_;
    std::string title_;
    std::vector<File> files_, extra_;
    int panel_ = 0;
    int cursor_ = 0; // index in the panel's files
    int top_ = 0;    // left panel: first row shown
    int op_ = 0;
    Op chosen_ = OpLoad;
    int choice_ = 0; // confirm: 0 OK, 1 CANCEL
    std::string name_;
    Phase phase_ = Phase::List;
    int result_ = 0;
    int frames_ = 0;
    int repeat_ = 0;
    MenuKeys prev_ = 0;
};

} // namespace gt3r::menu::flow
