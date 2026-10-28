// The GT-mode save library (game/save_library.h) and the save page's operations on it
// (game/gt_mode_menu.cpp through GtModeHooks::save_run). Works in the run's temporary folder only.

#include "test_framework.h"

#include "base/file.h"
#include "disc/volume.h"
#include "game/gt_mode_menu.h"
#include "game/gt_save.h"
#include "game/save_library.h"
#include "menu/flow/gt_mode_hooks.h"

#include <algorithm>
#include <filesystem>
#include <cstdlib>

using namespace gt3r;
using namespace gt3r::game;
namespace fs = std::filesystem;

namespace {

GameState make_game(s64 credits, const std::string& comment = {}) {
    GameState s;
    s.credits = credits;
    s.day = 3;
    s.file_comment = comment;
    return s;
}

void set_env(const char* name, const char* value) {
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

void touch_time(const std::string& path, int seconds_ago) {
    fs::last_write_time(path, fs::file_time_type::clock::now() - std::chrono::seconds(seconds_ago));
}

std::string read_text(const std::string& path) {
    std::optional<Bytes> b = read_host_file(path);
    return b ? std::string(b->begin(), b->end()) : std::string("<missing>");
}

bool write_text(const std::string& path, const std::string& text) {
    return write_host_file(path, ByteSpan(reinterpret_cast<const u8*>(text.data()), text.size()));
}

// The texts of the files set_aside_save made for `path` (beside it, "<name>.unreadable...").
std::vector<std::string> kept_texts(const std::string& path) {
    std::vector<std::string> out;
    const fs::path p(path);
    const std::string prefix = p.filename().string() + ".unreadable";
    for (const fs::directory_entry& e : fs::directory_iterator(p.parent_path()))
        if (e.path().filename().string().rfind(prefix, 0) == 0) out.push_back(read_text(e.path().string()));
    std::sort(out.begin(), out.end());
    return out;
}

} // namespace

// The run's own folder (test_main): the system temp folder and the user data folder of the
// code under test (the saves' library, the earlier versions' save) are in it, never the
// player's, so test runs can go at once.
TEST(save_library_test_run_folders) {
    const fs::path root = test::temp_root();
    std::error_code ec;
    CHECK(fs::is_directory(root) && root.filename().string().rfind("gt3r-tests-", 0) == 0);
    CHECK(fs::equivalent(fs::temp_directory_path(ec), root, ec));
    const fs::path data = user_data_dir();
    CHECK(fs::equivalent(data, root / "user-data", ec));
    for (const std::string& p :
         {SaveLibrary::user_dir(Region::America), SaveLibrary::user_dir(Region::Europe), SaveLibrary::legacy_path()})
        CHECK(p.rfind(data.string(), 0) == 0);
}

// Earlier versions' gtmode.sav and gtmode_2.sav are copied into a new library (the
// originals stay); the first one becomes current; nothing happens once the folder exists.
TEST(save_library_migration) {
    const fs::path root = test::temp_path("lib_migrate");
    fs::create_directories(root);
    const std::string legacy = (root / "gtmode.sav").string();
    CHECK(write_save(legacy, make_game(1000)));
    CHECK(write_save((root / "gtmode_2.sav").string(), make_game(2000, "SLOT2")));
    SaveLibrary lib((root / "saves").string());
    CHECK(lib.migrate(legacy) == 2);
    CHECK(fs::exists(legacy) && fs::exists(root / "gtmode_2.sav"));
    std::vector<SaveInfo> l = lib.list();
    CHECK(l.size() == 2);
    CHECK(lib.current() == lib.dir() + "gtmode.sav");
    std::optional<GameState> s = read_save(lib.current());
    CHECK(s && s->credits == 1000);
    bool second = false;
    for (const SaveInfo& f : l)
        if (f.name == "SLOT2" && f.state && f.state->credits == 2000) second = true;
    CHECK(second);
    CHECK(lib.migrate(legacy) == 0); // the folder exists: done once
    // No legacy save: an empty library.
    SaveLibrary empty((root / "other").string());
    CHECK(empty.migrate((root / "missing.sav").string()) == 0 && empty.list().empty() && empty.current().empty());
    fs::remove_all(root);
}

// A PAL disc's library the first time: versions up to 0.3 kept every disc's games in saves/,
// so saves/eu/ starts with copies of its *.sav files (not of its subfolders), the current one
// staying current; the originals stay, and it happens once.
TEST(save_library_pal_copies_earlier_saves) {
    const fs::path root = test::temp_path("lib_pal_copy");
    const fs::path us = root / "saves";
    fs::create_directories(us / "jp");
    CHECK(write_save((us / "Mine.sav").string(), make_game(1000, "MINE")));
    CHECK(write_save((us / "Other.sav").string(), make_game(2000)));
    CHECK(write_text((us / "Broken.sav").string(), "not a save")); // copied as it is (listed as unreadable)
    CHECK(write_text((us / "notes.txt").string(), "x"));
    CHECK(write_save((us / "jp" / "Japan.sav").string(), make_game(3000)));
    SaveLibrary(us.string()).set_current((us / "Mine.sav").string());

    SaveLibrary eu((us / "eu").string());
    CHECK(eu.copy_library(us.string()) == 3);
    CHECK(fs::exists(us / "eu" / "Mine.sav") && fs::exists(us / "eu" / "Other.sav"));
    CHECK(fs::exists(us / "eu" / "Broken.sav"));
    CHECK(!fs::exists(us / "eu" / "notes.txt") && !fs::exists(us / "eu" / "Japan.sav"));
    CHECK(read_text((us / "eu" / "Mine.sav").string()) == read_text((us / "Mine.sav").string()));
    CHECK(fs::exists(us / "Mine.sav") && fs::exists(us / "Other.sav")); // the originals stay
    CHECK(eu.current() == eu.dir() + "Mine.sav");
    std::optional<GameState> s = read_save(eu.current());
    CHECK(s && s->credits == 1000);
    CHECK(eu.list().size() == 3);
    // Once: the folder exists now (also after its saves are gone).
    CHECK(write_save((us / "Later.sav").string(), make_game(4000)));
    CHECK(eu.copy_library(us.string()) == 0 && !fs::exists(us / "eu" / "Later.sav"));
    // Nothing to copy (no earlier library): an empty library, made once.
    SaveLibrary fresh((root / "none" / "eu").string());
    CHECK(fresh.copy_library((root / "none").string()) == 0 && fresh.list().empty() && fresh.current().empty());
    CHECK(fs::is_directory(root / "none" / "eu"));
    fs::remove_all(root);
}

// Listing (newest first, names, unreadable files), unique names and file names, create,
// rename (the current save stays current), copy, delete.
TEST(save_library_operations) {
    const fs::path root = test::temp_path("lib_ops");
    SaveLibrary lib(root.string());
    std::string a, b;
    CHECK(lib.create(make_game(100), "FIRST", &a));
    CHECK(lib.create(make_game(200), "A/B:C?", &b)); // not a file name
    CHECK(fs::path(b).filename() == "A_B_C_.sav");
    touch_time(a, 100);
    CHECK(write_host_file(lib.dir() + "junk.sav", ByteSpan(reinterpret_cast<const u8*>("x"), 1)));
    touch_time(lib.dir() + "junk.sav", 200);
    std::vector<SaveInfo> l = lib.list();
    CHECK(l.size() == 3 && l[0].name == "A/B:C?" && l[1].name == "FIRST" && l[2].name == "junk" && !l[2].state);
    // A second "FIRST": the name and the file are made unique.
    CHECK(lib.unique_name("FIRST") == "FIRST 2" && lib.unique_name("NEW") == "NEW");
    CHECK(lib.unique_name("SIXTEEN CHARS 16") == "SIXTEEN CHARS 16");
    CHECK(lib.path_for("FIRST") == lib.dir() + "FIRST 2.sav" && lib.path_for("FIRST", a) == a);
    CHECK(SaveLibrary::file_stem(" .. ") == "save" && SaveLibrary::file_stem("Ok name.") == "Ok name");
    // Current save tracking.
    CHECK(lib.current().empty());
    CHECK(lib.set_current(a) && lib.current() == a);
    SaveLibrary again(root.string());
    CHECK(again.current() == a); // remembered (current.txt)
    // Rename the current save: new file, same game, still current; the old file is gone.
    std::string a2;
    CHECK(lib.rename(a, "RENAMED", &a2));
    CHECK(a2 == lib.dir() + "RENAMED.sav" && !fs::exists(a) && lib.current() == a2);
    std::optional<GameState> s = read_save(a2);
    CHECK(s && s->credits == 100 && s->file_comment == "RENAMED");
    // Rename with the live game's state.
    GameState live = make_game(150);
    std::string a3;
    CHECK(lib.rename(a2, "RENAMED", &a3, &live) && a3 == a2 && read_save(a3)->credits == 150);
    // Copy: "<name> 2", a new file; the original stays current.
    std::string c;
    CHECK(lib.copy(a3, &c) && c != a3);
    s = read_save(c);
    CHECK(s && s->file_comment == "RENAMED 2" && s->credits == 150 && lib.current() == a3);
    // Delete.
    CHECK(lib.remove(c) && !fs::exists(c) && lib.list().size() == 3);
    // A tracked = false library (GT3R_GTMODE_SAVE) remembers nothing.
    SaveLibrary untracked(root.string(), false);
    CHECK(untracked.current().empty() && untracked.set_current(b) && lib.current() == a3);
    // No path: refused, nothing written (not even "<path>.tmp", which would be ".tmp" in the
    // working directory).
    std::string err;
    CHECK(!write_save("", make_game(1), &err) && err == "no save file");
    fs::remove_all(root);
}

// A save that exists but cannot be read (damaged, from a newer version) is never written
// over: the session moves it aside before its new game autosaves to the path, every failure
// keeps its own copy, and the library does not list the copies. No disc: a session's new
// game needs no game data (the attach hook, which loads it, is save_library_disc_unreadable_attach).
TEST(save_library_unreadable_kept) {
    const fs::path root = test::temp_path("lib_unreadable");
    SaveLibrary lib(root.string());
    std::string path;
    CHECK(lib.create(make_game(500, "CAREER"), "CAREER", &path));
    const std::string career = read_text(path);
    const std::string damaged = career.substr(0, career.size() / 2); // no "end"
    const std::string newer = "GT3R-GTMODE 99\ncredits 1\nend\n";
    set_env("GT3R_GTMODE_NOSAVE", "0");
    const auto data = std::make_shared<const GtData>();
    for (const std::string& text : {damaged, newer}) {
        CHECK(write_text(path, text));
        GtModeSession s(data, path);
        std::string err;
        CHECK(!s.load(&err) && !err.empty());
        CHECK(!fs::exists(path)); // moved aside before anything can be written
        CHECK(s.save());          // what autosave() does
        std::optional<GameState> now = read_save(path);
        CHECK(now && now->garage.empty() && now->day == 1);
    }
    std::vector<std::string> kept = kept_texts(path);
    std::vector<std::string> want = {damaged, newer};
    std::sort(want.begin(), want.end());
    CHECK(kept == want); // both, byte for byte (the second one in the same second too)
    CHECK(lib.list().size() == 1);
    // No save at a path: a new game, nothing set aside.
    const std::string fresh = lib.dir() + "fresh.sav";
    {
        GtModeSession s(data, fresh);
        CHECK(!s.load() && kept_texts(fresh).empty());
    }
    // GT3R_GTMODE_NOSAVE writes nothing: the file stays where it is.
    set_env("GT3R_GTMODE_NOSAVE", "1");
    CHECK(write_text(path, damaged));
    {
        GtModeSession s(data, path);
        CHECK(!s.load() && read_text(path) == damaged && kept_texts(path).size() == 2);
    }
    fs::remove_all(root);
}

// The same through the GT-mode attach hook (with the disc data) and a menu action's autosave.
TEST(save_library_disc_unreadable_attach) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    const fs::path root = test::temp_path("lib_unreadable_attach");
    fs::create_directories(root);
    const std::string path = (root / "gtmode.sav").string();
    const std::string damaged = "GT3R-GTMODE 1\ncredits 900000\ncar 1 r34_gtr_v_99 0 - -\n settings 1 2 x\nend\n";
    CHECK(write_text(path, damaged));
    set_env("GT3R_GTMODE_SAVE", path.c_str());
    set_env("GT3R_GTMODE_NOSAVE", "0");
    const Volume* vol = &disc->volume();
    install_gt_mode_menu();
    const menu::flow::GtModeHooks& h = menu::flow::gt_mode_hooks();
    h.attach([vol](const std::string& p) -> std::optional<Bytes> {
        const Volume::Entry* e = vol->find(p);
        if (!e) return std::nullopt;
        return vol->read(*e);
    }, "US");
    GtModeSession* s = gt_mode_session();
    CHECK(s && s->save_path() == path && s->gt().state().credits == 18000); // a new game
    CHECK(!fs::exists(path) && kept_texts(path) == std::vector<std::string>{damaged});
    // Selling with no car chosen sells nothing and autosaves.
    menu::flow::MenuEvent ev;
    ev.kind = menu::flow::MenuEvent::Kind::Action;
    ev.action = "sell_car";
    std::vector<std::string> log;
    h.on_event(ev, log);
    std::optional<GameState> now = read_save(path);
    CHECK(now && now->credits == 18000 && kept_texts(path) == std::vector<std::string>{damaged});
    set_env("GT3R_GTMODE_NOSAVE", "1");
    fs::remove_all(root);
}

// The save page's operations through the GT-mode hooks with the disc: new save, load,
// rename and copy the current one, import a PS2 save (a .psu without extension, a card
// image), the guest garage from a PS2 card, the export (by extension) and a snapshot save.
TEST(save_library_disc_page_ops) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    using SavePage = menu::flow::SavePage;
    const fs::path root = test::temp_path("lib_page");
    fs::create_directories(root);
    const std::string first = (root / "gtmode.sav").string();
    set_env("GT3R_GTMODE_SAVE", first.c_str());
    set_env("GT3R_GTMODE_NOSAVE", "0");
    const Volume* vol = &disc->volume();
    auto read = [vol](const std::string& p) -> std::optional<Bytes> {
        const Volume::Entry* e = vol->find(p);
        if (!e) return std::nullopt;
        return vol->read(*e);
    };
    install_gt_mode_menu();
    const menu::flow::GtModeHooks& h = menu::flow::gt_mode_hooks();
    h.attach(read, "US"); // a fresh session on GT3R_GTMODE_SAVE
    GtModeSession* s = gt_mode_session();
    CHECK(s && s->save_path() == first);
    if (!s) return;
    auto run = [&](SavePage::Mode m, SavePage::Op op, const SavePage::File& f, const std::string& arg) {
        return h.save_run(m, op, f, arg);
    };
    s->gt().state().credits = 12345;
    CHECK(s->save());
    // Save as new: the game goes on in the new file.
    SavePage::File none;
    none.kind = SavePage::File::Kind::New;
    CHECK(run(SavePage::Mode::SaveGame, SavePage::OpSave, none, "SECOND") == 0);
    const std::string second = (root / "SECOND.sav").string();
    CHECK(fs::path(s->save_path()) == fs::path(second) && read_save(second)->credits == 12345);
    // The list: both, the current one marked, "New Save" first.
    std::vector<SavePage::File> l = h.save_list(SavePage::Mode::LoadGame);
    CHECK(l.size() == 3 && l[0].kind == SavePage::File::Kind::New);
    int current = 0;
    for (const SavePage::File& f : l) current += f.current && fs::path(f.id) == fs::path(second);
    CHECK(current == 1);
    // A PS2 save (an export of this game with 3 cars more credits) as a .psu without an
    // extension and as a card image.
    GameState ps2 = s->gt().state();
    ps2.credits = 54321;
    ps2.file_comment.clear();
    const std::string psu = (root / "from_ps2").string(), card = (root / "card.ps2").string();
    CHECK(write_host_file(psu, export_ps2_save_container(ps2, s->gt().data_ptr(), read, Ps2Export::Psu)));
    CHECK(write_host_file(card, export_ps2_save_container(ps2, s->gt().data_ptr(), read, Ps2Export::Card)));
    SavePage::File open;
    open.kind = SavePage::File::Kind::Open;
    CHECK(run(SavePage::Mode::Files, SavePage::OpImport, open, psu) == 0);
    bool imported = false;
    for (const SavePage::File& f : h.save_list(SavePage::Mode::Files))
        if (f.name == "from_ps2" && !f.current) {
            imported = read_save(f.id)->credits == 54321;
            // Load it: the game switches over.
            CHECK(run(SavePage::Mode::LoadGame, SavePage::OpLoad, f, {}) == 0);
            CHECK(s->save_path() == f.id && s->gt().state().credits == 54321);
        }
    CHECK(imported);
    // The guest garage from a card image (not added to the library).
    const size_t n = h.save_list(SavePage::Mode::Files).size();
    CHECK(run(SavePage::Mode::LoadGuest, SavePage::OpLoad, open, card) == 0);
    CHECK(s->guest && s->guest->state().credits == 54321 && h.save_list(SavePage::Mode::Files).size() == n);
    // A native save under any name, picked from elsewhere.
    GameState other = s->gt().state();
    other.credits = 777;
    CHECK(write_save((root / "friend.dat").string(), other));
    CHECK(run(SavePage::Mode::LoadGuest, SavePage::OpLoad, open, (root / "friend.dat").string()) == 0 &&
          s->guest->state().credits == 777);
    // A file that is no save: the read error.
    CHECK(run(SavePage::Mode::LoadGuest, SavePage::OpLoad, open, first + ".nothing") != 0);
    // Rename the current save: the game is written to the new file from now on.
    SavePage::File cur;
    for (const SavePage::File& f : h.save_list(SavePage::Mode::Files))
        if (f.current) cur = f;
    CHECK(run(SavePage::Mode::Files, SavePage::OpRename, cur, "MY RUN") == 0);
    CHECK(fs::path(s->save_path()) == root / "MY RUN.sav" && s->gt().state().file_comment == "MY RUN");
    CHECK(!fs::exists(cur.id));
    // The current save cannot be deleted; a copy can.
    for (const SavePage::File& f : h.save_list(SavePage::Mode::Files))
        if (f.current) cur = f;
    CHECK(run(SavePage::Mode::Files, SavePage::OpDelete, cur, {}) != 0);
    CHECK(run(SavePage::Mode::Files, SavePage::OpCopy, cur, {}) == 0);
    SavePage::File copy;
    for (const SavePage::File& f : h.save_list(SavePage::Mode::Files))
        if (f.name == "MY RUN 2") copy = f;
    CHECK(!copy.id.empty() && run(SavePage::Mode::Files, SavePage::OpDelete, copy, {}) == 0 && !fs::exists(copy.id));
    // Export in the format chosen on the page, whatever the file is called (a name without
    // an extension gets the format's); the exports import again, by their contents.
    const struct {
        SavePage::Op op;
        const char* out;
        const char* written;
    } exports[] = {{SavePage::OpExportPsu, "x.psu", "x.psu"},       {SavePage::OpExportCard, "x.ps2", "x.ps2"},
                   {SavePage::OpExportRaw, "x.raw", "x.raw"},       {SavePage::OpExportPsu, "plain", "plain.psu"},
                   {SavePage::OpExportRaw, "bare", "bare.raw"},     {SavePage::OpExportCard, "mc", "mc.ps2"},
                   {SavePage::OpExportRaw, "named.psu", "named.psu"}};
    for (const auto& e : exports) {
        CHECK(run(SavePage::Mode::Files, e.op, cur, (root / e.out).string()) == 0);
        CHECK(read_any_save((root / e.written).string(), s->gt().data()).credits == 54321);
    }
    const Bytes raw = export_ps2_save(s->gt().state(), s->gt().data());
    for (const char* name : {"x.raw", "bare.raw", "named.psu"}) CHECK(read_host_file((root / name).string()) == raw);
    CHECK(read_host_file((root / "mc.ps2").string())->size() == 0x840000);
    CHECK(run(SavePage::Mode::Files, SavePage::OpExport, cur, (root / "y.psu").string()) != 0 && !fs::exists(root / "y.psu"));
    // A raw save (from a save editor) imports under its file name.
    GameState edited = s->gt().state();
    edited.credits = 4242;
    edited.file_comment.clear();
    CHECK(write_host_file((root / "edited.raw").string(), export_ps2_save(edited, s->gt().data())));
    CHECK(run(SavePage::Mode::Files, SavePage::OpImport, open, (root / "edited.raw").string()) == 0);
    bool raw_imported = false;
    for (const SavePage::File& f : h.save_list(SavePage::Mode::Files))
        raw_imported |= f.name == "edited" && read_save(f.id)->credits == 4242;
    CHECK(raw_imported);
    // A snapshot: saved as a new file, the running game and its save untouched.
    GameState snap = s->gt().state();
    snap.credits = 99;
    const std::string playing = s->save_path();
    begin_snapshot_save(snap);
    CHECK(run(SavePage::Mode::SaveGame, SavePage::OpSave, none, "BETWEEN") == 0);
    CHECK(s->save_path() == playing && s->gt().state().credits == 54321);
    CHECK(read_save((root / "BETWEEN.sav").string())->credits == 99);
    // A new game.
    CHECK(run(SavePage::Mode::LoadGame, SavePage::OpNewGame, none, "FRESH") == 0);
    CHECK(fs::path(s->save_path()) == root / "FRESH.sav" && s->gt().state().garage.empty() &&
          s->gt().state().day == 1 && fs::exists(root / "FRESH.sav"));
    set_env("GT3R_GTMODE_NOSAVE", "1");
    fs::remove_all(root);
}
