// Music jukebox: the sequenced menu music (music/music.seq, played through
// the SPU2-style sequencer) and the streamed soundtrack (bgm/*.ads), from
// the disc or a mod.

#include "app/app.h"
#include "app/screen_registry.h"
#include "formats/bgm_index.h"
#include "game/data/car_database.h"

#include <algorithm>
#include <map>

namespace gt3r {

namespace {

class JukeboxScreen : public Screen {
public:
    void update(App& app, double) override {
        if (!loaded_) load(app);
        const MenuInput& m = app.menu();
        int n = int(tracks_.size());
        if (n) sel_ = std::clamp(sel_ + m.vertical(), 0, n - 1);
        if (m.horizontal()) app.audio().set_music_volume(app.audio().music_volume() + 0.05f * m.horizontal());
        if (m.pressed(PadButton::Cross) && n) play(app, sel_);
        if (m.pressed(PadButton::Circle)) {
            app.audio().stop_music();
            playing_ = -1;
        }
        if (m.pressed(PadButton::Square)) {
            VoiceMixer& v = app.audio().voices();
            v.set_reverb(v.reverb_mode() == SpuReverb::Off ? SpuReverb::StudioC : SpuReverb::Off, 0x0FFF, 0x0FFF);
        }
        // Auto-advance when a streamed track ends (sequences loop forever).
        if (playing_ >= 0 && !app.audio().music_playing() && n) play(app, (playing_ + 1) % n);
        if (m.pressed(PadButton::Triangle)) {
            app.audio().stop_music();
            app.save_config();
            app.pop();
        }
    }

    void render(App& app, Ui& ui) override {
        char vol[64];
        int rev = app.audio().voices().reverb_mode();
        std::snprintf(vol, sizeof(vol), "Volume %d%%  reverb %s", int(app.audio().music_volume() * 100 + 0.5f),
                      SpuReverb::mode_name(rev));
        ui.header("MUSIC JUKEBOX", vol);
        if (tracks_.empty()) {
            ui.text_center(150, error_.empty() ? "No tracks found" : error_, colors::kBad);
        }
        int n = int(tracks_.size());
        int first = ui.list_window(n, sel_, kRows);
        for (int i = 0; i < kRows && first + i < n; ++i) {
            int idx = first + i;
            const Track& t = tracks_[idx];
            float y = 28 + i * 12.0f;
            if (idx == sel_) ui.fill(4, y - 2, Ui::kWidth - 8, 12, colors::kHighlight);
            Color c = idx == playing_ ? colors::kGood : colors::kText;
            std::string title = t.title;
            if (title.size() > 40) title = title.substr(0, 37) + "...";
            std::string artist = t.artist.size() > 24 ? t.artist.substr(0, 21) + "..." : t.artist;
            ui.text(10, y, idx == playing_ ? ">" : " ", colors::kGood);
            ui.text(22, y, title, c);
            ui.text(352, y, t.sequence >= 0 ? "SEQ" : "ADS", colors::kDim);
            ui.text(408, y, artist, colors::kDim);
        }
        if (playing_ >= 0) {
            double pos = app.audio().music_position(), len = app.audio().music_length();
            char t[64];
            if (tracks_[playing_].sequence >= 0) {
                // Sequences loop: show the time, the loop length and the passes.
                int loops = app.audio().sequencer().loops_completed();
                std::snprintf(t, sizeof(t), "%d:%02d  loop %d:%02d  x%d  %d voices", int(pos) / 60, int(pos) % 60,
                              int(len) / 60, int(len) % 60, loops, app.audio().voices().active_voices());
                ui.text_right(Ui::kWidth - 10, Ui::kHeight - 31, t, colors::kDim);
            } else {
                std::snprintf(t, sizeof(t), "%d:%02d / %d:%02d", int(pos) / 60, int(pos) % 60, int(len) / 60,
                              int(len) % 60);
                ui.bar(10, Ui::kHeight - 30, Ui::kWidth - 120, 6, len > 0 ? float(pos / len) : 0.0f, colors::kAccent);
                ui.text_right(Ui::kWidth - 10, Ui::kHeight - 31, t, colors::kDim);
            }
        }
        ui.footer("Cross: play  Circle: stop  Left/Right: volume  Square: reverb  Tri: back");
    }

private:
    static constexpr int kRows = 24;

    struct Track {
        std::string title, artist;
        std::string file; // streamed: path under bgm/
        int sequence = -1; // sequenced: index into music.seq
    };

    void load(App& app) {
        loaded_ = true;
        // Sequenced music: one entry per sequence, named by the game's ids
        // that use it.
        std::map<u32, Track> seqs;
        for (const sound::MusicEntry& e : app.audio().music_index(*app.vfs())) {
            Track& t = seqs[e.sequence];
            if (t.sequence < 0) {
                t.sequence = int(e.sequence);
                t.title = e.title + " -";
                t.artist = e.artist;
            }
            t.title += " " + e.name;
        }
        for (auto& [i, t] : seqs) tracks_.push_back(std::move(t));

        auto inf = app.vfs()->read("bgm/ads.inf");
        if (!inf) {
            if (tracks_.empty()) error_ = "bgm/ads.inf not found";
            return;
        }
        std::vector<BgmTrack> bgm;
        try {
            bgm = parse_bgm_index(*inf);
        } catch (const std::exception& e) {
            error_ = e.what();
        }
        // Keep only tracks whose stream exists (mods may add or remove).
        for (const BgmTrack& b : bgm) {
            if (!app.vfs()->exists("bgm/" + b.file)) continue;
            tracks_.push_back({b.title.empty() ? b.name : b.title, b.artist, b.file, -1});
        }
    }

    void play(App& app, int idx) {
        const Track& t = tracks_[idx];
        bool ok = t.sequence >= 0 ? app.audio().play_sequence(*app.vfs(), t.sequence)
                                  : app.audio().play_music(*app.vfs(), "bgm/" + t.file, false);
        playing_ = ok ? idx : -1;
    }

    std::vector<Track> tracks_;
    std::string error_;
    bool loaded_ = false;
    int sel_ = 0;
    int playing_ = -1;
};

class ModsScreen : public Screen {
public:
    void update(App& app, double) override {
        Vfs& vfs = *app.vfs();
        const MenuInput& m = app.menu();
        int n = int(vfs.mods().size());
        if (n) sel_ = std::clamp(sel_ + m.vertical(), 0, n - 1);
        if (m.pressed(PadButton::Cross) && n) {
            const ModInfo& mod = vfs.mods()[sel_];
            bool on = !mod.enabled;
            app.config().set("mods", mod.id, on ? "on" : "off");
            vfs.set_mod_enabled(mod.id, on);
            app.save_config();
            cars_loaded_ = false;
        }
        if (m.pressed(PadButton::Square)) {
            vfs.scan_mods(app.mods_dir(), &app.config());
            cars_loaded_ = false;
        }
        if (m.pressed(PadButton::Start)) SDL_OpenURL(("file://" + app.mods_dir()).c_str());
        if (m.pressed(PadButton::Triangle)) app.pop();
    }

    void render(App& app, Ui& ui) override {
        ui.header("MODS", "Higher priority wins");
        const auto& mods = app.vfs()->mods();
        if (mods.empty()) {
            ui.text_center(120, "No mods installed.", colors::kText);
            ui.text_center(140, "Put mods in:", colors::kDim);
            std::string dir = app.mods_dir();
            if (dir.size() > 76) dir = "..." + dir.substr(dir.size() - 73);
            ui.text_center(154, dir, colors::kAccent);
            ui.text_center(176, "Each mod: mod.ini + files/<path inside GT3.VOL>", colors::kDim);
            ui.text_center(188, "See docs/MODDING.md", colors::kDim);
        }
        for (size_t i = 0; i < mods.size(); ++i) {
            const ModInfo& mod = mods[i];
            float y = 30 + i * 26.0f;
            if (int(i) == sel_) ui.fill(4, y - 3, Ui::kWidth - 8, 24, colors::kHighlight);
            ui.text(10, y, mod.enabled ? "[x]" : "[ ]", mod.enabled ? colors::kGood : colors::kDim);
            ui.text(44, y, mod.name + (mod.version.empty() ? "" : "  v" + mod.version), colors::kText);
            std::string counts = std::to_string(mod.files.size()) + " files";
            if (!mod.cars.empty()) counts += "  " + std::to_string(mod.cars.size()) + " cars";
            ui.text_right(Ui::kWidth - 10, y, counts + "  prio " + std::to_string(mod.priority), colors::kDim);
            std::string desc = mod.description.empty() ? mod.author : mod.description;
            if (desc.size() > 70) desc = desc.substr(0, 67) + "...";
            ui.text(44, y + 10, desc, colors::kDim);
        }
        if (sel_ >= 0 && size_t(sel_) < mods.size() && !mods[size_t(sel_)].cars.empty())
            render_cars(app, ui, mods[size_t(sel_)], 30 + mods.size() * 26.0f + 8);
        ui.footer("Cross: enable/disable   Square: rescan   Start: open folder   Triangle: back");
    }

private:
    // The selected mod's cars (mods/<id>/cars/*.ini): what loaded, and why the
    // others did not. The database is reloaded whenever the mod set changes.
    void render_cars(App& app, Ui& ui, const ModInfo& mod, float y) {
        if (!cars_loaded_) {
            cars_loaded_ = true;
            db_ok_ = db_.load(*app.vfs());
        }
        ui.text(10, y, "Cars in " + mod.name + (mod.enabled ? "" : " (disabled)"), colors::kAccent);
        y += 14;
        for (const std::string& file : mod.cars) {
            if (y > Ui::kHeight - 40) break;
            std::string base = file.substr(file.find_last_of('/') + 1);
            std::string err;
            for (const std::string& e : db_.mod_car_errors())
                if (e.rfind(file + ": ", 0) == 0) err = e.substr(file.size() + 2);
            const data::ModCar* car = nullptr;
            for (const data::ModCar& c : db_.mod_cars())
                if (c.file == file) car = &c;
            if (!db_ok_) {
                ui.text(20, y, base, colors::kDim);
            } else if (car) {
                const data::Car* c = db_.car(car->label);
                auto sm = c ? db_.summarize(*c) : data::CarDatabase::Summary{};
                char line[160];
                std::snprintf(line, sizeof(line), "%s %s %u, %s%s %.0f kW %u kg, shell %s", sm.maker.c_str(), sm.name.c_str(),
                              sm.year, sm.drive.c_str(), car->electric ? " EV" : "", sm.power_ps / 1.35962, sm.weight_kg,
                              car->donor.c_str());
                std::string t = line;
                if (t.size() > 76) t = t.substr(0, 73) + "...";
                ui.text(20, y, t, colors::kGood);
            } else if (!err.empty()) {
                std::string t = base + ": " + err;
                if (t.size() > 76) t = t.substr(0, 73) + "...";
                ui.text(20, y, t, colors::kBad);
            } else {
                ui.text(20, y, base + (mod.enabled ? ": replaced by another mod" : ""), colors::kDim);
            }
            y += 11;
        }
    }

    int sel_ = 0;
    data::CarDatabase db_;
    bool cars_loaded_ = false, db_ok_ = false;
};

} // namespace

std::unique_ptr<Screen> make_jukebox_screen() { return std::make_unique<JukeboxScreen>(); }
std::unique_ptr<Screen> make_mods_screen() { return std::make_unique<ModsScreen>(); }

GT3R_REGISTER_PLAYER_SCREEN(mods, "mods", "Mods", "Enable or disable installed mods.", 5, [] { return make_mods_screen(); });
GT3R_REGISTER_PLAYER_SCREEN(jukebox, "jukebox", "Music jukebox", "Play the soundtrack streamed from your disc.", 6,
                     [] { return make_jukebox_screen(); });

} // namespace gt3r
