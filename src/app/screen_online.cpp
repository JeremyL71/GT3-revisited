// Online battle lobby: the arcade menu's i.LINK Battle, over IP (docs/dev/ONLINE.md).
//
// Start page: host a battle (UDP port [online] port, default 47630) or join one by
// address; the player name. Lobby page: the players (name, car, ready, ping), the
// course and laps (the host chooses), your car, colour and gearbox, Ready / Start.
// The host starts when everyone is ready; every player's race screen loads, and the
// countdown starts for all at once. After the race everyone comes back here.
//
// Scripted runs (tests, two instances on one machine):
//   GT3R_ONLINE="host,players=2,course=trialmt,laps=1,car=r34_gtr_v_99"
//   GT3R_ONLINE="join=127.0.0.1,car=nsx_type_r_02,name=Two"
// readies at once; the host starts when `players` are in and ready. `page=cars`,
// `page=makers` or `page=courses` opens that picker once in the lobby (screenshots; no auto start then).

#include "app/screen_online.h"

#include "app/app.h"
#include "app/online_pickers.h"
#include "app/screen_registry.h"
#include "app/updates.h"
#include "app/ui.h"
#include "base/log.h"
#include "game/data/car_database.h"
#include "game/gt_mode.h"
#include "game/gt_mode_menu.h"
#include "game/net_car.h"
#include "game/mod_package.h"
#include "base/sha256.h"
#include "menu/flow/menu_set.h"
#include "menu/flow/game_hooks.h"
#include "net/session.h"
#include "race/paint.h"
#include "vfs/vfs.h"

#include <algorithm>
#include <deque>
#include <map>
#include <set>
#include <cstdlib>
#include <string>
#include <vector>

namespace gt3r {

namespace {

using Entry = PickerCar;

struct Script {
    bool have = false, host = false;
    std::string join, name, car, course, server, code, codefile, page;
    int laps = 0, players = 2, paint = -1;
    bool accept = false;
};

Script read_script() {
    Script s;
    const char* env = SDL_getenv("GT3R_ONLINE");
    if (!env) return s;
    s.have = true;
    std::string all = env;
    size_t pos = 0;
    while (pos <= all.size()) {
        size_t end = all.find(',', pos);
        if (end == std::string::npos) end = all.size();
        std::string kv = all.substr(pos, end - pos);
        pos = end + 1;
        size_t eq = kv.find('=');
        std::string k = kv.substr(0, eq), v = eq == std::string::npos ? "" : kv.substr(eq + 1);
        if (k == "host") s.host = true;
        else if (k == "join") s.join = v;
        else if (k == "name") s.name = v;
        else if (k == "car") s.car = v;
        else if (k == "course") s.course = v;
        else if (k == "laps") s.laps = std::atoi(v.c_str());
        else if (k == "players") s.players = std::max(2, std::atoi(v.c_str()));
        else if (k == "paint") s.paint = std::atoi(v.c_str());
        else if (k == "server") s.server = v;
        else if (k == "accept") s.accept = v != "0"; // answer download prompts with yes (tests)
        else if (k == "code") s.code = v;
        else if (k == "codefile") s.codefile = v; // host: write the code here; client: read it (tests)
        else if (k == "page") s.page = v;
    }
    return s;
}

// Database names may hold non-ASCII bytes (e.g. "V.spec"); the UI font is ASCII.
std::string ascii(const std::string& s) {
    std::string out;
    for (unsigned char c : s)
        if (c < 0x80) out += char(c);
        else if ((c & 0xC0) == 0xC0) out += '-';
    return out;
}

bool has_course(const Vfs& vfs, const std::string& label) {
    for (const char* region : {"US", "JP", "UK", "EU"})
        if (vfs.exists(std::string("courses/") + region + "/" + label)) return true;
    return false;
}

class OnlineScreen : public Screen {
public:
    OnlineScreen() : script_(read_script()) {}

    ~OnlineScreen() override {
        if (editing_ && window_) SDL_StopTextInput(window_);
        net::online().close(); // leaving the lobby ends the session
    }

    void update(App& app, double) override {
        if (!init_) init(app);
        net::Session& ns = net::online();
        if (in_race_) { // back from the race
            in_race_ = false;
            app.menu().suppress();
            if (ns.is_host() && !ns.in_lobby()) ns.lobby();
            if (ns.active()) ns.set_ready(false);
        }
        ns.poll();
        while (std::optional<net::Event> e = ns.next_event()) {
            using K = net::Event::Kind;
            switch (e->kind) {
            case K::Joined: status("Joined. Choose your car and press Ready.", false); break;
            case K::Rejected:
            case K::Disconnected:
                if (ns.update_required()) { // the relay server: this game is below its min_online
                    updates().note_required(app, *ns.update_required());
                    updates().open_dialog(net::UpdatePrompt::Required);
                    status(update_text(), true);
                } else {
                    status((e->kind == K::Rejected ? "The host refused: " : "Disconnected: ") + e->text, true);
                }
                page_ = Page::Start;
                break;
            case K::Retire:
                if (const net::Player* p = ns.player(e->player)) status(p->name + " left", false);
                else status("A player left", false);
                break;
            case K::Settings: course_sel_ = find_course(e->course, course_sel_); break;
            case K::Link:
                if (const net::Player* p = ns.player(e->player)) {
                    net::Link l = ns.link(e->player);
                    status(p->name + (l.direct ? ": direct connection" : l.relay ? ": through the relay server"
                                                                                 : ": relayed through the host"),
                           false);
                }
                break;
            case K::Load: start_race(app, *e); return;
            case K::Queue:
                if (ns.update_required()) {
                    updates().note_required(app, *ns.update_required());
                    updates().open_dialog(net::UpdatePrompt::Required);
                }
                status("Relay server: " + e->text, true);
                break;
            case K::PackageDone: install_download(app, e->text); break;
            case K::PackageFailed:
                if (auto it = downloading_.find(e->text); it != downloading_.end()) {
                    status("Download of " + it->second.name + " failed: " + e->course, true);
                    downloading_.erase(it);
                }
                break;
            default: break;
            }
        }
        if (page_ == Page::Lobby) offer_downloads(ns);
        // Waiting for the relay's link slots needs everyone still ready (the race starts by itself).
        if (ns.is_host() && ns.relay_wait().waiting && !ns.all_ready()) {
            ns.cancel_start();
            status("Start cancelled: not everyone is ready", true);
        }
        if (page_ != Page::Start && !ns.active()) page_ = Page::Start;
        if (script_.have) run_script(app);
        if (editing_) return; // text entry has the keyboard (on_event)
        if (page_ == Page::Lobby && !prompts_.empty()) {
            update_prompt(app);
            return;
        }
        if (page_ == Page::Start) update_start(app);
        else if (page_ == Page::Cars) update_car_picker(app);
        else if (page_ == Page::Courses) update_course_picker(app);
        else update_lobby(app);
    }

    void on_event(App& app, const SDL_Event& e) override {
        if (!editing_) return;
        std::string& s = edit_target();
        if (e.type == SDL_EVENT_TEXT_INPUT) {
            for (const char* c = e.text.text; *c; ++c)
                if (s.size() < 63 && u8(*c) >= 32 && u8(*c) < 127) s += *c;
        } else if (e.type == SDL_EVENT_KEY_DOWN) {
            if (e.key.key == SDLK_BACKSPACE && !s.empty()) s.pop_back();
            else if (e.key.key == SDLK_RETURN || e.key.key == SDLK_KP_ENTER) end_edit(app, true);
            else if (e.key.key == SDLK_ESCAPE || e.key.key == SDLK_AC_BACK) end_edit(app, false);
        } else if (e.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN) {
            // Controller typing: up/down change the last character, right adds one, left deletes.
            static const std::string kChars = "0123456789.:abcdefghijklmnopqrstuvwxyz-_[]";
            auto cycle = [&](int d) {
                if (s.empty()) s += '0';
                size_t i = kChars.find(char(std::tolower(u8(s.back()))));
                i = i == std::string::npos ? 0 : (i + kChars.size() + size_t(d)) % kChars.size();
                s.back() = kChars[i];
            };
            switch (e.gbutton.button) {
            case SDL_GAMEPAD_BUTTON_DPAD_UP: cycle(1); break;
            case SDL_GAMEPAD_BUTTON_DPAD_DOWN: cycle(-1); break;
            case SDL_GAMEPAD_BUTTON_DPAD_RIGHT: if (s.size() < 63) s += s.empty() ? '0' : s.back(); break;
            case SDL_GAMEPAD_BUTTON_DPAD_LEFT: if (!s.empty()) s.pop_back(); break;
            case SDL_GAMEPAD_BUTTON_SOUTH: case SDL_GAMEPAD_BUTTON_START: end_edit(app, true); break;
            case SDL_GAMEPAD_BUTTON_EAST: end_edit(app, false); break;
            default: break;
            }
        }
    }

    void render(App& app, Ui& ui) override {
        net::Session& ns = net::online();
        ui.clear();
        if (page_ == Page::Start) render_start(ui);
        else if (page_ == Page::Cars) car_picker_.render(app, ui);
        else if (page_ == Page::Courses) course_picker_.render(app, ui, art_);
        else render_lobby(app, ui, ns);
        if (page_ == Page::Lobby && !prompts_.empty()) render_prompt(ui);
        for (const auto& [hash, d] : downloading_) // progress of the downloads
            if (const net::Session::Download* dl = ns.download(hash); dl && dl->total) {
                std::string t = "Downloading " + d.name + ": " + std::to_string(int(100.0 * dl->received / dl->total)) + " %";
                ui.text_center(Ui::kHeight - 48, t, colors::kAccent);
                break;
            }
        if (!status_.empty())
            ui.text_center(Ui::kHeight - 34, Ui::clip(status_, Ui::kWidth - 40), status_bad_ ? colors::kBad : colors::kGood);
    }

private:
    enum class Page { Start, Lobby, Cars, Courses };

    void init(App& app) {
        init_ = true;
        // Too old for the relay: say so at once (every time Online battle opens).
        if (updates().required() && !script_.have) updates().open_dialog(net::UpdatePrompt::Required);
        app_ = &app;
        window_ = app.window();
        if (app.vfs()) {
            db_.load(*app.vfs());
            colors_.load(*app.vfs());
            build_car_list(app);
            for (const data::Course& c : db_.courses())
                if (c.label.rfind("L_", 0) != 0 && has_course(*app.vfs(), c.label))
                    courses_.push_back({c.label, ascii(c.name.empty() ? c.label : c.name)});
        }
        IniDocument& cfg = app.config();
        name_ = cfg.get("online", "name").value_or("Player");
        address_ = cfg.get("online", "address").value_or("");
        server_ = cfg.get("online", "server").value_or(kDefaultServer);
        code_ = cfg.get("online", "code").value_or("");
        port_ = u16(std::clamp(std::atoi(cfg.get("online", "port").value_or("47630").c_str()), 1, 65535));
        car_sel_ = find(cars_, cfg.get("online", "car").value_or("r34_gtr_v_99"), 0);
        course_sel_ = find_course(cfg.get("online", "course").value_or("trialmt"), 0);
        laps_ = std::clamp(std::atoi(cfg.get("online", "laps").value_or("2").c_str()), 1, 99);
        paint_ = std::max(0, std::atoi(cfg.get("online", "paint").value_or("0").c_str()));
        manual_ = cfg.get("online", "gearbox").value_or("at") == "mt";
        allow_mods_ = cfg.get("online", "mods").value_or("on") != "off";
        ghost_ms_ = u16(std::clamp(std::atoi(cfg.get("online", "ghost_ms").value_or("150").c_str()), 0, 1000));
        stock_only_ = cfg.get("online", "stock_only").value_or("off") == "on";
        max_ps_ = u16(std::clamp(std::atoi(cfg.get("online", "max_ps").value_or("0").c_str()), 0, 2000));
        if (script_.have) {
            if (!script_.name.empty()) name_ = script_.name;
            if (!script_.car.empty()) {
                car_sel_ = find(cars_, script_.car, -1);
                if (car_sel_ < 0) LOG_WARN("Online: no car '%s' on this disc", script_.car.c_str()), car_sel_ = 0;
            }
            if (!script_.course.empty()) course_sel_ = find_course(script_.course, course_sel_);
            if (script_.laps > 0) laps_ = script_.laps;
            if (script_.paint >= 0) paint_ = script_.paint;
        }
        if (cars_.empty()) status("No cars found on the disc", true);
    }

    // GT Mode garage cars first (with their parts and settings), then every disc and mod car.
    void build_car_list(App& app) {
        cars_.clear();
        if (game::ensure_gt_mode_session(menu::flow::vfs_reader(*app.vfs())))
            if (game::GtModeSession* gs = game::gt_mode_session()) {
                const std::vector<game::GarageCar>& g = gs->gt().state().garage;
                for (size_t i = 0; i < g.size(); ++i) {
                    const data::Car* c = db_.car(g[i].car);
                    if (!c) continue;
                    Entry e{g[i].car, "Garage: " + ascii(c->name.empty() ? c->label : c->name), gs->gt().model_of(g[i]),
                            int(i), db_.mod_car(g[i].car) != nullptr, c->maker, g[i].color, g[i].wheel};
                    cars_.push_back(e);
                }
            }
        size_t garage = cars_.size();
        for (const data::Car& c : db_.cars()) {
            std::string model = db_.model_of(c);
            if (model.empty() || !app.vfs()->exists("cars/day/" + model)) continue;
            Entry e{c.label, ascii(c.name.empty() ? c.label : c.name), model, -1, db_.mod_car(c.label) != nullptr, c.maker, 0, {}};
            if (e.mod) e.name += " (mod)";
            cars_.push_back(e);
        }
        std::sort(cars_.begin() + ptrdiff_t(garage), cars_.end(), [](const Entry& a, const Entry& b) { return a.name < b.name; });
    }

    // The chosen car as the lobby sends it.
    game::OnlineCar current_car() const { return car_at(car_sel_, paint_); }
    game::OnlineCar car_at(int index, int paint) const {
        if (cars_.empty() || index < 0 || size_t(index) >= cars_.size()) return {};
        const Entry& e = cars_[size_t(index)];
        if (e.garage >= 0)
            if (game::GtModeSession* gs = game::gt_mode_session())
                if (size_t(e.garage) < gs->gt().state().garage.size())
                    return game::online_car_from_garage(db_, gs->gt().state().garage[size_t(e.garage)]);
        return game::online_car_from_db(db_, e.label, u8(paint));
    }
    bool garage_selected() const { return !cars_.empty() && cars_[size_t(car_sel_)].garage >= 0; }
    bool mod_selected() const { return !cars_.empty() && cars_[size_t(car_sel_)].mod; }

    // Tells the session our car (label for the roster, the full descriptor for everyone).
    void announce_car() {
        net::Session& ns = net::online();
        game::OnlineCar oc = current_car();
        if (oc.kind == game::OnlineCar::Kind::Mod && app_ && app_->vfs()) offer_own_package(oc);
        ns.set_car(oc.car, oc.paint);
        ns.set_car_info(game::encode_online_car(oc));
    }

    // --- mod car downloads (docs/dev/ONLINE.md "Mod cars") ---

    // Our mod car's package: built once per car, offered to the session, its hash in our descriptor.
    void offer_own_package(game::OnlineCar& oc) {
        auto it = own_packages_.find(oc.car);
        if (it == own_packages_.end()) {
            std::string err;
            std::optional<Bytes> blob = game::build_mod_package(db_, *app_->vfs(), oc.car, &err);
            std::optional<game::ModPackage> pkg = blob ? game::parse_mod_package(*blob, &err) : std::nullopt;
            if (!pkg || !game::validate_mod_package(*pkg, db_, oc.car, &err)) { // others would refuse it too
                LOG_WARN("Online: %s cannot be shared: %s", oc.car.c_str(), err.c_str());
                status(oc.name + " cannot be shared (" + err + "); others see a stand-in", true);
                own_packages_[oc.car] = {};
                return;
            }
            OwnPackage op{sha256_hex(*blob), u32(blob->size())};
            net::online().offer_package(op.hash, std::move(*blob));
            it = own_packages_.emplace(oc.car, op).first;
        }
        oc.package = it->second.hash;
        oc.package_size = it->second.size;
        if (oc.package.empty()) oc.kind = game::OnlineCar::Kind::Disc; // not shareable: others use a stand-in
    }

    // Other players' mod cars we lack: ask once per package (when the lobby allows mods).
    void offer_downloads(const net::Session& ns) {
        if (!ns.settings().allow_mods) return;
        for (const net::Player& p : ns.players()) {
            if (int(p.id) == ns.my_id()) continue;
            const Bytes* info = ns.car_info(p.id);
            std::optional<game::OnlineCar> oc = info ? game::decode_online_car(*info) : std::nullopt;
            if (!oc || oc->kind != game::OnlineCar::Kind::Mod || oc->package.empty() || db_.car(oc->car)) continue;
            if (!asked_.insert(oc->package).second) continue;
            prompts_.push_back({p.id, p.name, *oc});
        }
    }

    void update_prompt(App& app) {
        const MenuInput& m = app.menu();
        Prompt p = prompts_.front();
        if (m.pressed(PadButton::Cross) || (script_.have && script_.accept)) {
            prompts_.pop_front();
            downloading_[p.car.package] = {p.car.name.empty() ? p.car.car : p.car.name, p.car.car, p.name};
            net::online().request_package(p.car.package, p.player);
            status("Downloading " + downloading_[p.car.package].name + " from " + p.name + "...", false);
        } else if (m.pressed(PadButton::Circle) || m.pressed(PadButton::Triangle)) {
            prompts_.pop_front();
            status(p.name + "'s car will show as a stand-in", false);
        }
    }

    void render_prompt(Ui& ui) {
        const Prompt& p = prompts_.front();
        float w = 440, h = 104, x = (Ui::kWidth - w) / 2, y = 120;
        ui.fill_round(x, y, w, h, 8, colors::kPanel);
        ui.frame_round(x, y, w, h, 8, 1, colors::kOutline);
        std::string car = p.car.name.empty() ? p.car.car : p.car.name;
        char size[32];
        std::snprintf(size, sizeof size, "%.1f MB", double(p.car.package_size) / (1024.0 * 1024.0));
        ui.text_center_in(x, w, y + 14, p.name + " is racing a mod car you don't have:", colors::kText);
        ui.text_center_in(x, w, y + 32, Ui::clip(car, w - 40) + "  (" + size + ")", colors::kAccent);
        ui.text_center_in(x, w, y + 50, "Download it? It is checked, then added to your mods.", colors::kDim);
        ui.text_center_in(x, w, y + 78, "Cross: download     Circle: no (a stand-in car)", colors::kText);
    }

    void install_download(App& app, const std::string& hash) {
        auto it = downloading_.find(hash);
        std::optional<Bytes> blob = net::online().take_package(hash);
        if (it == downloading_.end() || !blob) return;
        Download d = it->second;
        downloading_.erase(it);
        std::string err;
        std::optional<game::ModPackage> pkg = game::parse_mod_package(*blob, &err);
        if (!pkg || !game::validate_mod_package(*pkg, db_, d.label, &err)) {
            LOG_WARN("Online: refused %s's package for %s: %s", d.from.c_str(), d.label.c_str(), err.c_str());
            status("Refused " + d.name + ": " + err, true);
            return;
        }
        std::optional<std::string> dir = game::install_mod_package(*pkg, hash, app.mods_dir(), d.name, d.from, &err);
        if (!dir) {
            status("Could not install " + d.name + ": " + err, true);
            return;
        }
        LOG_INFO("Online: installed %s from %s in %s", d.label.c_str(), d.from.c_str(), dir->c_str());
        // Mount it and reload the cars (the race screen loads its own database from the VFS).
        std::string keep = cars_.empty() ? "" : cars_[size_t(car_sel_)].label;
        int keep_garage = cars_.empty() ? -1 : cars_[size_t(car_sel_)].garage;
        app.vfs()->scan_mods(app.mods_dir(), &app.config());
        db_ = data::CarDatabase{};
        db_.load(*app.vfs());
        rule_key_.clear();
        build_car_list(app);
        for (size_t i = 0; i < cars_.size(); ++i)
            if (cars_[i].label == keep && cars_[i].garage == keep_garage) car_sel_ = int(i);
        status("Installed " + d.name + " (from " + d.from + ")", false);
    }

    static int find(const std::vector<Entry>& v, const std::string& label, int fallback) {
        for (size_t i = 0; i < v.size(); ++i)
            if (v[i].label == label || v[i].label == "_" + label || v[i].model == label) return int(i);
        return fallback;
    }
    int find_course(const std::string& label, int fallback) const {
        for (size_t i = 0; i < courses_.size(); ++i)
            if (courses_[i].label == label || courses_[i].label == "_" + label) return int(i);
        return fallback;
    }
    const std::string& car_label() const { static const std::string none; return cars_.empty() ? none : cars_[size_t(car_sel_)].label; }
    const std::string& course_label() const { static const std::string none; return courses_.empty() ? none : courses_[size_t(course_sel_)].label; }
    int paint_count() const {
        if (cars_.empty()) return 1;
        return app_ ? int(car_picker_.paints(*app_, cars_[size_t(car_sel_)]).size()) : 1;
    }
    void status(std::string s, bool bad) { status_ = std::move(s), status_bad_ = bad; }

    void remember(App& app) {
        IniDocument& cfg = app.config();
        cfg.set("online", "name", name_);
        cfg.set("online", "address", address_);
        cfg.set("online", "server", server_);
        cfg.set("online", "code", code_);
        cfg.set("online", "car", car_label());
        cfg.set("online", "course", course_label());
        cfg.set("online", "laps", std::to_string(laps_));
        cfg.set("online", "paint", std::to_string(paint_));
        cfg.set("online", "gearbox", manual_ ? "mt" : "at");
        cfg.set("online", "mods", allow_mods_ ? "on" : "off");
        cfg.set("online", "ghost_ms", std::to_string(ghost_ms_));
        cfg.set("online", "stock_only", stock_only_ ? "on" : "off");
        cfg.set("online", "max_ps", std::to_string(max_ps_));
        app.save_config();
    }

    // --- start page ---

    enum StartRow { kCreate, kJoinCode, kHostIp, kJoinIp, kName, kServer, kBack, kStartRows };

    // "Update required to play online: version 0.2.0" (the update check, or a relay's refusal).
    static std::string update_text() {
        const net::GameVersion& v = updates().latest();
        return "Update required to play online" + (v.known() ? ": version " + v.number() : std::string());
    }

    void update_start(App& app) {
        const MenuInput& m = app.menu();
        sel_ = (sel_ + m.vertical() + kStartRows) % kStartRows;
        if (m.pressed(PadButton::Triangle)) {
            app.pop();
            return;
        }
        bool circle = m.pressed(PadButton::Circle);
        if (!m.pressed(PadButton::Cross) && !circle) return;
        // Below the relay's min_online: no sessions through it; the update dialog again (direct-IP
        // and offline play stay open).
        const bool blocked = updates().required() && (sel_ == kCreate || (sel_ == kJoinCode && !circle && !code_.empty()));
        if (blocked && !server_.empty()) {
            updates().open_dialog(net::UpdatePrompt::Required);
            return;
        }
        switch (sel_) {
        case kCreate:
            if (server_.empty()) begin_edit(Edit::Server);
            else create(app);
            break;
        case kJoinCode:
            if (code_.empty() || circle) begin_edit(Edit::Code);
            else if (server_.empty()) begin_edit(Edit::Server);
            else join_code(app);
            break;
        case kHostIp: host(app); break;
        case kJoinIp:
            if (address_.empty() || circle) begin_edit(Edit::Address);
            else join(app);
            break;
        case kName: begin_edit(Edit::Name); break;
        case kServer: begin_edit(Edit::Server); break;
        default: app.pop(); break;
        }
    }

    // Through the relay server: a session code, no port forward.
    void create(App& app) {
        std::string err;
        net::Session& ns = net::online();
        if (!ns.host_via(server_, name_, car_label(), u8(paint_), &err)) {
            status("Cannot create a session: " + err, true);
            return;
        }
        push_settings();
        announce_car();
        page_ = Page::Lobby;
        sel_ = 0;
        status("Creating the session...", false);
        remember(app);
    }

    void join_code(App& app) {
        std::string err;
        net::Session& ns = net::online();
        if (!ns.join_code(server_, code_, name_, car_label(), u8(paint_), &err)) {
            status("Cannot join: " + err, true);
            return;
        }
        announce_car();
        page_ = Page::Lobby;
        sel_ = kCar;
        status("Looking for session " + net::format_code(net::normalize_code(code_)) + "...", false);
        remember(app);
    }

    void host(App& app) {
        std::string err;
        net::Session& ns = net::online();
        if (!ns.host(port_, name_, car_label(), u8(paint_), &err)) {
            status("Cannot host: " + err, true);
            return;
        }
        push_settings();
        announce_car();
        page_ = Page::Lobby;
        sel_ = 0;
        std::string where;
        for (const std::string& a : net::local_addresses()) where += (where.empty() ? "" : ", ") + a;
        status("Hosting on port " + std::to_string(ns.port()) + (where.empty() ? "" : " - LAN address " + where), false);
        remember(app);
    }

    void join(App& app) {
        std::string err;
        net::Session& ns = net::online();
        if (!ns.join(address_, name_, car_label(), u8(paint_), &err)) {
            status("Cannot join: " + err, true);
            return;
        }
        announce_car();
        page_ = Page::Lobby;
        sel_ = kCar;
        status("Connecting to " + address_ + "...", false);
        remember(app);
    }

    void render_start(Ui& ui) {
        ui.header("ONLINE BATTLE", "i.LINK Battle over the internet or a LAN");
        const char* labels[kStartRows] = {"Create a session", "Join with a code", "Host by address (LAN / port forward)",
                                          "Join by address",  "Name",             "Relay server",
                                          "Back"};
        for (int i = 0; i < kStartRows; ++i) {
            float y = 46 + i * 24.0f + (i >= kName ? 10 : 0);
            bool f = i == sel_;
            if (f) ui.fill_round(40, y - 6, 560, 21, 4, colors::kHighlight);
            ui.text(56, y, labels[i], f ? colors::kAccent : colors::kText);
            std::string v;
            if (i == kCreate) v = server_.empty() ? "(set the relay server first)" : "you get a code to share";
            if (i == kJoinCode) v = code_.empty() ? "(enter the code)" : net::format_code(net::normalize_code(code_));
            if (i == kHostIp) v = "UDP port " + std::to_string(port_);
            if (i == kJoinIp) v = address_.empty() ? "(enter address)" : address_;
            if (i == kName) v = name_;
            if (i == kServer) v = server_.empty() ? "(none)" : server_;
            bool edit = editing_ && ((i == kJoinCode && edit_ == Edit::Code) || (i == kJoinIp && edit_ == Edit::Address) ||
                                     (i == kName && edit_ == Edit::Name) || (i == kServer && edit_ == Edit::Server));
            if (edit) v = edit_target() + "_";
            if (!v.empty()) ui.text(330, y, Ui::clip(v, 260), edit ? colors::kAccent : colors::kDim);
        }
        std::string hint = editing_ ? "Type, Enter to finish (pad: up/down letter, right add, left delete, A done)"
                                    : "Cross: choose   Circle: edit the code / address   Triangle: back";
        ui.footer(hint);
        ui.text_center(Ui::kHeight - 62, "With a code nobody forwards ports: players connect directly when they can,",
                       colors::kDim);
        ui.text_center(Ui::kHeight - 50, "otherwise through the relay server.", colors::kDim);
    }

    // --- text entry ---

    enum class Edit { Address, Name, Code, Server } edit_ = Edit::Address;
    bool editing_ = false;
    std::string edit_backup_;
    std::string& edit_target() {
        switch (edit_) {
        case Edit::Address: return address_;
        case Edit::Name: return name_;
        case Edit::Code: return code_;
        default: return server_;
        }
    }

    void begin_edit(Edit which) {
        edit_ = which;
        editing_ = true;
        edit_backup_ = edit_target();
        if (window_) SDL_StartTextInput(window_);
    }

    void end_edit(App& app, bool keep) {
        if (!keep) edit_target() = edit_backup_;
        if (edit_ == Edit::Name && name_.empty()) name_ = "Player";
        editing_ = false;
        if (window_) SDL_StopTextInput(window_);
        app.menu().suppress(); // the keys typed are no menu input
        remember(app);
    }

    // --- lobby page ---

    enum Row { kCourse, kLaps, kMods, kGhost, kRule, kPower, kCar, kColour, kGearbox, kGo, kKick, kLeave, kRows };

    // Host: the players that can be removed (everyone but us), and their warnings.
    std::vector<const net::Player*> kickable(const net::Session& ns) const {
        std::vector<const net::Player*> out;
        for (const net::Player& p : ns.players())
            if (p.id != 0) out.push_back(&p);
        return out;
    }
    static std::string warnings_of(const net::Session& ns, u8 id) {
        std::string t;
        int n = 0;
        for (const net::Session::FlagRecord& f : ns.flags())
            if (f.suspect == id) {
                ++n;
                if (t.size() < 80) t += (t.empty() ? "" : "; ") + std::string(net::flag_text(f.kind)) + " (seen by " +
                                        (f.reporter_name.empty() ? "?" : f.reporter_name) + ")";
            }
        return n ? std::to_string(n) + (n == 1 ? " warning: " : " warnings: ") + t : "";
    }
    static constexpr u16 kPowerSteps[] = {0, 200, 250, 300, 350, 400, 450, 500, 600, 700};
    static constexpr int kPowerCount = int(sizeof(kPowerSteps) / sizeof(kPowerSteps[0]));

    // Our car against the lobby's rules (empty = fine).
    std::string rule_problem(const net::Session& ns) const { return rule_problem(ns, current_car()); }
    std::string rule_problem(const net::Session& ns, const game::OnlineCar& oc) const {
        Bytes key = game::encode_online_car(oc);
        key.push_back(ns.settings().stock_only ? 1 : 0);
        key.push_back(u8(ns.settings().max_ps)), key.push_back(u8(ns.settings().max_ps >> 8));
        if (key == rule_key_) return rule_why_; // the spec is rebuilt only when the car or the rules change
        rule_key_ = key;
        std::optional<sim::CarSpec> spec = game::online_car_spec(db_, oc);
        rule_why_.clear();
        game::car_meets_rules(oc, spec ? &*spec : nullptr, ns.settings().stock_only, ns.settings().max_ps, &rule_why_);
        return rule_why_;
    }
    mutable Bytes rule_key_;
    mutable std::string rule_why_;
    // Ghost above this round trip (0 = always collide).
    static constexpr u16 kGhostMs[] = {0, 80, 100, 150, 200, 300};
    static constexpr int kGhostSteps = int(sizeof(kGhostMs) / sizeof(kGhostMs[0]));
    static std::string ghost_text(u16 ms) { return ms ? "above " + std::to_string(ms) + " ms round trip" : "never (always contact)"; }

    void push_settings() {
        net::Settings st;
        st.course = course_label();
        st.laps = laps_;
        st.allow_mods = allow_mods_;
        st.ghost_ping_ms = ghost_ms_;
        st.stock_only = stock_only_;
        st.max_ps = max_ps_;
        net::online().set_settings(st);
    }

    void update_lobby(App& app) {
        net::Session& ns = net::online();
        const MenuInput& m = app.menu();
        int before = sel_;
        sel_ = (sel_ + m.vertical() + kRows) % kRows;
        if (sel_ != before) kick_armed_ = false;
        if (m.pressed(PadButton::Triangle)) {
            ns.close();
            page_ = Page::Start;
            status("Left the session", false);
            return;
        }
        const net::Player* me = ns.my_id() >= 0 ? ns.player(u8(ns.my_id())) : nullptr;
        bool ready = me && me->ready && !ns.is_host();
        int h = m.horizontal();
        if (m.pressed(PadButton::L1)) h = -10;
        if (m.pressed(PadButton::R1)) h = 10;
        bool cross = m.pressed(PadButton::Cross);
        if (h || cross) {
            switch (sel_) {
            case kCourse:
                if (!ns.is_host() || courses_.empty()) break;
                if (cross) { // the course page
                    course_picker_.open(courses_, course_sel_, [this](const std::string& c) { return course_length(c); });
                    page_ = Page::Courses;
                    picker_sound(app, "ok");
                    break;
                }
                if (!h) break;
                course_sel_ = (course_sel_ + h % int(courses_.size()) + int(courses_.size())) % int(courses_.size());
                push_settings();
                break;
            case kLaps:
                if (!ns.is_host() || !h) break;
                laps_ = std::clamp(laps_ + (h > 0 ? 1 : -1), 1, 99);
                push_settings();
                break;
            case kMods:
                if (!ns.is_host()) break;
                allow_mods_ = !allow_mods_;
                push_settings();
                break;
            case kRule:
                if (!ns.is_host()) break;
                stock_only_ = !stock_only_;
                push_settings();
                break;
            case kPower: {
                if (!ns.is_host() || !h) break;
                int k = 0;
                for (int j = 0; j < kPowerCount; ++j)
                    if (kPowerSteps[j] == max_ps_) k = j;
                max_ps_ = kPowerSteps[std::clamp(k + (h > 0 ? 1 : -1), 0, kPowerCount - 1)];
                push_settings();
                break;
            }
            case kGhost: {
                if (!ns.is_host() || !h) break;
                int k = 0;
                for (int j = 0; j < kGhostSteps; ++j)
                    if (kGhostMs[j] == ghost_ms_) k = j;
                ghost_ms_ = kGhostMs[std::clamp(k + (h > 0 ? 1 : -1), 0, kGhostSteps - 1)];
                push_settings();
                break;
            }
            case kCar:
                if (cars_.empty() || ready) break;
                if (cross) { // the car page
                    car_picker_.open(app, art_, db_, cars_, car_sel_, paint_, [this](int i, int paint) {
                        return car_note(i, paint);
                    });
                    page_ = Page::Cars;
                    picker_sound(app, "ok");
                    break;
                }
                if (!h) break;
                car_sel_ = (car_sel_ + h % int(cars_.size()) + int(cars_.size())) % int(cars_.size());
                paint_ = 0;
                announce_car();
                break;
            case kColour:
                if (!h || ready || garage_selected()) break; // a garage car keeps its own paint
                paint_ = (paint_ + (h > 0 ? 1 : -1) + paint_count()) % paint_count();
                announce_car();
                break;
            case kGearbox:
                if (h || cross) manual_ = !manual_;
                break;
            case kGo:
                if (!cross) break;
                if (ns.is_host() && ns.relay_wait().waiting) { // in the relay's queue: give up waiting
                    ns.cancel_start();
                    status("Stopped waiting for the relay server", false);
                } else if (ns.is_host()) {
                    std::string mod_driver;
                    if (!ns.settings().allow_mods)
                        for (const net::Player& p : ns.players())
                            if (const Bytes* info = ns.car_info(p.id))
                                if (auto oc = game::decode_online_car(*info); oc && oc->kind == game::OnlineCar::Kind::Mod)
                                    mod_driver = p.name;
                    std::string rule_breaker;
                    for (const net::Player& p : ns.players())
                        if (const Bytes* info = ns.car_info(p.id))
                            if (auto oc = game::decode_online_car(*info)) {
                                std::optional<sim::CarSpec> sp = game::online_car_spec(db_, *oc); // null: not on this disc
                                std::string why;
                                if (!game::car_meets_rules(*oc, sp ? &*sp : nullptr, ns.settings().stock_only, ns.settings().max_ps, &why))
                                    rule_breaker = p.name + "'s car: " + why;
                            }
                    if (!mod_driver.empty()) status(mod_driver + " has a mod car, and this lobby does not allow them", true);
                    else if (!rule_breaker.empty()) status(rule_breaker, true);
                    else if (ns.all_ready()) ns.load();
                    else status(ns.players().size() < 2 ? "Waiting for players to join" : "Not everyone is ready", true);
                } else if (ns.joined()) {
                    std::string why = rule_problem(ns);
                    if (!ready && mod_selected() && !ns.settings().allow_mods)
                        status("This lobby does not allow mod cars", true);
                    else if (!ready && !why.empty())
                        status("Your car: " + why, true);
                    else ns.set_ready(!ready);
                }
                break;
            case kKick: {
                if (!ns.is_host()) break;
                std::vector<const net::Player*> k = kickable(ns);
                if (k.empty()) break;
                if (h) {
                    kick_sel_ = (kick_sel_ + (h > 0 ? 1 : -1) + int(k.size())) % int(k.size());
                    kick_armed_ = false;
                }
                kick_sel_ = std::clamp(kick_sel_, 0, int(k.size()) - 1);
                if (cross) {
                    const net::Player* p = k[size_t(kick_sel_)];
                    if (!kick_armed_) {
                        kick_armed_ = true;
                        status("Press Cross again to remove " + p->name, true);
                    } else {
                        std::string name = p->name;
                        ns.kick(p->id);
                        kick_armed_ = false;
                        status("Removed " + name, false);
                    }
                }
                break;
            }
            case kLeave:
                if (!cross) break;
                ns.close();
                page_ = Page::Start;
                sel_ = 0;
                break;
            default: break;
            }
            if (sel_ != kGo && sel_ != kLeave) remember(app);
        }
    }

    // A player's paint: the colour of paint index `paint` of database car `car` (nullopt = unknown).
    std::optional<u32> paint_rgb(App& app, const std::string& car, int paint) {
        const data::Car* c = db_.car(car);
        if (!c) return std::nullopt;
        Entry e{car, "", db_.model_of(*c), -1, false, {}, 0, {}};
        std::vector<PaintInfo> p = car_picker_.paints(app, e);
        if (paint < 0 || size_t(paint) >= p.size()) return std::nullopt;
        return p[size_t(paint)].rgb;
    }

    void render_lobby(App& app, Ui& ui, const net::Session& ns) {
        std::string head = ns.via_server() ? (ns.is_host() ? "Hosting" : "Joined")
                           : ns.is_host() ? "Hosting - port " + std::to_string(ns.port())
                                          : "Joined";
        if (!ns.is_host() && ns.rtt_ms() > 0) head += " - ping " + std::to_string(int(ns.rtt_ms())) + " ms";
        ui.header("ONLINE BATTLE", head);
        // The session code, big, for the host to read out.
        if (ns.via_server()) {
            std::string code = net::format_code(ns.is_host() ? ns.code() : net::normalize_code(code_));
            if (code.empty()) ui.text_right(Ui::kWidth - 24, 30, "getting a code...", colors::kDim);
            else {
                ui.text_right(Ui::kWidth - 24, 30, "SESSION CODE", colors::kDim);
                int scale = Ui::text_width(code, 3) > 170 ? 2 : 3; // clear of the players' link column
                float w = Ui::text_width(code, scale);
                ui.text_big(Ui::kWidth - 24 - w, 44, code, scale, colors::kAccent);
            }
        }
        // Players.
        ui.text(24, 30, "PLAYERS", colors::kDim);
        float y = 44;
        for (const net::Player& p : ns.players()) {
            bool me = int(p.id) == ns.my_id();
            ui.text(24, y, Ui::clip(p.name, 110), me ? colors::kAccent : colors::kText);
            std::string car = p.car;
            for (const Entry& e : cars_)
                if (e.label == p.car && e.garage < 0) car = e.name;
            if (const Bytes* info = ns.car_info(p.id))
                if (std::optional<game::OnlineCar> oc = game::decode_online_car(*info)) {
                    car = oc->name.empty() ? oc->car : oc->name;
                    if (oc->kind == game::OnlineCar::Kind::Garage) car = "Garage: " + car;
                    if (oc->kind == game::OnlineCar::Kind::Mod) car += db_.car(oc->car) ? " (mod)" : " (mod, not installed)";
                }
            if (std::optional<u32> rgb = paint_rgb(app, p.car, p.paint)) draw_paint_chip(ui, 140, y, 8, 8, *rgb, false);
            ui.text(152, y, Ui::clip(car, 130), colors::kDim);
            if (ns.is_host())
                for (const net::Session::FlagRecord& f : ns.flags())
                    if (f.suspect == p.id) {
                        ui.text(118, y, "!", colors::kBad); // has warnings (details on "Remove a player")
                        break;
                    }
            const char* st = p.id == 0 ? "HOST" : p.ready ? "READY" : "...";
            ui.text(290, y, st, p.id == 0 || p.ready ? colors::kGood : colors::kDim);
            if (!me) {
                net::Link l = ns.link(p.id);
                if (l.known) {
                    std::string t = std::string(l.direct ? "direct " : l.relay ? "relay " : "via host ") +
                                    std::to_string(int(l.rtt_ms + 0.5)) + " ms";
                    u16 gms = ns.settings().ghost_ping_ms;
                    bool ghost = gms && l.rtt_ms > gms;
                    ui.text(350, y, t, ghost ? colors::kBad : l.direct ? colors::kGood : colors::kDim);
                }
            }
            y += 14;
        }
        if (!ns.joined()) ui.text(24, y, "Connecting...", colors::kDim);
        // Settings.
        const net::Player* meP = ns.my_id() >= 0 ? ns.player(u8(ns.my_id())) : nullptr;
        bool ready = meP && meP->ready && !ns.is_host();
        std::string course = ns.is_host() ? course_label() : ns.course();
        for (const PickerCourse& e : courses_)
            if (e.label == course) course = e.name;
        // The course's still at the right of the settings (the rows beside it are clipped).
        const SDL_FRect thumb{472, 124, 148, 111};
        auto row = [&](int r, const std::string& label, const std::string& value, bool enabled, float indent = 0) {
            float ry = 130 + r * 15.0f;
            bool f = r == sel_;
            float right = ry - 3 < thumb.y + thumb.h ? thumb.x - 6 : 620;
            if (f) ui.fill_round(20, ry - 3, right - 20, 14, 4, colors::kHighlight);
            ui.text(36, ry, label, !enabled ? colors::kDim : f ? colors::kAccent : colors::kText);
            ui.text(200 + indent, ry, Ui::clip(value, right - 206 - indent), enabled ? colors::kText : colors::kDim);
        };
        if (!course.empty()) {
            std::string label = ns.is_host() ? course_label() : ns.course();
            ui.fill_round(thumb.x - 2, thumb.y - 2, thumb.w + 4, thumb.h + 4, 4, colors::kOutline);
            if (!art_.course_still(app, label, thumb, true)) {
                ui.fill(thumb.x, thumb.y, thumb.w, thumb.h, colors::kPanel);
                if (!art_.course_icon(app, label, SDL_FRect{thumb.x + 20, thumb.y + 20, thumb.w - 40, thumb.h - 40}))
                    ui.text_center_in(thumb.x, thumb.w, thumb.y + thumb.h / 2 - 4, Ui::clip(course, thumb.w - 8), colors::kDim);
            } else {
                art_.course_logo(app, label, SDL_FRect{thumb.x + 4, thumb.y + thumb.h - 34, 90, 30});
            }
        }
        row(kCourse, "Course", (ns.is_host() ? "< " + course + " >   (Cross: course page)" : course), ns.is_host());
        row(kLaps, "Laps", ns.is_host() ? "< " + std::to_string(laps_) + " >" : std::to_string(ns.laps()), ns.is_host());
        bool mods = ns.is_host() ? allow_mods_ : ns.settings().allow_mods;
        row(kMods, "Mod cars", mods ? "allowed (others are offered the download)" : "not allowed", ns.is_host());
        u16 gms = ns.is_host() ? ghost_ms_ : ns.settings().ghost_ping_ms;
        row(kGhost, "Ghost cars", ns.is_host() ? "< " + ghost_text(gms) + " >" : ghost_text(gms), ns.is_host());
        bool stock = ns.is_host() ? stock_only_ : ns.settings().stock_only;
        row(kRule, "Car rules", stock ? "stock cars only (no tuned garage cars)" : "any car", ns.is_host());
        u16 cap = ns.is_host() ? max_ps_ : ns.settings().max_ps;
        std::string cap_text = cap ? "at most " + std::to_string(cap) + " PS" : "no limit";
        row(kPower, "Power limit", ns.is_host() ? "< " + cap_text + " >" : cap_text, ns.is_host());
        std::string problem = ns.joined() ? rule_problem(ns) : "";
        if (cars_.empty()) row(kCar, "Car", "-", !ready);
        else {
            // The car's strip icon (the arcade car page's) before its name.
            const Entry& e = cars_[size_t(car_sel_)];
            float ry = 130 + float(kCar) * 15.0f;
            bool icon = art_.car_icon(app, e.label, SDL_FRect{200, ry - 3, 26, 14});
            std::string hint = !problem.empty() ? "   " + problem : ready ? "" : "   (Cross: car page)";
            row(kCar, "Car", "< " + e.name + " >" + hint, !ready, icon ? 30.0f : 0.0f);
        }
        {
            std::vector<PaintInfo> pal = cars_.empty() ? std::vector<PaintInfo>{} : car_picker_.paints(app, cars_[size_t(car_sel_)]);
            int cur = garage_selected() ? 0 : std::clamp(paint_, 0, std::max(0, int(pal.size()) - 1));
            float ry = 130 + float(kColour) * 15.0f;
            if (!pal.empty()) draw_paint_chip(ui, 200, ry - 1, 18, 9, pal[size_t(cur)].rgb, false);
            std::string pname = pal.empty() ? "" : pal[size_t(cur)].name;
            if (garage_selected()) row(kColour, "Colour", (pname.empty() ? "" : pname + "  ") + "(the garage car's own)", false, 26);
            else
                row(kColour, "Colour",
                    "< " + (pname.empty() ? "" : pname + "  ") + std::to_string(paint_ + 1) + " of " +
                        std::to_string(paint_count()) + " >",
                    !ready, 26);
        }
        row(kGearbox, "Transmission", manual_ ? "Manual" : "Automatic", true);
        // "Relay busy: 2nd in line, about 3 min" while the race waits for the relay's link slots.
        std::string wait = net::relay_wait_text(ns.relay_wait());
        if (!wait.empty())
            row(kGo, ns.is_host() ? "Cancel start" : ready ? "Ready - waiting for the host" : "Ready",
                wait + " - the race starts by itself", true);
        else
            row(kGo, ns.is_host() ? "Start race" : ready ? "Ready - waiting for the host" : "Ready",
                ns.is_host() ? (ns.all_ready() ? "everyone is ready" : "waiting for everyone to be ready") : "", true);
        if (ns.is_host()) {
            std::vector<const net::Player*> k = kickable(ns);
            if (k.empty()) row(kKick, "Remove a player", "(nobody else here)", false);
            else {
                const net::Player* p = k[size_t(std::clamp(kick_sel_, 0, int(k.size()) - 1))];
                std::string w = warnings_of(ns, p->id);
                row(kKick, kick_armed_ ? "Remove? Cross again" : "Remove a player", "< " + p->name + " >  " + w, true);
            }
        } else {
            row(kKick, "Remove a player", "(the host's choice)", false);
        }
        row(kLeave, "Leave", "", true);
        ui.footer("Up/Down: choose   Left/Right: change   Cross: confirm / car and course pages   Triangle: leave");
    }

    // --- pickers ---

    void update_car_picker(App& app) {
        CarPicker::Result r = car_picker_.update(app);
        if (r == CarPicker::Result::None) return;
        page_ = Page::Lobby;
        const net::Player* me = net::online().my_id() >= 0 ? net::online().player(u8(net::online().my_id())) : nullptr;
        if (r != CarPicker::Result::Chosen || (me && me->ready && !net::online().is_host())) return;
        car_sel_ = car_picker_.car();
        paint_ = cars_[size_t(car_sel_)].garage >= 0 ? 0 : car_picker_.paint();
        announce_car();
        remember(app);
    }

    void update_course_picker(App& app) {
        CoursePicker::Result r = course_picker_.update(app);
        if (r == CoursePicker::Result::None) return;
        page_ = Page::Lobby;
        if (r != CoursePicker::Result::Chosen || !net::online().is_host()) return;
        course_sel_ = course_picker_.course();
        push_settings();
        remember(app);
    }

    // A car on the car page against the lobby's rules ("" = fine).
    std::string car_note(int index, int paint) const {
        const net::Session& ns = net::online();
        if (!ns.joined() || index < 0 || size_t(index) >= cars_.size()) return "";
        if (cars_[size_t(index)].mod && !ns.settings().allow_mods) return "This lobby does not allow mod cars";
        std::string why = rule_problem(ns, car_at(index, paint));
        return why.empty() ? "" : "Not allowed here: " + why;
    }

    // A course's length in metres from the arcade course lists (0 = unknown).
    u32 course_length(const std::string& label) const {
        if (game::GtModeSession* gs = game::gt_mode_session())
            for (const auto& [game_name, list] : gs->gt().data().arcade_courses)
                for (const game::GtData::ArcadeCourse& c : list)
                    if (c.course == label && c.length_m) return c.length_m;
        return 0;
    }

    // --- race ---

    void start_race(App& app, const net::Event& e) {
        if (!app.vfs()) return;
        OnlineLaunch l;
        l.course = e.course;
        l.laps = e.laps;
        game::OnlineCar oc = current_car();
        l.car = oc.car;
        l.paint = oc.paint;
        l.car_info = game::encode_online_car(oc);
        l.manual = manual_;
        LOG_INFO("Online: race on %s, %d laps, %zu cars", l.course.c_str(), l.laps, e.grid.size());
        in_race_ = true;
        page_ = Page::Lobby; // a picker left open closes for the race
        app.push(make_online_race_screen(l));
    }

    // --- scripted runs ---

    void run_script(App& app) {
        net::Session& ns = net::online();
        if (!ns.active() && !script_connected_) {
            if (!script_.server.empty()) server_ = script_.server;
            if (script_.host && !script_.server.empty()) {
                script_connected_ = true;
                create(app);
            } else if (script_.host) {
                script_connected_ = true;
                host(app);
            } else if (!script_.code.empty() || !script_.codefile.empty()) {
                std::string code = script_.code;
                if (code.empty()) { // wait for the host's code file
                    if (FILE* f = std::fopen(script_.codefile.c_str(), "r")) {
                        char buf[16] = {};
                        if (std::fgets(buf, sizeof buf, f)) code = buf;
                        std::fclose(f);
                    }
                    while (!code.empty() && (code.back() == '\n' || code.back() == '\r')) code.pop_back();
                    if (code.empty()) return;
                }
                script_connected_ = true;
                code_ = code;
                join_code(app);
            } else if (!script_.join.empty()) {
                script_connected_ = true;
                address_ = script_.join;
                join(app);
            }
            return;
        }
        if (ns.is_host() && !ns.code().empty() && !script_.codefile.empty() && !script_code_written_) {
            script_code_written_ = true;
            if (FILE* f = std::fopen(script_.codefile.c_str(), "w")) std::fprintf(f, "%s\n", ns.code().c_str()), std::fclose(f);
        }
        if (!ns.joined()) return;
        if (!script_.page.empty()) { // a picker for screenshots
            if (!script_page_opened_ && page_ == Page::Lobby) {
                script_page_opened_ = true;
                if (script_.page == "cars" || script_.page == "makers") {
                    car_picker_.open(app, art_, db_, cars_, car_sel_, paint_, [this](int i, int paint) { return car_note(i, paint); });
                    if (script_.page == "makers") car_picker_.show_makers();
                    page_ = Page::Cars;
                } else if (script_.page == "courses") {
                    course_picker_.open(courses_, course_sel_, [this](const std::string& c) { return course_length(c); });
                    page_ = Page::Courses;
                }
            }
            return;
        }
        const net::Player* me = ns.player(u8(ns.my_id()));
        if (!ns.is_host() && me && !me->ready && ns.in_lobby() && prompts_.empty() && downloading_.empty())
            ns.set_ready(true);
        if (ns.is_host() && !script_started_ && int(ns.players().size()) >= script_.players && ns.all_ready()) {
            script_started_ = true;
            ns.load();
        }
    }

    App* app_ = nullptr;
    SDL_Window* window_ = nullptr;
    bool init_ = false, in_race_ = false;
    Script script_;
    bool script_connected_ = false, script_started_ = false, script_code_written_ = false, script_page_opened_ = false;
    data::CarDatabase db_;
    race::CarColors colors_;
    std::vector<Entry> cars_;
    std::vector<PickerCourse> courses_;
    PickerArt art_;
    CarPicker car_picker_;
    CoursePicker course_picker_;
    Page page_ = Page::Start;
    int sel_ = 0;
    std::string name_, address_, server_, code_;
    u16 port_ = net::kDefaultPort;
    int car_sel_ = 0, course_sel_ = 0, laps_ = 2, paint_ = 0;
    bool manual_ = false;
    struct OwnPackage {
        std::string hash; // empty: the car cannot be shared
        u32 size = 0;
    };
    std::map<std::string, OwnPackage> own_packages_; // by car label
    struct Prompt {
        u8 player = 0;
        std::string name;
        game::OnlineCar car;
    };
    std::deque<Prompt> prompts_;
    std::set<std::string> asked_; // package hashes already offered to the user
    struct Download {
        std::string name, label, from;
    };
    std::map<std::string, Download> downloading_; // by package hash
    bool allow_mods_ = true;
    u16 ghost_ms_ = 150;
    bool stock_only_ = false;
    u16 max_ps_ = 0;
    int kick_sel_ = 0;
    bool kick_armed_ = false;
    std::string status_;
    bool status_bad_ = false;
};

} // namespace

std::unique_ptr<Screen> make_online_lobby_screen() { return std::make_unique<OnlineScreen>(); }

GT3R_REGISTER_PLAYER_SCREEN(online, "online", "Online battle", "Race other players over a LAN or the internet (i.LINK Battle).", 2,
                     [] { return make_online_lobby_screen(); });

} // namespace gt3r
