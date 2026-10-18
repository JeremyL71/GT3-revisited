// Interactive menu runtime (src/menu/flow) over synthetic .mbl/.pmb files,
// plus disc checks of the shipped menu flow.

#include "test_framework.h"

#include "menu_flow_synth.h"

#include "formats/menu/mbl.h"
#include "formats/menu/pmb.h"
#include "menu/flow/game_flow.h"
#include "menu/flow/menu_controller.h"
#include "menu/flow/menu_set.h"
#include "vfs/vfs.h"

#include <algorithm>
#include <cstring>
#include <set>

using namespace gt3r;
using namespace gt3r::menu;
using namespace gt3r::menu::flow;
using namespace gt3r::test::menusynth;

namespace {

// Two pages in pmb 1 ("top" with three buttons, "opt" with a multi item and
// a back target), one page in pmb 2 that imports "status" from pmb 1.
SynElement el(const char* name, float x, float y, float w, float h, float z, u32 rgba = 0xFFFFFFFF) {
    SynElement e;
    e.name = name;
    e.x = x;
    e.y = y;
    e.w = w;
    e.h = h;
    e.z = z;
    e.rgba = rgba;
    return e;
}

std::map<std::string, Bytes> synthetic_files() {
    auto button = [](const char* name, float y) {
        SynElement e;
        e.name = name;
        e.y = y;
        e.flags = 0x1E; // fade-in flag set, like the game's page items
        e.text = name;
        e.has_alt = true;
        return e;
    };
    SynPage top{"top", {button("item1", 300), button("item2", 250), button("item3", 200)},
                {{"defaultLayer", {el("bg", 0, 0, 640, 448, 0, 0xFF202020)}},
                 {"hidden", {el("tmpl", 0, 0, 10, 10, 0)}},
                 {"layer1", {el("help1", 0, 0, 10, 10, 2)}},
                 {"layer2", {el("help2", 0, 0, 10, 10, 2)}}}};
    SynPage opt{"opt", {button("multi_x", 300), button("plain", 250)}, {{"defaultLayer", {}}}};
    SynPage status{"status", {}, {{"defaultLayer", {el("bar", 0, 400, 640, 48, 50)}}}};
    SynPage sub{"sub", {button("go", 300)}, {{"defaultLayer", {}}}};

    SynItem it1{"item1", "", "layer1", "single_race", "", -1, 2, {}};
    SynItem it2{"item2", "", "layer2", "", "opt", 1, 0, {}};
    SynItem it3{"item3", "", "", "", "sub", 2, 0, {}};
    SynItem multi{"multi_x", "", "", "", "", -1, 0x14, {{"x_on", "", "", "commit", "", -1, 3, {}},
                                                        {"x_off", "", "", "commit", "", -1, 3, {}}}};
    SynItem plain{"plain", "", "", "plain_action", "", -1, 2, {}};
    SynItem go{"go", "", "", "", "opt", 1, 0, {}};

    std::map<std::string, Bytes> files;
    files["menu/test.mbl"] = write_mbl({
        {"opt", 1, 0x122, "", "top", "commit", {multi, plain}},
        {"status", 1, 0x168, "", "", "", {}},
        {"sub", 2, 0x1E1, "status", "return", "", {go}},
        {"top", 1, 0x1E6, "", "", "cancel_select_game", {it1, it2, it3}},
    });
    files["menu/US/test1.pmb"] = write_pmb({opt, status, top});
    files["menu/US/test2.pmb"] = write_pmb({sub});
    return files;
}

FileReader map_reader(std::map<std::string, Bytes> files) {
    auto shared = std::make_shared<std::map<std::string, Bytes>>(std::move(files));
    return [shared](const std::string& p) -> std::optional<Bytes> {
        auto it = shared->find(p);
        if (it == shared->end()) return std::nullopt;
        return it->second;
    };
}

// Runs frames with nothing held, until the controller accepts input.
void settle(MenuController& c) {
    for (int i = 0; i < 120 && c.busy(); ++i) c.tick(0);
    c.tick(0);
}

void press(MenuController& c, MenuKey k) {
    c.tick(key_bit(k));
    c.tick(0);
}

bool has_event(const std::vector<MenuEvent>& ev, MenuEvent::Kind k, std::string_view action = {}) {
    return std::any_of(ev.begin(), ev.end(),
                       [&](const MenuEvent& e) { return e.kind == k && (action.empty() || e.action == action); });
}

} // namespace

TEST(menu_flow_synthetic_files_parse) {
    auto files = synthetic_files();
    MblFile m = MblFile::parse(files["menu/test.mbl"]);
    CHECK(!m.legacy && m.pages.size() == 4);
    CHECK(m.pages[0].items.at(0).children.size() == 2);
    PmbFile p = PmbFile::parse(files["menu/US/test1.pmb"]);
    CHECK(p.pages.size() == 3);
    CHECK(p.pages[2].items.size() == 3 && p.pages[2].items[0].alt.size() == 1);
}

TEST(menu_flow_navigation) {
    MenuSet set(map_reader(synthetic_files()), "test", "US");
    MenuController c(set);
    CHECK(c.open("top"));
    CHECK(c.busy()); // the build blocks input for a few frames
    settle(c);
    CHECK(c.items().size() == 3);
    CHECK(c.focus() == 0);
    c.take_sounds();
    c.take_events();

    // Focused layer follows the focus; other item layers and "hidden" stay off.
    const auto page_ref = c.page(); // named: GCC flags a reference into a temporary
    const Page& top = page_ref.layout();
    auto layer_index = [&](const char* n) {
        for (int i = 0; i < int(top.layers.size()); ++i)
            if (top.layers[i].name == n) return i;
        return -1;
    };
    CHECK(c.layer_visible(layer_index("defaultLayer")));
    CHECK(c.layer_visible(layer_index("layer1")));
    CHECK(!c.layer_visible(layer_index("layer2")));
    CHECK(!c.layer_visible(layer_index("hidden")));

    press(c, MenuKey::Down);
    CHECK(c.focus() == 1);
    CHECK(c.layer_visible(layer_index("layer2")) && !c.layer_visible(layer_index("layer1")));
    auto snd = c.take_sounds();
    CHECK(snd.size() == 1 && snd[0] == "select");
    // Left/Right do nothing on a vertical list.
    press(c, MenuKey::Right);
    CHECK(c.focus() == 1);
    // Wrap (page flag 2): up from the first item goes to the last.
    c.set_focus(0);
    press(c, MenuKey::Up);
    CHECK(c.focus() == 2);

    // The focused item draws its alternative (focused) element.
    auto dl = c.draw_list();
    bool alt_drawn = std::any_of(dl.begin(), dl.end(), [&](const ElementDraw& d) {
        return d.element == &c.items()[2].element->alt[0];
    });
    CHECK(alt_drawn);

    // Auto-repeat: press, then one event 31 frames later, then every 4 frames.
    c.set_focus(0);
    c.take_sounds();
    int moves = 0;
    for (int i = 0; i < 40; ++i) {
        int before = c.focus();
        c.tick(key_bit(MenuKey::Down));
        if (c.focus() != before) ++moves;
    }
    c.tick(0);
    CHECK(moves == 4); // frames 1, 32, 36, 40

    // Link item (kind 0): page switch after the confirm feedback.
    c.set_focus(1);
    c.take_sounds();
    press(c, MenuKey::Confirm);
    snd = c.take_sounds();
    CHECK(!snd.empty() && snd[0] == "next"); // page flags 0xC0: "next" + blink feedback
    CHECK(c.busy());
    settle(c);
    CHECK(c.page_name() == "opt");
    CHECK(has_event(c.take_events(), MenuEvent::Kind::PageChanged));

    // Multi item: left/right change the sub-choice (kind 0x14: horizontal, clamped).
    CHECK(c.focused() && c.focused()->flow->name == "multi_x");
    press(c, MenuKey::Right);
    CHECK(c.choice("multi_x") == 1);
    CHECK(has_event(c.take_events(), MenuEvent::Kind::Choice, "commit"));
    press(c, MenuKey::Right);
    CHECK(c.choice("multi_x") == 1);
    // Up/Down still move the outer list.
    press(c, MenuKey::Down);
    CHECK(c.focus() == 1);
    press(c, MenuKey::Up);

    // Cancel goes to the back target with its back action; the focus lands
    // on the item that leads to the page we left.
    press(c, MenuKey::Cancel);
    CHECK(has_event(c.take_events(), MenuEvent::Kind::BackAction, "commit"));
    settle(c);
    CHECK(c.page_name() == "top");
    CHECK(c.focus() == 1);

    // Command item (kind 2): the action runs at once, the menu finishes after the feedback.
    c.set_focus(0);
    c.take_events();
    press(c, MenuKey::Confirm);
    CHECK(has_event(c.take_events(), MenuEvent::Kind::Action, "single_race"));
    settle(c);
    CHECK(c.finished() && c.result() == 1);
    CHECK(has_event(c.take_events(), MenuEvent::Kind::Finish, "single_race"));

    // Cancel at the root page (no back target) exits with the back action.
    c.open("top");
    settle(c);
    c.take_events();
    press(c, MenuKey::Cancel);
    auto ev = c.take_events();
    CHECK(has_event(ev, MenuEvent::Kind::Exit, "cancel_select_game"));
    CHECK(c.finished() && c.result() == 0);
}

TEST(menu_flow_return_and_import) {
    MenuSet set(map_reader(synthetic_files()), "test", "US");
    MenuController c(set);
    c.open("top");
    settle(c);
    c.set_focus("item3");
    press(c, MenuKey::Confirm);
    settle(c);
    CHECK(c.page_name() == "sub");
    // The page lives in pmb 2 and imports "status" from pmb 1.
    CHECK(c.page().scene == set.scene(2));
    auto dl = c.draw_list();
    bool bar = std::any_of(dl.begin(), dl.end(),
                           [](const ElementDraw& d) { return d.element && d.element->name == "bar"; });
    CHECK(bar);
    // back="return" goes to the page we came from.
    press(c, MenuKey::Cancel);
    settle(c);
    CHECK(c.page_name() == "top");
    CHECK(c.focus() == 2);
}

TEST(menu_flow_disabled_and_fade) {
    MenuSet set(map_reader(synthetic_files()), "test", "US");
    MenuController c(set);
    c.enabled = [](const std::string&, const MblItem& it) { return it.name != "item2"; };
    c.open("top");
    // Elements with the fade-in flag (bits 20-23) start transparent and reach
    // full opacity after 24 frames.
    c.tick(0);
    auto alpha_of = [&](const char* name) {
        for (const ElementDraw& d : c.draw_list())
            if (d.element && d.element->name == name && d.kind == ElementDraw::Kind::Element) return d.alpha;
        return -1.0f;
    };
    CHECK(alpha_of("item2") > 0.0f && alpha_of("item2") < 0.1f);
    for (int i = 0; i < 30; ++i) c.tick(0);
    CHECK(alpha_of("item2") == 1.0f);
    // Disabled items can be focused but refuse confirm.
    c.take_sounds();
    press(c, MenuKey::Down);
    CHECK(c.focus() == 1);
    press(c, MenuKey::Confirm);
    auto snd = c.take_sounds();
    CHECK(snd.size() == 2 && snd[1] == "disable");
    CHECK(c.page_name() == "top" && !c.busy());
}

TEST(menu_flow_disc_targets) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    // The menu sets loaded by the game's control classes.
    const char* sets[] = {"arcade",       "arcade_mini",   "car_get",          "consolation_prize", "gt_mode",
                          "gtmode_mini",  "language",      "lc_get",           "logger",            "memorycard",
                          "name_entry",   "not_get_car",   "option",           "pause",             "records",
                          "replay_panel", "setting",       "trophy_get",       "qm_arc_tt_free",    "qm_arc_tt_ltd",
                          "qm_gt_event",  "qm_gt_license", "qm_gt_result",     "qm_arc_result"};
    int pages = 0, found = 0, links = 0, links_ok = 0, sets_ok = 0;
    for (const char* name : sets) {
        if (!vfs.exists(std::string("menu/") + name + ".mbl")) continue;
        MenuSet set(vfs_reader(vfs), name, "US");
        ++sets_ok;
        for (const MblPage& p : set.flow().pages) {
            ++pages;
            if (set.page(p.name)) ++found;
            else std::printf("  missing page %s:%s (pmb %d)\n", name, p.name.c_str(), pmb_number(p));
            auto check_target = [&](const std::string& t) {
                // "return" = previous page; two qm_* pages keep a stale texture path
                // ("T:/menu/qm/...png") in the import field, which the game never resolves.
                if (t.empty() || t == "return" || t.find(':') != std::string::npos) return;
                ++links;
                if (set.flow().find_page(t) >= 0) ++links_ok;
                else std::printf("  dangling target %s:%s -> %s\n", name, p.name.c_str(), t.c_str());
            };
            check_target(p.back);
            check_target(import_page(p));
            for (const MblItem& it : p.items) check_target(it.link);
        }
    }
    std::printf("  %d sets, %d/%d pages found in their pmb, %d/%d targets exist\n", sets_ok, found, pages,
                links_ok, links);
    CHECK(sets_ok >= 20);
    CHECK(found == pages);
    CHECK(links_ok == links);
}

namespace {

// Runs the flow until it accepts input again (or a request/switch is done).
void flow_settle(GameFlow& f) {
    for (int i = 0; i < 200; ++i) {
        if (!f.switching() && f.controller() && !f.controller()->busy()) break;
        f.tick(0);
    }
    f.tick(0);
}

void flow_press(GameFlow& f, MenuKey k) {
    f.tick(key_bit(k));
    f.tick(0);
    flow_settle(f);
}

} // namespace

TEST(menu_flow_disc_title_to_time_trial) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    flow_settle(f);
    CHECK(f.control() == "gt_mode" && f.controller()->page_name() == "root");
    CHECK(f.controller()->focused() && f.controller()->focused()->flow->name == "arcade");
    // Title -> arcade (Fadeout, type 0, while switching controls).
    flow_press(f, MenuKey::Confirm);
    CHECK(f.take_transition() == TransitionType::Fadeout); // menus.md §6: leaving a menu = type 0
    flow_settle(f);
    CHECK(f.control() == "arcade" && f.controller()->page_name() == "ar/top");
    // "Time Trial" is the second item (action "compulsory").
    flow_press(f, MenuKey::Down);
    CHECK(f.controller()->focused()->flow->action == "compulsory");
    flow_press(f, MenuKey::Confirm);
    flow_settle(f);
    CHECK(f.controller()->page_name() == "ar/select_compulsory");
    CHECK(f.carousel() && f.carousel()->entries.size() >= 5);
    flow_press(f, MenuKey::Right);
    CHECK(f.carousel()->index == 1);
    flow_press(f, MenuKey::Confirm);
    for (int i = 0; i < 60 && !f.waiting_for_game(); ++i) f.tick(0);
    auto req = f.take_request();
    CHECK(req.has_value());
    if (req) {
        CHECK(req->mode == "time_trial");
        CHECK(req->course == "tt02");
    }
    // Back from the race: the arcade menu reopens on the same game.
    f.game_finished();
    flow_settle(f);
    CHECK(f.controller() && f.controller()->page_name() == "ar/top");
    CHECK(f.controller()->focused()->flow->action == "compulsory");
    // Cancel on ar/top returns to the title, focused on "arcade".
    flow_press(f, MenuKey::Cancel);
    flow_settle(f);
    CHECK(f.control() == "gt_mode" && f.controller()->page_name() == "root");
    CHECK(f.controller()->focus() == 0);
    // GT mode: the map with the status bar import page; cancel goes back.
    flow_press(f, MenuKey::Down);
    flow_press(f, MenuKey::Confirm);
    flow_settle(f);
    CHECK(f.controller()->page_name() == "map");
    flow_press(f, MenuKey::Cancel);
    flow_settle(f);
    CHECK(f.controller()->page_name() == "root" && f.controller()->focus() == 1);
}
