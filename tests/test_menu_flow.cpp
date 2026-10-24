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
#include <cmath>
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

// The name entry's text box (menus §12.2): the cursor starts after the starting text; a key
// goes in at the cursor (a key without text is a space) while there is room; Back Space steps
// left and deletes there, at the start the first character; L1 / R1 stop at the ends.
TEST(menu_flow_text_box) {
    TextBox b;
    b.reset("GT3R", 6);
    CHECK(b.text == "GT3R" && b.cursor == 4);
    CHECK(b.move(-1) && b.move(-1) && b.cursor == 2);
    CHECK(b.insert("X") && b.text == "GTX3R" && b.cursor == 3);
    CHECK(b.insert("") && b.text == "GTX 3R" && b.cursor == 4);
    CHECK(!b.insert("Y") && b.text == "GTX 3R"); // full
    CHECK(b.back_space() && b.text == "GTX3R" && b.cursor == 3);
    CHECK(b.move(-1) && b.move(-1) && b.move(-1) && !b.move(-1) && b.cursor == 0);
    CHECK(b.back_space() && b.text == "TX3R" && b.cursor == 0); // the first character goes
    CHECK(b.move(1) && b.move(1) && b.move(1) && b.move(1) && !b.move(1) && b.cursor == 4);
    b.reset("", 15);
    CHECK(!b.back_space() && b.text.empty() && b.cursor == 0); // nothing to delete: no sound
    b.reset("ABCDEFGHIJKLMNOPQRS", 15);
    CHECK(b.text == "ABCDEFGHIJKLMNO" && b.cursor == 15);
}

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

// One popup level: a popup opened over a popup replaces it, and closing it returns to
// the base page (the tune shop's Purchase? -> Install parts now? -> Installed chain).
TEST(menu_flow_popup_replaces_popup) {
    MenuSet set(map_reader(synthetic_files()), "test", "US");
    MenuController c(set);
    c.open("top");
    settle(c);
    c.set_focus(2);
    CHECK(c.popup("opt"));
    settle(c);
    CHECK(c.depth() == 1 && c.page_name() == "opt");
    CHECK(c.popup("status"));
    settle(c);
    CHECK(c.depth() == 1 && c.page_name() == "status");
    press(c, MenuKey::Cancel);
    settle(c);
    CHECK(c.depth() == 0 && c.page_name() == "top" && c.focus() == 2);
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

// Initial focus (menus §4.2): the control's hook, else the first item leading to the page we
// came from, else 0. Nothing remembers the item confirmed last where two items share a target.
TEST(menu_flow_initial_focus_rule) {
    auto button = [](const char* name, float y) {
        SynElement e;
        e.name = name;
        e.y = y;
        e.text = name;
        return e;
    };
    SynPage detail{"detail", {button("ok", 300)}, {{"defaultLayer", {}}}};
    SynPage home{"home", {button("go", 300)}, {{"defaultLayer", {}}}};
    SynPage list{"list", {button("a", 300), button("b", 250), button("c", 200)}, {{"defaultLayer", {}}}};
    SynItem ok{"ok", "", "", "", "", -1, 3, {}};
    SynItem go{"go", "", "", "", "list", -1, 0, {}};
    SynItem a{"a", "", "", "", "detail", -1, 0, {}};
    SynItem b{"b", "", "", "", "detail", -1, 0, {}};
    SynItem cc{"c", "", "", "", "", -1, 2, {}};
    std::map<std::string, Bytes> files;
    files["menu/test.mbl"] = write_mbl({
        {"detail", 1, 0x100, "", "return", "", {ok}},
        {"home", 1, 0x100, "", "", "", {go}},
        {"list", 1, 0x100, "", "home", "", {a, b, cc}},
    });
    files["menu/US/test1.pmb"] = write_pmb({detail, home, list});
    MenuSet set(map_reader(std::move(files)), "test", "US");
    MenuController c(set);
    CHECK(c.open("home"));
    settle(c);
    press(c, MenuKey::Confirm);
    settle(c);
    CHECK(c.page_name() == "list" && c.focus() == 0);
    c.set_focus("b");
    press(c, MenuKey::Confirm);
    settle(c);
    CHECK(c.page_name() == "detail");
    // Back: the first item leading to the page we left ("a", not "b").
    press(c, MenuKey::Cancel);
    settle(c);
    CHECK(c.page_name() == "list" && c.focus() == 0);
    // From a page no item leads to: item 0, not "b" confirmed before.
    c.set_focus("b");
    press(c, MenuKey::Confirm);
    settle(c);
    press(c, MenuKey::Cancel);
    settle(c);
    press(c, MenuKey::Cancel);
    settle(c);
    CHECK(c.page_name() == "home" && c.focus() == 0);
    press(c, MenuKey::Confirm);
    settle(c);
    CHECK(c.page_name() == "list" && c.focus() == 0);
    // A control hook comes first.
    c.initial_focus = [](const std::string& p) { return p == "list" ? 2 : -1; };
    press(c, MenuKey::Cancel);
    settle(c);
    press(c, MenuKey::Confirm);
    settle(c);
    CHECK(c.page_name() == "list" && c.focus() == 2);
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

namespace {

// Where a draw puts its element: the replacement box, else the element's own place.
bool drawn_at(const ElementDraw& d, float x, float y) {
    const float dx = d.has_box ? d.box.x : d.element->x + d.dx, dy = d.has_box ? d.box.y : d.element->y + d.dy;
    return std::fabs(dx - x) < 0.01f && std::fabs(dy - y) < 0.01f;
}

// Checks the focused look of a button over one pulse period (30 frames) and returns the frames
// that broke a rule: the highlight (alt) drawn once a frame, where the button is and in its
// place in the draw order, and the normal look pulsing over it in the same place.
int highlight_misses(MenuController& c, const Element& button, int& pulses) {
    const Element& alt = button.alt.at(0);
    int misses = 0;
    pulses = 0;
    for (int f = 0; f < 30; ++f) {
        c.tick(0);
        int highlights = 0;
        bool ok = true;
        for (const ElementDraw& d : c.draw_list()) {
            if (d.kind != ElementDraw::Kind::Element || (d.element != &alt && d.element != &button)) continue;
            ok = ok && drawn_at(d, button.x, button.y);
            if (d.element == &alt) {
                ++highlights;
                ok = ok && d.z == button.z && d.box.w == alt.w && d.box.h == alt.h;
            } else if (d.alpha > 0) {
                ++pulses;
            }
        }
        if (!ok || highlights != 1) ++misses;
    }
    return misses;
}

} // namespace

TEST(menu_flow_disabled_overlay) {
    // menus §5.3: a page with a `disable` widget (the race quick menus': black at alpha 0xB1)
    // draws it over every disabled button in the button's box (the widget's own place and size
    // unused) and leaves the button's normal look as it is; a focused disabled button's
    // highlight is drawn at half brightness under it. Disabled items keep the focus moving.
    auto button = [](const char* name, float x) {
        SynElement e = el(name, x, 380, 60, 60, 8);
        e.has_alt = true;
        e.focus_style = 0x2000;
        return e;
    };
    SynElement dis = el("disable", 700, 300, 88, 76, 8, 0xB1000000);
    dis.widget = true;
    dis.widget_class = "disable";
    SynPage root{"root", {button("a", 100), button("b", 200)}, {{"defaultLayer", {dis}}}};
    SynItem a{"a", "", "", "act_a", "", -1, 2, {}}, b{"b", "", "", "act_b", "", -1, 2, {}};
    std::map<std::string, Bytes> files;
    files["menu/dis.mbl"] = write_mbl({{"root", 1, 0x1E1, "", "", "", {a, b}}});
    files["menu/US/dis1.pmb"] = write_pmb({root});
    MenuSet set(map_reader(files), "dis", "US");
    MenuController c(set);
    c.enabled = [](const std::string&, const MblItem& it) { return it.name != "a"; };
    CHECK(c.open("root"));
    settle(c);
    c.take_sounds();
    c.take_events();
    const Element& ea = *c.items().at(0).element;
    const Element& eb = *c.items().at(1).element;
    struct Seen {
        float alt_a = -1, alt_b = -1, look_a = -1, look_b = -1; // brightness, -1 not drawn
        int overlays_a = 0, overlays = 0;
    };
    auto seen = [&] {
        Seen s;
        for (const ElementDraw& d : c.draw_list()) {
            if (d.kind != ElementDraw::Kind::Element || !d.element) continue;
            if (d.element == &ea.alt.at(0)) s.alt_a = d.brightness;
            if (d.element == &eb.alt.at(0)) s.alt_b = d.brightness;
            if (d.element == &ea && d.alpha > 0) s.look_a = d.brightness;
            if (d.element == &eb && d.alpha > 0) s.look_b = d.brightness;
            if (d.element->name == "disable") {
                ++s.overlays;
                s.overlays_a += d.has_box && drawn_at(d, 100, 380) && d.box.w == 60 && d.box.h == 60;
            }
        }
        return s;
    };
    CHECK(c.focus() == 0);
    Seen s = seen();
    CHECK(s.alt_a == 0.5f && s.overlays == 1 && s.overlays_a == 1 && s.look_b == 1.0f);
    // Confirm refuses with SE "disable" and nothing else.
    press(c, MenuKey::Confirm);
    CHECK(c.take_sounds() == std::vector<std::string>{"disable"} && c.take_events().empty() && !c.busy());
    press(c, MenuKey::Right);
    CHECK(c.focus() == 1);
    for (int i = 0; i < 5; ++i) c.tick(0);
    s = seen();
    CHECK(s.alt_a < 0 && s.look_a == 1.0f && s.overlays == 1 && s.overlays_a == 1 && s.alt_b == 1.0f);
}

// A focused button's highlight (its alt look) sits on the button however the layout places the
// alt (menus §5.3): the GT Auto page's Wheel Shop highlight is 10 units right of its button and
// 23 above it in z, the dealer car pages' Spec highlight is parked off-screen, the credits page's
// last dots' highlights are at 0, 0 under the background, and a choice's highlight is shifted a
// little. The normal look pulses over the highlight in the same place, never beside it.
TEST(menu_flow_highlight_on_its_item) {
    auto button = [](const char* name, float x, float y, float dx = 0, float dy = 0, float dz = 1) {
        SynElement e = el(name, x, y, 160, 32, 23);
        e.text = name;
        e.has_alt = true;
        e.focus_style = 0x2000; // pulsing
        e.alt_dx = dx, e.alt_dy = dy, e.alt_dz = dz;
        return e;
    };
    SynElement chosen = button("assist_on", 300, 268.899f, 0, 1.881f);
    SynElement other = button("assist_off", 400, 268.899f, 0, 1.881f);
    chosen.focus_style = other.focus_style = 0; // choices have no focus style
    std::map<std::string, Bytes> files;
    files["menu/hl.mbl"] = write_mbl({
        {"auto", 1, 0x1E0, "", "", "", {{"wash", "", "", "", "", -1, 2, {}},
                                        {"oil", "", "", "", "", -1, 2, {}},
                                        {"wheel", "", "", "", "", -1, 2, {}},
                                        {"spec", "", "", "", "", -1, 2, {}},
                                        {"dot", "", "", "", "", -1, 2, {}},
                                        {"multi_assist", "", "", "", "", -1, 0x14,
                                         {{"assist_on", "", "", "", "", -1, 3, {}},
                                          {"assist_off", "", "", "", "", -1, 3, {}}}}}},
    });
    files["menu/US/hl1.pmb"] = write_pmb({{"auto",
                                           {button("wash", 260, 204), button("oil", 208, 160),
                                            button("wheel", 166, 116, 10, 0, 23), button("spec", 452, 68, 452, 0, 46),
                                            button("dot", 528, 387, -528, -387, -23), button("multi_assist", 64, 268),
                                            chosen, other},
                                           {{"defaultLayer", {el("bg", 0, 0, 640, 448, 1, 0xFF202020)}}}}});
    MenuSet set(map_reader(files), "hl", "US");
    MenuController c(set);
    CHECK(c.open("auto"));
    settle(c);
    CHECK(c.items().size() == 6);
    for (int i = 0; i < 6; ++i) {
        c.set_focus(i);
        const Element& b = *c.items()[size_t(i)].element;
        int pulses = 0;
        CHECK(highlight_misses(c, b, pulses) == 0);
        CHECK(pulses > 0); // the normal look pulsed over the highlight
    }
    // The chosen choice of the multi item (no pulse: the highlight alone, on the choice).
    c.set_focus(5);
    const Element* on = c.items()[5].children.at(0);
    CHECK(on && on->name == "assist_on" && c.choice("multi_assist") == 0);
    int pulses = 0;
    CHECK(on && highlight_misses(c, *on, pulses) == 0 && pulses == 0);
    // The button whose highlight was pinned at 0, 0 under the background draws it over the
    // background now (in the button's place in the draw order).
    c.set_focus(4);
    c.tick(0);
    auto list = c.draw_list();
    auto at = [&](const Element* e) {
        for (size_t k = 0; k < list.size(); ++k)
            if (list[k].element == e && list[k].kind == ElementDraw::Kind::Element) return int(k);
        return -1;
    };
    const auto ref = c.page(); // named: GCC flags a reference into a temporary
    CHECK(at(&c.items()[4].element->alt[0]) > at(&ref.layout().layers[0].elements[0]));
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

// Every button of the disc's menu pages draws its highlight (alt look) on itself when focused
// (menus §5.3): where the button is, or where its scrolling list puts it, in the button's place
// in the draw order. Many alts sit elsewhere in the layout files (shifted, parked off-screen, at
// 0, 0, at another z); the GT Auto page's Wheel Shop highlight is 10 units right of its button.
TEST(menu_flow_disc_highlights_on_their_items) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    const char* sets[] = {"arcade",       "arcade_mini",   "car_get",          "consolation_prize", "gt_mode",
                          "gtmode_mini",  "language",      "lc_get",           "logger",            "memorycard",
                          "name_entry",   "not_get_car",   "option",           "pause",             "records",
                          "replay_panel", "setting",       "trophy_get",       "qm_arc_tt_free",    "qm_arc_tt_ltd",
                          "qm_gt_event",  "qm_gt_license", "qm_gt_result",     "qm_arc_result"};
    int buttons = 0, elsewhere = 0, on_button = 0, wheel_shop = 0;
    for (const char* name : sets) {
        if (!vfs.exists(std::string("menu/") + name + ".mbl")) continue;
        MenuSet set(vfs_reader(vfs), name, "US");
        for (const MblPage& p : set.flow().pages) {
            MenuController c(set);
            if (!c.open(p.name)) continue;
            for (int i = 0; i < 200 && c.busy(); ++i) c.tick(0);
            // The progress popups ("Washing . . .") hide their buttons until the game gives the
            // controls back (menus §3): measure them as they are drawn then.
            if (c.ignoring_control()) c.give_control();
            for (int i = 0; i < int(c.items().size()); ++i) {
                const Element* e = c.items()[size_t(i)].element;
                if (!e || e->alt.empty()) continue;
                const Element& alt = e->alt[0];
                ++buttons;
                if (alt.x != e->x || alt.y != e->y || alt.z != e->z) ++elsewhere;
                c.set_focus(i);
                SDL_FRect box{};
                if (!c.item_box(i, box)) continue;
                int drawn = 0;
                bool placed = true;
                for (const ElementDraw& d : c.draw_list())
                    if (d.element == &alt && d.kind == ElementDraw::Kind::Element) {
                        ++drawn;
                        placed = placed && drawn_at(d, box.x, box.y) && d.z == e->z;
                    }
                if (drawn == 1 && placed) ++on_button;
                else std::printf("  %s:%s %s: highlight drawn %d time(s) at another place\n", name, p.name.c_str(),
                                 e->name.c_str(), drawn);
                if (std::string(name) == "gt_mode" && p.name == "at/at" && e->name == "sl" && drawn == 1 && placed &&
                    std::fabs(alt.x - e->x - 10) < 0.01f)
                    ++wheel_shop;
            }
        }
    }
    std::printf("  %d buttons with a highlight (%d placed elsewhere in the layout), %d drawn on their button\n",
                buttons, elsewhere, on_button);
    CHECK(buttons > 1000 && elsewhere > 500);
    CHECK(on_button == buttons);
    CHECK(wheel_shop == 1);
}

// The GT-mode control's race page hooks on the disc's pages (menus §4.2): 6 licence pages,
// 49 series and 27 championship series (the nine endurance pages spelt with a capital E get
// none). On every hooked page each race item's own code gives its own index (the info item
// first).
TEST(menu_flow_disc_race_page_hooks) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    MenuSet set(vfs_reader(vfs), "gt_mode", "US");
    int counts[4] = {}, races = 0, races_ok = 0;
    for (const MblPage& p : set.flow().pages) {
        const RacePageHook hook = race_page_hook(p.name, p.items);
        ++counts[int(hook)];
        if (hook == RacePageHook::None) continue;
        const MenuSet::PageRef ref = set.page(p.name);
        if (!ref) continue;
        int index = 0;
        for (const MblItem& it : p.items) {
            bool built = false;
            for (const Element& e : ref.layout().items) built |= e.name == it.name;
            if (!built) continue;
            std::string code = it.name;
            int type = 4;
            if (it.action == "set_race_code_champ") ++code.back(), type = 2;
            if (it.action == "set_race_code" || it.action == "set_race_code_champ" || it.action == "go_license") {
                ++races;
                if (race_page_focus(hook, code, type) == index) ++races_ok;
                else std::printf("  %s: %s -> %d, item %d\n", p.name.c_str(), code.c_str(), race_page_focus(hook, code, type), index);
            }
            ++index;
        }
    }
    std::printf("  hooks: %d licence, %d series, %d championship series; %d/%d race items\n",
                counts[int(RacePageHook::License)], counts[int(RacePageHook::Series)],
                counts[int(RacePageHook::ChampionshipSeries)], races_ok, races);
    CHECK(counts[int(RacePageHook::License)] == 6 && counts[int(RacePageHook::Series)] == 49 &&
          counts[int(RacePageHook::ChampionshipSeries)] == 27);
    CHECK(races > 300 && races_ok == races);
    const MblPage* end = nullptr;
    for (const MblPage& p : set.flow().pages)
        if (p.name == "r/end/Eakasaka_n") end = &p;
    CHECK(end && race_page_hook(end->name, end->items) == RacePageHook::None);
}

// Every car dealer page (the countries' maker lists and the makers' car lists) is a strip
// of tiles in its window: the focused tile is inside the window, and a pointer over a
// tile focuses it (VR / mouse). England and Germany name their window "menu_dealer";
// the Toyota list keeps four of its cars parked below the screen instead of to the right.
TEST(menu_flow_disc_dealer_strips) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    MenuSet set(vfs_reader(vfs), "gt_mode", "US");
    int pages = 0, strips = 0, hovered = 0;
    for (const MblPage& p : set.flow().pages) {
        if (p.name.rfind("cd/cd_", 0) != 0 || p.name.size() < 4 || p.name.substr(p.name.size() - 4) == "_car")
            continue;
        ++pages;
        MenuController c(set);
        if (!c.open(p.name)) {
            std::printf("  %s: does not open\n", p.name.c_str());
            continue;
        }
        for (int i = 0; i < 200 && c.busy(); ++i) c.tick(0); // the page's entry transition
        SDL_FRect box, clip;
        const bool in_window = c.item_box(c.focus(), box, &clip) && clip.w < 640 && box.x >= clip.x - 0.5f &&
                               box.x + box.w <= clip.x + clip.w + 0.5f;
        if (in_window) ++strips;
        else std::printf("  %s: not a strip in a window\n", p.name.c_str());
        if (c.items().size() >= 2 && c.item_box(1, box)) {
            c.hover(box.x + box.w * 0.5f, box.y + box.h * 0.5f);
            if (c.focus() == 1) ++hovered;
            else std::printf("  %s: hover over tile 1 left focus %d\n", p.name.c_str(), c.focus());
        } else {
            ++hovered; // one maker (Australia, Belgium)
        }
    }
    std::printf("  %d dealer pages: %d strips, %d hover ok\n", pages, strips, hovered);
    CHECK(pages >= 40);
    CHECK(strips == pages);
    CHECK(hovered == pages);
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
    CHECK(f.take_transition() == TransitionType::Fadeout); // menus §6: leaving a menu = type 0
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
