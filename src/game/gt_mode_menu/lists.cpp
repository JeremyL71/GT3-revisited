// GT-mode menu bindings: the code-driven lists and the per-frame input (on_frame): the GT Mode
// map pointer, the garage list and its sorting, the Cars Permitted list, the wheel shop's tiles
// and the wheel inventory, the dealer's paint picker, the settings sliders; their geometry for
// touch and a hovering pointer.

#include "game/gt_mode_menu/internal.h"

#include "game/gt_setting.h"

#include <algorithm>
#include <cmath>

namespace gt3r::game {

namespace gt_menu {

namespace {

void map_centre(const MenuController& ctl, int i, float& x, float& y) {
    const Element* e = i >= 0 && i < int(ctl.items().size()) ? ctl.items()[size_t(i)].element : nullptr;
    x = e ? e->x + e->w * 0.5f : 320;
    y = e ? e->y + e->h * 0.5f : 224;
}

// GT Mode map: the pointer eases to its building, which takes the focus on arrival.
void ease_map_pointer(MenuController& ctl) {
    MenuState& ms = menu_state();
    if (ctl.page_name() == "map" && ctl.depth() == 0 && ms.map.active && ms.map.target >= 0) {
        // Focus set by other means (taps, returning to the page): the pointer follows it.
        if (!ms.map.moving && ctl.focus() >= 0) ms.map.target = ctl.focus();
        float tx, ty;
        map_centre(ctl, ms.map.target, tx, ty);
        ms.map.x += (tx - ms.map.x) * 0.3f;
        ms.map.y += (ty - ms.map.y) * 0.3f;
        float ex = tx - ms.map.x, ey = ty - ms.map.y;
        if (ex * ex + ey * ey < 144 && ms.map.moving) {
            ms.map.moving = false;
            if (ctl.focus() != ms.map.target && ctl.items()[size_t(ms.map.target)].flow)
                ctl.set_focus(ctl.items()[size_t(ms.map.target)].flow->name);
        }
    } else if (ctl.page_name() != "map") {
        ms.map.active = false;
    }
}

bool list_rows(const MenuController& ctl, const char* window, int n, int cursor, GarageRows& g) {
    menu::flow::MenuSet::PageRef ref = ctl.page();
    if (!ref || ctl.items().empty() || !ctl.items()[0].element) return false;
    g.tmpl = ctl.items()[0].element;
    g.win = find_widget_named(ref.layout(), window);
    if (!g.win) return false;
    g.row = g.tmpl->h + 2;
    g.rows = std::max(1, int(g.win->h / g.row));
    g.n = n;
    g.first = std::clamp(cursor - g.rows / 2, 0, std::max(0, g.n - g.rows));
    return true;
}

bool garage_rows(const MenuController& ctl, GtModeSession& s, GarageRows& g) {
    const std::string& page = ctl.page_name();
    if (page != "hm/hm_gr" && page != "hm/hm_gr_trade") return false;
    return list_rows(ctl, page == "hm/hm_gr_trade" ? "menu_garage_trade" : "menu_garage",
                     int(s.list_gt().state().garage.size()), s.garage_cursor, g);
}

} // namespace

// The race information's Cars Permitted popup.
bool regulated_rows(const MenuController& ctl, GtModeSession& s, int n, GarageRows& g) {
    return ctl.page_name() == "00cmn/car_reguration" && list_rows(ctl, "menu_regulated_car", n, s.regulated_cursor, g);
}

bool wheel_row(const MenuController& ctl, WheelRow& r) {
    if (!ctl.has_page() || ctl.depth() != 0 || !starts(ctl.page_name(), "at/ws/ws_")) return false;
    r.win = find_widget_named(ctl.page().layout(), "menu_wheel_shop");
    if (!r.win) return false;
    r.n = int(ctl.items().size());
    r.tile = ctl.items().empty() || !ctl.items()[0].element ? 52.0f : ctl.items()[0].element->w;
    r.step = r.tile + 4.5f;
    const float total = float(r.n) * r.step - 4.5f;
    const int f = std::max(0, ctl.focus());
    MenuState& ms = menu_state();
    if (ms.wheel_pin.page == ctl.page_name() && ms.wheel_pin.focus != f) ms.wheel_pin = {}; // moved by key
    if (ms.wheel_pin.page == ctl.page_name()) r.off = ms.wheel_pin.off;
    else if (total <= r.win->w) r.off = (r.win->w - total) * 0.5f;
    else r.off = std::clamp(r.win->w * 0.5f - (float(f) * r.step + r.tile * 0.5f), r.win->w - total, 0.0f);
    return true;
}

namespace {

// The tile under a point (inside the window), or -1.
int wheel_tile_at(const WheelRow& r, float x, float y) {
    if (x < r.win->x || x > r.win->x + r.win->w) return -1;
    for (int i = 0; i < r.n; ++i) {
        SDL_FRect b = r.box(i);
        if (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h) return i;
    }
    return -1;
}

// The garage (or regulated-car) row under a point and its cursor; -1 when none.
int garage_row_at(MenuController& ctl, float x, float y, int*& cursor) {
    GtModeSession* s = S();
    cursor = nullptr;
    if (!s) return -1;
    GarageRows g;
    if (ctl.depth() == 0 && garage_rows(ctl, *s, g)) cursor = &s->garage_cursor;
    else if (regulated_rows(ctl, *s, int(permitted_car_names(s->gt(), s->race_code).size()), g))
        cursor = &s->regulated_cursor;
    if (!cursor || x < g.win->x || x > g.win->x + g.win->w) return -1;
    for (int i = g.first; i < g.n && i < g.first + g.rows; ++i) {
        float ry = g.y(i);
        if (y >= ry && y <= ry + g.tmpl->h) return i;
    }
    return -1;
}

} // namespace

void map_input(MenuController& ctl, menu::flow::MenuKeys held) {
    MenuState& ms = menu_state();
    if (!ms.map.active) {
        ms.map = MapPointer{};
        ms.map.active = true;
        ms.map.target = ctl.focus();
    }
    menu::flow::MenuKeys fired = held & ~ms.map.prev;
    ms.map.prev = held;
    float dx = (fired & key_bit(MenuKey::Right)) ? 1.0f : (fired & key_bit(MenuKey::Left)) ? -1.0f : 0.0f;
    float dy = (fired & key_bit(MenuKey::Up)) ? 1.0f : (fired & key_bit(MenuKey::Down)) ? -1.0f : 0.0f;
    if ((dx != 0 || dy != 0) && ms.map.target >= 0) {
        float cx, cy;
        map_centre(ctl, ms.map.target, cx, cy);
        float dl = std::sqrt(dx * dx + dy * dy), best = 0;
        int pick = -1;
        for (int i = 0; i < int(ctl.items().size()); ++i) {
            if (i == ms.map.target || !ctl.items()[size_t(i)].element) continue;
            float bx, by;
            map_centre(ctl, i, bx, by);
            float vx = bx - cx, vy = by - cy, dist = std::sqrt(vx * vx + vy * vy);
            if (dist < 1e-3f) continue;
            float cosv = (vx * dx + vy * dy) / (dist * dl);
            if (cosv < 0) continue;
            float score = cosv * cosv / dist;
            if (score > best) best = score, pick = i;
        }
        if (pick >= 0) {
            ms.map.target = pick;
            ms.map.moving = true;
            ms.sounds.push_back("cursor");
        }
    }
}

namespace {

// The top page's per-frame code: code-driven lists, sliders, presentation timelines. Returns
// the effect it plays on this frame, or "".
std::string page_frame(MenuController& ctl) {
    GtModeSession* s = S();
    if (!s || !ctl.has_page()) return {};
    const std::string& page = ctl.page_name();
    // Code-driven lists live on base pages; the settings sliders and the permitted cars
    // are popups.
    if (ctl.depth() != 0 && !starts(page, "set/adjast_") && page != "00cmn/car_reguration") return {};
    // Presentations step their timelines on every frame (presentations.cpp); the prize
    // counter's beep plays on the frame of its step.
    if (std::string se; presentation_frame(ctl, *s, page, se)) return se;
    // Wheel shop: the focused wheel is previewed on the car.
    if (starts(page, "at/ws/ws_")) {
        const MenuController::ItemState* it = ctl.focused();
        s->focus_wheel_preview = it && it->flow ? wheel_of_item(it->flow->name) : std::string();
    } else if (!s->focus_wheel_preview.empty()) {
        s->focus_wheel_preview.clear();
    }
    menu::flow::MenuKeys f = ctl.fired();
    int d = (f & key_bit(MenuKey::Down)) ? 1 : (f & key_bit(MenuKey::Up)) ? -1 : 0;
    // Settings sliders: Left/Right one step, L1/R1 ten steps, clamped (no wrap); SE "param"
    // on a change, "disable" at either end or when the change is refused.
    if (starts(page, "set/adjast_"))
        if (const MenuController::ItemState* it = ctl.focused(); it && it->flow && starts(it->flow->name, "slider_")) {
            int step = (f & key_bit(MenuKey::Right)) ? 1 : (f & key_bit(MenuKey::Left)) ? -1
                       : (f & key_bit(MenuKey::R1)) ? 10 : (f & key_bit(MenuKey::L1)) ? -10 : 0;
            GarageCar* real = setting_real();
            GarageCar* work = setting_work();
            if (!step || !real || !work) return {};
            std::optional<SliderRange> r = slider_range(s->gt(), *real, *work, it->flow->name);
            if (!r || !slider_enabled(s->gt(), *real, it->flow->name)) return "disable";
            int v = std::clamp(r->value + step, r->lo(), std::max(r->lo(), r->hi()));
            if (r->fixed() || v == r->value) return "disable";
            return slider_set(s->gt(), *real, *work, it->flow->name, v, s->setting_event) ? "param" : "disable";
        }
    // Dealer car page: Up/Down pick the paint (the page's buttons move with Left/Right).
    if (starts(page, "cd/") && page.size() > 4 && page.compare(page.size() - 4, 4, "_car") == 0) {
        int n = s->gt().data().color_count(s->focus_car);
        if (d && n > 1) {
            s->color_cursor = (s->color_cursor + d + n) % n;
            return "cursor";
        }
        return {};
    }
    if (page == "hm/hm_gr" || page == "hm/hm_gr_trade") {
        // START opens the sort panel.
        if (ctl.pressed() & key_bit(MenuKey::Start)) {
            ctl.popup(page == "hm/hm_gr" ? "hm/hm_gr_panel" : "hm/hm_gr_panel_trade");
            return {};
        }
        int n = int(s->list_gt().state().garage.size());
        if (d && n > 0) {
            int c = std::clamp(s->garage_cursor + d, 0, n - 1);
            if (c != s->garage_cursor) {
                s->garage_cursor = c;
                return "cursor";
            }
        }
    } else if (page == "00cmn/car_reguration") {
        int n = int(permitted_car_names(s->gt(), s->race_code).size());
        if (d && n > 0) {
            int c = std::clamp(s->regulated_cursor + d, 0, n - 1);
            if (c != s->regulated_cursor) {
                s->regulated_cursor = c;
                return "cursor";
            }
        }
    } else if (page == "hm/hm_wheel") {
        int dx = (f & key_bit(MenuKey::Right)) ? 1 : (f & key_bit(MenuKey::Left)) ? -1 : d;
        int n = int(s->gt().state().wheels.size());
        if (dx) {
            int c = std::clamp(s->wheel_cursor + dx, -1, n - 1);
            if (c != s->wheel_cursor) {
                s->wheel_cursor = c;
                return "cursor";
            }
        }
    }
    return {};
}

} // namespace

std::string on_tick(MenuController& ctl) {
    MenuState& ms = menu_state();
    ms.frame = ctl.frame();
    std::string se = step_car_wash(ctl);
    if (std::string oil = step_oil_change(ctl); se.empty()) se = std::move(oil);
    return se;
}

std::string on_frame(MenuController& ctl) {
    MenuState& ms = menu_state();
    // The menus step the GT-mode random generator once per frame (gt_mode §20.2).
    if (GtModeSession* rs = S()) rs->gt().tick_random();
    // (Ours: a page that took the controls away with nothing running to give them back, e.g.
    // one opened directly, gives them back once it has settled.)
    if (ctl.ignoring_control() && !ms.wash.on && !ms.oil_change.on) ctl.give_control();
    ease_map_pointer(ctl);
    // The page's code runs on every frame; a queued effect (the map pointer's) plays on the
    // next frame the page plays none.
    std::string se = page_frame(ctl);
    if (se.empty() && !ms.sounds.empty()) {
        se = ms.sounds.front();
        ms.sounds.erase(ms.sounds.begin());
    }
    return se;
}

int garage_hover(MenuController& ctl, float x, float y) {
    if (WheelRow r; wheel_row(ctl, r)) {
        int i = wheel_tile_at(r, x, y);
        if (i < 0) return 0;
        if (i == ctl.focus()) return 1;
        const float off = r.off;
        ctl.set_focus(ctl.items()[size_t(i)].flow->name);
        menu_state().wheel_pin = {ctl.page_name(), i, off};
        return 2;
    }
    int* cursor = nullptr;
    int i = garage_row_at(ctl, x, y, cursor);
    if (i < 0) return 0;
    if (*cursor == i) return 1;
    *cursor = i;
    return 2;
}

// A tap on a garage row moves the cursor there; a tap on the row under the
// cursor opens the car (confirms the list's item). The permitted-car rows only move
// the cursor.
int garage_tap(MenuController& ctl, float x, float y) {
    if (WheelRow r; wheel_row(ctl, r)) {
        int i = wheel_tile_at(r, x, y);
        if (i < 0) return 0;
        if (i == ctl.focus()) return 2;
        ctl.set_focus(ctl.items()[size_t(i)].flow->name);
        return 1;
    }
    GtModeSession* s = S();
    if (!s) return 0;
    GarageRows g;
    int* cursor = nullptr;
    if (ctl.depth() == 0 && garage_rows(ctl, *s, g)) cursor = &s->garage_cursor;
    else if (regulated_rows(ctl, *s, int(permitted_car_names(s->gt(), s->race_code).size()), g))
        cursor = &s->regulated_cursor;
    if (!cursor || x < g.win->x || x > g.win->x + g.win->w) return 0;
    for (int i = g.first; i < g.n && i < g.first + g.rows; ++i) {
        float ry = g.y(i);
        if (y < ry || y > ry + g.tmpl->h) continue;
        if (i == *cursor) return cursor == &s->garage_cursor ? 2 : 1;
        *cursor = i;
        return 1;
    }
    return 0;
}

} // namespace gt_menu

using namespace gt_menu;

std::vector<std::string> permitted_car_names(const GtMode& gt, const std::string& code) {
    std::vector<std::string> all;
    std::optional<EventRules> r = gt.rules(code);
    if (!r || r->regulation.empty()) return all;
    for (const data::Regulation& g : gt.db().regulations()) {
        if (g.label != r->regulation) continue;
        for (const std::string& label : g.cars) {
            std::string n = car_name(gt, label);
            if (const data::Car* c = gt.db().car(label); c && c->name_alt.find_first_not_of(' ') != std::string::npos)
                n += " " + c->name_alt;
            all.push_back(std::move(n));
        }
        break;
    }
    // A car and its variants share a name: each string once, at its last place.
    std::vector<std::string> out;
    for (size_t i = 0; i < all.size(); ++i)
        if (std::find(all.begin() + std::ptrdiff_t(i) + 1, all.end(), all[i]) == all.end()) out.push_back(all[i]);
    return out;
}

} // namespace gt3r::game
