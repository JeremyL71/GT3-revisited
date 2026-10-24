// GT-mode menu bindings: extra_draws, the code-drawn content of the top page added to its draw
// list: the presentations' prize line, OK button and 3D model, the map pointer, the status bar's
// day, the garage car page's text insets, the series race buttons, the settings sliders' marks,
// the records rows, the Cars Permitted rows, the wheel shop's tiles, the image widgets
// (widget_art.cpp), the dealer's paint column and the garage list.

#include "game/gt_mode_menu/internal.h"

#include "game/gt_setting.h"
#include "menu/menu_renderer.h"
#include "race/machine_test.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace gt3r::game::gt_menu {

namespace {

using DrawList = std::vector<menu::ElementDraw>;
using PageRef = menu::flow::MenuSet::PageRef;

// The prize line ("Prize Cr. <left>", get_money_disp) fades in when its layer
// appears (the trophy's get_mess box) and out after the count.
void fade_prize_line(const std::string& page, DrawList& out) {
    const MenuState& ms = menu_state();
    if ((page == "gtmode_mini" || page == "consolation_prize" || page == "trophy_get") && ms.show) {
        float a = remain_alpha(*ms.show, ms.show_frames, page == "trophy_get" ? kTrophyMoney : 0);
        for (menu::ElementDraw& d : out)
            if (d.element && d.element->widget_class && *d.element->widget_class == "get_money_disp") d.alpha *= a;
    }
}

// Race prizes and the trophy: the OK button appears only once it is accepted (the
// reference captures: none while the money counts, then the round OK; the trophy's OK
// belongs to its mess1 layer).
void hide_ok_until_accepted(const MenuController& ctl, const std::string& page, DrawList& out) {
    const MenuState& ms = menu_state();
    if ((is_mini_prize(page) || page == "trophy_get") && ms.show && ctl.has_page() && !ctl.items().empty() &&
        !enabled(page, *ctl.items()[0].flow)) {
        const Element* ok = ctl.items()[0].element;
        out.erase(std::remove_if(out.begin(), out.end(),
                                 [&](const menu::ElementDraw& d) {
                                     return ok && d.element && (d.element == ok || (!ok->alt.empty() && d.element == &ok->alt[0]));
                                 }),
                  out.end());
    }
}

// GT Mode map: the pointer.
void draw_map_pointer(const MenuController& ctl, const std::string& page, DrawList& out) {
    const MenuState& ms = menu_state();
    if (page == "map" && ctl.depth() == 0) {
        const menu::flow::MenuSet::PageRef ref = ctl.page(); // (a handle: the layout lives in the menu set)
        const menu::Page& lay = ref.layout();
        if (const Element* p = hidden_element(lay, "pointer")) {
            menu::ElementDraw d;
            d.scene = ctl.page().scene;
            d.element = p;
            d.has_box = true;
            float px = ms.map.active ? ms.map.x : 320, py = ms.map.active ? ms.map.y : 224;
            d.box = {px, py - p->h, p->w, p->h}; // the tip (top-left corner) at the point
            d.z = p->z + 1000;
            out.push_back(d);
        }
        // (The hidden "name_<id>" labels are not shown: the original's bar carries only the
        // description ticker, as the reference screens show.)
    }
}

// Prize presentations: the 3D model goes over the page's art (its widget sits
// under the full-page background in the layout's order).
void raise_prize_model(const MenuController& ctl, const std::string& page, DrawList& out) {
    if (page == "license_get" || page == "trophy_get") {
        auto it = std::find_if(out.begin(), out.end(), [](const menu::ElementDraw& d) {
            return d.element && d.element->widget_class && *d.element->widget_class == "render_model";
        });
        menu::ElementDraw d;
        if (it != out.end()) {
            d = *it;
            out.erase(it);
        } else if (const Element* w = find_widget_named(ctl.page().layout(), "model")) {
            // Not in the controller's list (an empty 3 x 3 marker): drawn by us.
            d.scene = ctl.page().scene;
            d.element = w;
            d.widget = true;
            d.z = w->z;
        }
        if (d.element) {
            d.alpha = 1; // the marker itself may be transparent; the model fades by itself
            out.push_back(d);
        }
    }
}

// Status bar: the day counter ("day 12", widget_text) in the unit widgets' own style
// (grey, black shadow), right-aligned. The widget is 24 px wide; the original does not
// squeeze the text into it, and the reference screens end it 10 px before the box's right
// edge (M: maybe the width of the "day %d" template is what is aligned).
void place_status_day(const MenuController& ctl, DrawList& out) {
    if (ctl.page().flow && ctl.page().flow->str44 == "00cmn/status")
        for (menu::ElementDraw& d : out)
            if (d.element && d.element->widget_class && *d.element->widget_class == "user_day_unit") {
                const float right = d.element->x + d.element->w - 10;
                d.has_box = true;
                d.box = {right - 96, d.element->y, 96, d.element->h};
            }
}

// Garage car page: its left / right aligned texts sit 12 units in from their boxes, as
// the original draws them (menus §5.7 "Text inset"; the reference: VALUE, FR, power,
// mileage, colour name all within 2 units). Only the page's own texts (not the status bar).
void inset_garage_car_texts(const PageRef& ref, const std::string& page, DrawList& out) {
    if (starts(page, "hm/hm_car")) {
        const menu::Page& lay = ref.layout();
        auto own = [&](const Element* e) {
            for (const menu::Layer& l : lay.layers) {
                if (!l.elements.empty() && e >= &l.elements.front() && e <= &l.elements.back()) return true;
                if (!l.widgets.empty() && e >= &l.widgets.front() && e <= &l.widgets.back()) return true;
            }
            return false;
        };
        for (menu::ElementDraw& d : out)
            if (d.element && own(d.element)) {
                std::vector<menu::ElementDraw> one{d};
                menu::apply_text_inset(one);
                d.text_dx = one[0].text_dx;
            }
    }
}

// Prize presentation: the reason line ("Championship winner", ...) is one of the
// page's hidden text elements, shown in the "event_message" widget's place.
void draw_prize_reason(const PageRef& ref, const std::string& page, DrawList& out) {
    const MenuState& ms = menu_state();
    if (page == "car_get" && ms.show)
        if (const Element* msg = find_widget_named(ref.layout(), "event_message"))
            if (const Element* e = hidden_element(ref.layout(), ms.show->reason)) {
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = e;
                d.has_box = true;
                d.box = {msg->x, msg->y, msg->w, msg->h};
                d.z = msg->z;
                out.push_back(d);
            }
}

// Series race pages: each race button is a frame that code fills with the
// course logo (hidden element named after the event's course) and, in its
// round slot, the best-result mark (hidden "mark_<place>").
void draw_race_buttons(const MenuController& ctl, GtModeSession& s, const PageRef& ref, const std::string& page,
                       DrawList& out) {
    if (starts(page, "r/") && ref.flow) {
        for (int i = 0; i < int(ctl.items().size()); ++i) {
            const MenuController::ItemState& it = ctl.items()[size_t(i)];
            if (!it.flow || !it.element || it.flow->action != "set_race_code") continue;
            SDL_FRect box, clip;
            if (!ctl.item_box(i, box, &clip)) continue;
            auto put = [&](const Element* el, SDL_FRect b) {
                if (!el) return;
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = el;
                d.has_box = true;
                d.box = b;
                d.has_clip = true;
                d.clip = clip;
                d.z = it.element->z + 0.01f;
                out.push_back(d);
            };
            if (auto rules = s.gt().rules(it.flow->name))
                if (const Element* logo = hidden_element(ref.layout(), rules->course))
                    put(logo, {box.x + 3, box.y + 2, box.h * 1.3f, box.h - 4});
            // The focused race's course name ("<course>_doc") in the page's text bar.
            if (i == ctl.focus())
                if (auto rules = s.gt().rules(it.flow->name))
                    if (const Element* doc = hidden_element(ref.layout(), rules->course + "_doc"))
                        for (const menu::Layer& l : ref.layout().layers)
                            for (const Element& bar : l.elements)
                                if (bar.name == "bar_text1" || bar.name == "bar_text") {
                                    menu::ElementDraw d;
                                    d.scene = ref.scene;
                                    d.element = doc;
                                    d.has_box = true;
                                    d.box = {bar.x, bar.y, bar.w, bar.h};
                                    d.z = bar.z + 0.01f;
                                    out.push_back(d);
                                }
            int best = s.gt().state().best_position(it.flow->name);
            if (best >= 0 && best < 6)
                if (const Element* mark = hidden_element(ref.layout(), "mark_" + std::to_string(best + 1)))
                    put(mark, {box.x + box.w - box.h + 2, box.y + 2, box.h - 4, box.h - 4});
        }
    }
}

// Settings sliders: the value marker ("pinch"; "pinch_fix" for a fixed setting) and
// the value the page opened with ("pinch_prev", until a commit).
void draw_slider_marks(const MenuController& ctl, GtModeSession& s, const PageRef& ref, const std::string& page,
                       DrawList& out) {
    if (starts(page, "set/adjast_"))
        if (GarageCar* real = setting_real(); real && setting_work()) {
            const Element* pinch = hidden_element(ref.layout(), "pinch");
            const Element* prev = hidden_element(ref.layout(), "pinch_prev");
            const Element* fix = hidden_element(ref.layout(), "pinch_fix");
            for (int i = 0; i < int(ctl.items().size()); ++i) {
                const MenuController::ItemState& it = ctl.items()[size_t(i)];
                if (!it.flow || !it.element || !starts(it.flow->name, "slider_")) continue;
                std::optional<SliderRange> r = slider_range(s.gt(), *real, *setting_work(), it.flow->name);
                if (!r) continue;
                const Element& bar = *it.element;
                auto put = [&](const Element* m, int value) {
                    if (!m) return;
                    int n = r->end - r->begin;
                    float x = bar.x;
                    if (n >= 2) x += float(value - r->begin) * (bar.w - m->w) / float(n - 1);
                    menu::ElementDraw d;
                    d.scene = ref.scene;
                    d.element = m;
                    d.has_box = true;
                    d.box = {std::clamp(x, bar.x, bar.x + bar.w - m->w), bar.y + (bar.h - m->h) * 0.5f, m->w, m->h};
                    d.z = bar.z + 0.02f;
                    out.push_back(d);
                };
                bool locked = !slider_enabled(s.gt(), *real, it.flow->name);
                if (r->fixed() || locked) {
                    put(fix, r->begin);
                    continue;
                }
                auto entry = s.setting_entry.find(it.flow->name);
                if (prev && !s.setting_committed && entry != s.setting_entry.end() && entry->second != r->value)
                    put(prev, entry->second);
                put(pinch, r->value);
            }
        }
}

// Records menu: the ranking rows (the "record_bar" template and the hidden
// record_* text elements) in the "menu_records" window (gt_mode §17.2): every place of the
// table (12, a licence test's 6), an empty one with no name and the value and date
// placeholders. The machine tests' and the course time trials' pages also show each entry's car,
// after the car icon.
void draw_records(const MenuController& ctl, GtModeSession& s, const PageRef& ref, const std::string& page,
                  DrawList& out) {
    if (page == "records_free_tt" && !ctl.items().empty() && ctl.items()[0].element) {
        const Element& bar = *ctl.items()[0].element;
        const Element* win = find_widget_named(ref.layout(), "menu_records");
        if (win) {
            const GtMode& gt = s.gt();
            const bool speed = s.records_key == "mt:MMAX";
            const bool with_car = starts(s.records_key, "mt:") || starts(s.records_key, "ct:");
            const size_t places = starts(s.records_key, "lic:") ? 6 : 12;
            const std::vector<RankEntry> table = gt.ranking_top(s.records_key, places);
            const int rows = std::max(1, int(win->h / bar.h));
            for (int i = 0; i < int(places) && i < rows; ++i) {
                const RankEntry* r = size_t(i) < table.size() ? &table[size_t(i)] : nullptr;
                const float y = win->y + win->h - float(i + 1) * bar.h;
                auto put = [&](const Element* el, std::string text) {
                    if (!el) return;
                    menu::ElementDraw d;
                    d.scene = ref.scene;
                    d.element = el;
                    d.has_box = true;
                    // Left-aligned texts get room to the right; centred ones (the rank) keep their box.
                    bool left = el->text && (el->text->align & 0x0F) % 3 == 0;
                    d.box = {el->x, y + (el->y - bar.y), left ? std::max(el->w, 200.0f) : el->w, el->h};
                    d.z = el->z;
                    if (!text.empty() || el->text) d.text = std::make_shared<const std::string>(std::move(text));
                    out.push_back(d);
                };
                menu::ElementDraw b;
                b.scene = ref.scene;
                b.element = &bar;
                b.has_box = true;
                b.box = {bar.x, y, bar.w, bar.h};
                b.z = bar.z;
                out.push_back(b);
                const std::string data = speed ? race::record_speed_text(r ? r->value : 0, gt.data().imperial)
                                         : r   ? format_time_ms(u32(std::llround(r->value * 1000)))
                                               : std::string("--'--.---");
                put(hidden_element(ref.layout(), "record_rank_shadow"), std::to_string(i + 1));
                put(hidden_element(ref.layout(), "record_rank"), std::to_string(i + 1));
                put(hidden_element(ref.layout(), "record_name"), r ? gt.ranking_name(*r) : std::string());
                if (with_car) {
                    // "<name> <second name>" (the car's two name strings), nothing without a car.
                    std::string car;
                    if (r && !r->car.empty()) {
                        const data::Car* c = gt.db().car(r->car);
                        car = c ? c->name + " " + c->name_alt : r->car;
                    }
                    put(hidden_element(ref.layout(), "record_car_pict"), std::string());
                    put(hidden_element(ref.layout(), "record_car"), car);
                }
                put(hidden_element(ref.layout(), "record_data"), data);
                put(hidden_element(ref.layout(), "record_date"), record_date_text(r ? r->stamp : 0));
            }
        }
    }
}

// Race information -> Cars Permitted: one row per permitted car, the "link_menu" template
// ("active_menu" on the cursor row) in the "menu_regulated_car" window; without a list
// the hidden "No Limitations" (in its own place, the middle of the box: M).
void draw_permitted_cars(const MenuController& ctl, GtModeSession& s, const PageRef& ref,
                         const std::string& page, DrawList& out) {
    if (page == "00cmn/car_reguration") {
        const std::vector<std::string> names = permitted_car_names(s.gt(), s.race_code);
        GarageRows g;
        if (names.empty()) {
            if (const Element* none = hidden_element(ref.layout(), "no_regulated_car")) {
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = none;
                d.z = none->z;
                out.push_back(d);
            }
        } else if (regulated_rows(ctl, s, int(names.size()), g)) {
            for (int i = g.first; i < g.n && i < g.first + g.rows; ++i) {
                const Element* e = i == s.regulated_cursor && !g.tmpl->alt.empty() ? &g.tmpl->alt[0] : g.tmpl;
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = e;
                d.has_box = true;
                d.box = {g.win->x, g.y(i), g.tmpl->w, g.tmpl->h};
                d.z = g.tmpl->z;
                d.text = std::make_shared<const std::string>(names[size_t(i)]);
                d.text_dx = menu::kTextInset; // left-aligned text 12 units in (menus §5.7 "Text inset")
                out.push_back(d);
            }
        }
    }
}

// Wheel shop: the brand page's wheels as tiles in a row in the "menu_wheel_shop" window
// (WheelShopButton: each tile is the page's hidden image named after the wheel, 52 x 52,
// with "sold_out" over it for a wheel already owned; the reference: tiles 56.5 units
// apart, centred while they fit, the focused one washed light grey).
void draw_wheel_tiles(const MenuController& ctl, GtModeSession& s, const PageRef& ref, DrawList& out) {
    if (WheelRow row; wheel_row(ctl, row)) {
        const Element* win = row.win;
        const Element* sold = hidden_element(ref.layout(), "sold_out");
        for (int i = 0; i < row.n; ++i) {
            const MenuController::ItemState& it = ctl.items()[size_t(i)];
            if (!it.element || !it.flow) continue;
            const bool focus = i == ctl.focus();
            const SDL_FRect box = row.box(i);
            if (box.x + box.w < win->x || box.x > win->x + win->w) continue;
            const std::string code = wheel_of_item(it.flow->name);
            auto image = [&](const Element* e, float z) {
                if (!e) return;
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = e;
                d.has_box = true;
                d.box = box;
                d.z = z;
                out.push_back(d);
            };
            menu::ElementDraw sh; // a soft drop shadow down and right (reference)
            sh.kind = menu::ElementDraw::Kind::Fill;
            sh.has_box = true;
            sh.box = {box.x + 3, box.y - 3, box.w, box.h}; // (layout y grows upwards)
            sh.fill = {0, 0, 0, 0.5f};
            sh.z = win->z - 0.001f;
            out.push_back(sh);
            image(hidden_element(ref.layout(), code), win->z);
            if (focus) {
                menu::ElementDraw w;
                w.kind = menu::ElementDraw::Kind::Fill;
                w.has_box = true;
                w.box = box;
                w.fill = {1, 1, 1, 0.38f}; // measured: dark pixels -> ~98 / 255
                w.z = win->z + 0.001f;
                out.push_back(w);
            }
            const std::vector<std::string>& inv = s.gt().state().wheels;
            if (!code.empty() && std::find(inv.begin(), inv.end(), code) != inv.end()) image(sold, win->z + 0.002f);
        }
    }
}

// Dealer car page: one swatch per paint of the model in a column ending one step above
// the "car_color_box" widget, the first on top, 12.5 px apart, each over a soft drop
// shadow (+3, +3); the chosen one is marked by a white 8 x 10 square 4 px to its left
// (the reference screens, measured).
void draw_paint_column(GtModeSession& s, const PageRef& ref, const std::string& page, DrawList& out) {
    if (starts(page, "cd/") && page.size() > 4 && page.compare(page.size() - 4, 4, "_car") == 0) {
        const Element* box = find_widget_named(ref.layout(), "car_color_box");
        const GtData& data = s.gt().data();
        int n = data.color_count(s.focus_car);
        if (!box || n <= 0) return;
        const float step = box->h * 1.25f;
        auto fill = [&](SDL_FRect r, float g, float a, float z) {
            menu::ElementDraw f;
            f.kind = menu::ElementDraw::Kind::Fill;
            f.has_box = true;
            f.box = r;
            f.fill = {g, g, g, a};
            f.z = z;
            out.push_back(f);
        };
        for (int i = 0; i < n; ++i) {
            u32 rgb = data.color_rgb(data.color_id(s.focus_car, i));
            // (Layout coordinates: y grows upwards.)
            float x = box->x, y = box->y + float(n - i) * step; // first paint on top
            fill({x + 2, y - 4, box->w + 2, box->h + 2}, 0, 0.35f, box->z - 0.002f);
            fill({x + 3, y - 3, box->w, box->h}, 0, 0.75f, box->z - 0.001f);
            if (i == s.color_cursor) fill({x - 12, y, 8, box->h}, 1, 1, box->z);
            menu::ElementDraw d;
            d.kind = menu::ElementDraw::Kind::Fill;
            d.has_box = true;
            d.box = {x, y, box->w, box->h};
            d.fill = {float(rgb & 0xFF) / 255.0f, float((rgb >> 8) & 0xFF) / 255.0f,
                      float((rgb >> 16) & 0xFF) / 255.0f, 1};
            d.z = box->z;
            out.push_back(d);
        }
    }
}

// Garage list (code-drawn): rows of the "car1" template in the "menu_garage"
// window, in the chosen sort order, with the sort column header and values.
void draw_garage_list(const MenuController& ctl, GtModeSession& s, const PageRef& ref, const std::string& page,
                      DrawList& out) {
    if ((page != "hm/hm_gr" && page != "hm/hm_gr_trade") || ctl.depth() != 0) return;
    if (ctl.items().empty() || !ctl.items()[0].element) return;
    const Element& tmpl = *ctl.items()[0].element;
    const Element* win = find_widget_named(ref.layout(), page == "hm/hm_gr_trade" ? "menu_garage_trade" : "menu_garage");
    if (!win) return;
    // Date / name order show the MAKE column (as the original's new-game garage does).
    static const char* const kHeader[] = {"maker", "maker", "maker", "color", "price", "distance", "power", "drive", "aspiration"};
    const int key = std::clamp(s.sort_key, 0, 8);
    if (*kHeader[key])
        if (const Element* h = hidden_element(ref.layout(), kHeader[key])) {
            menu::ElementDraw d;
            d.scene = ref.scene;
            d.element = h;
            d.z = h->z;
            out.push_back(d);
        }
    const Element* info = hidden_element(ref.layout(), "info");
    const Element* info_a = hidden_element(ref.layout(), "info_a");
    GtMode& lg = s.list_gt(); // the guest garage during a trade
    const GameState& st = lg.state();
    const std::vector<int> order = lg.garage_order(s.sort_key);
    const float row = tmpl.h + 2;
    const int rows = std::max(1, int(win->h / row));
    const int n = int(order.size());
    int first = std::clamp(s.garage_cursor - rows / 2, 0, std::max(0, n - rows));
    for (int i = first; i < n && i < first + rows; ++i) {
        const GarageCar& car = st.garage[size_t(order[size_t(i)])];
        const bool focus = i == s.garage_cursor;
        const Element* e = focus && !tmpl.alt.empty() ? &tmpl.alt[0] : &tmpl;
        const float y = win->y + win->h - float(i - first + 1) * row;
        menu::ElementDraw d;
        d.scene = ref.scene;
        d.element = e;
        d.has_box = true;
        d.box = {win->x, y, tmpl.w, tmpl.h};
        d.z = tmpl.z;
        d.text = std::make_shared<const std::string>(car_name(lg, car.car));
        out.push_back(d);
        // The ride car: the hidden "mark" (orange car) at the right of the name column.
        if (!s.trade && order[size_t(i)] == st.ride)
            if (const Element* mk = hidden_element(ref.layout(), "mark")) {
                menu::ElementDraw m;
                m.scene = ref.scene;
                m.element = mk;
                m.has_box = true;
                m.box = {412, y + (tmpl.h - mk->h) * 0.5f, mk->w, mk->h}; // place from the reference (M)
                m.z = tmpl.z + 0.02f;
                out.push_back(m);
            }
        const Element* ie = focus && info_a ? info_a : info;
        std::string v = lg.sort_value(car, s.sort_key < 2 ? 2 : s.sort_key);
        for (char& ch : v)
            if (s.sort_key < 3) ch = char(std::toupper(static_cast<unsigned char>(ch))); // "MAZDA"
        if (ie && !v.empty()) {
            menu::ElementDraw t;
            t.scene = ref.scene;
            t.element = ie;
            t.has_box = true;
            t.box = {ie->x, y, ie->w, tmpl.h};
            t.z = tmpl.z + 0.01f;
            t.text = std::make_shared<const std::string>(v);
            out.push_back(t);
        }
    }
}

} // namespace

void extra_draws(const MenuController& ctl, std::vector<menu::ElementDraw>& out) {
    GtModeSession* s = S();
    if (!s || !ctl.has_page()) return;
    const std::string& page = ctl.page_name();
    oil_gauge_draws(ctl, out); // first: it goes in at its depth while the list is still sorted
    fade_prize_line(page, out);
    hide_ok_until_accepted(ctl, page, out);
    draw_map_pointer(ctl, page, out);
    raise_prize_model(ctl, page, out);
    place_status_day(ctl, out);
    menu::flow::MenuSet::PageRef ref = ctl.page();
    if (!ref) return;
    inset_garage_car_texts(ref, page, out);
    draw_prize_reason(ref, page, out);
    draw_race_buttons(ctl, *s, ref, page, out);
    draw_slider_marks(ctl, *s, ref, page, out);
    draw_records(ctl, *s, ref, page, out);
    draw_permitted_cars(ctl, *s, ref, page, out);
    draw_wheel_tiles(ctl, *s, ref, out);
    // Image widgets of the layers currently shown.
    for (size_t li = 0; li < ref.layout().layers.size(); ++li) {
        if (!ctl.layer_visible(int(li))) continue;
        for (const Element& w : ref.layout().layers[li].widgets) widget_draws(ctl, w, out);
    }
    draw_paint_column(*s, ref, page, out);
    draw_garage_list(ctl, *s, ref, page, out);
}

} // namespace gt3r::game::gt_menu
