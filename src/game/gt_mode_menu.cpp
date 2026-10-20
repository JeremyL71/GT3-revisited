#include "game/gt_mode_menu.h"
#include "sim/car_spec.h"

#include "base/file.h"
#include "base/log.h"
#include "game/gt_save.h"
#include "game/gt_setting.h"
#include "game/save_library.h"
#include "menu/flow/gt_mode_hooks.h"
#include "menu/menu_renderer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

namespace gt3r::game {

namespace {

using menu::Element;
using menu::MblItem;
using menu::flow::GameRequest;
using menu::flow::GameResult;
using menu::flow::MenuController;
using menu::flow::MenuEvent;
using menu::flow::MenuKey;
using menu::flow::key_bit;

std::shared_ptr<GtData> g_data;
GtData::Reader g_reader; // disc files (the PS2 export's icons)
std::unique_ptr<GtModeSession> g_session;
std::optional<GameRequest> g_last_request;
std::string g_free_run_course;
bool g_round_pending = false; // the last result was a championship round with more to come

// Presentations queued by a result (car_get / license_get pages).
struct Show {
    std::string set, page;
    std::vector<std::string> layers;
    std::string reason;                  // car_get: hidden text element naming why ("championship_win")
    std::string car;                     // car_get: the car received
    int paint = 0;
    std::vector<std::string> candidates; // car_get: the set's cars (roulette)
    std::string record_key;              // name_entry: the ranking and the new entry's rank
    int record_rank = -1;
    std::string model;                   // license_get / trophy_get: the prize model (.prm path)
    s64 money = 0;                       // trophy_get: the championship bonus counted up
    std::string rank;                    // arcade_mini / gtmode_mini / consolation_prize: "1st", "4th", ...
    bool credited = true;                // false: `money` is not in the credits yet (shown from the race)
    // The prize counter (race prizes gtmode_mini / consolation_prize, the championship trophy):
    // the balance counting from `from` to `from + money` (presentation.md §F).
    struct Counter {
        s64 cur = 0, target = 0;
        double frac = 0, v = 0, a = 0, mult = 1, beep = 0;
        int state = 1, n = 0, wait = 0, presses = 0, beep_n = 0;
        int delay = 120;      // frames before the count starts (trophy 360)
        int done_frame = -1;  // presentation frame the count finished on
        bool started = false;
    } counter;
};
std::vector<Show> g_shows;
std::vector<std::string> g_sounds; // effects to play on the next frames (one per frame)
std::string g_popup;               // popup to open after a consumed Finish (load_warning)
std::string g_movie;               // ending movie to play after the result screens
std::optional<Show> g_show; // being shown
int g_show_frames = 0;
int g_show_serial = 0;      // bumped for every presentation shown
// Presentation timelines (frames at 60 Hz, spec §18): when the model fades in.
constexpr int kLicFadeIn = 120, kLicFadeEnd = 180, kTrophyFadeIn = 180, kTrophyFadeEnd = 270;
constexpr int kLicMess0End = 240, kLicMess1 = 360, kLicOk = 420;
// Trophy (spec gt_mode.md §18): mess0 for 180 frames, 90 more, then get_mess;
// after 60 frames it waits for the prize counter (its count starts 360 frames after the page
// opened); 15 frames after the count, mess1 and OK.
constexpr int kTrophyMess0End = 180, kTrophyMoney = 270, kTrophyCountDelay = 360, kTrophyOkAfter = 15;
// Race prizes (presentation.md §F): the arcade cup accepts OK after 180 frames; the GT-mode cup and
// the consolation prize once the prize-money counter (below) has finished.
constexpr int kMiniFadeFrames = 30, kArcadeMiniOk = 180;
// The prize counter (presentation.md §F): 120 frames' wait (360 on
// the trophy page; counted from the page's first frame, the first step two frames later), then
// state 1 creeps by 0.2380952 per frame for 90 frames, then state 2 accelerates (a += 5.2720418
// per frame up to 474.48376, v += a up to 142345.125), step = v x speed, the whole part added
// to the balance; done (state 3) at the target. Confirm while counting: the first press triples
// the speed, the next one completes the count. The counting beeps (SE "cursor") when the
// summed steps pass 1 at least 3 frames after the last beep.
constexpr int kCounterCreep = 90;
void step_counter(Show::Counter& c, bool confirm, std::vector<std::string>* sounds) {
    if (c.state == 3) return;
    if (c.wait < c.delay + 2) {
        ++c.wait;
        return;
    }
    if (confirm) {
        if (c.presses == 0) c.mult = 3.0;
        else c.cur = c.target;
        ++c.presses;
    }
    ++c.n;
    double add = 0;
    if (c.state == 1) {
        c.v = 0.2380952388048172;
        if (double(c.n) > double(kCounterCreep)) c.state = 2;
    } else if (c.state == 2) {
        add = 5.2720417976379395;
        if (c.a >= 474.4837646484375) c.a = 474.4837646484375, add = 0;
        if (c.v >= 142345.125) c.v = 142345.125, add = 0;
    }
    c.a += add;
    double step = (c.v + c.a) * c.mult;
    c.v = step / c.mult; // (v' = v + a; the stored velocity is the unscaled one)
    c.frac += step;
    s64 whole = s64(c.frac);
    c.frac -= double(whole);
    c.cur += whole;
    if (c.cur >= c.target) {
        c.cur = c.target;
        c.state = 3;
        return;
    }
    c.beep += step;
    ++c.beep_n;
    if (c.beep >= 1.0) {
        if (c.beep_n >= 3) {
            c.beep = 0, c.beep_n = 0;
            if (sounds) sounds->push_back("cursor");
        } else {
            c.beep = 1.0;
        }
    }
}
bool counter_done(const Show& sh) { return sh.counter.state == 3; }
// Trophy: OK (and mess1) 15 frames after the count, and not before the bonus box's 60 frames.
bool trophy_ok() {
    return g_show && g_show->counter.done_frame >= 0 &&
           g_show_frames >= std::max(g_show->counter.done_frame, kTrophyMoney + 60) + kTrophyOkAfter;
}
// The remaining-prize line ("Prize Cr. <left>", presentation.md §F): fades in over 60 frames from
// `shown` (when its layer appears), stays 61 frames after the count is done, then fades out
// over 60 frames.
float remain_alpha(const Show& sh, int frame, int shown) {
    float a = std::clamp(float(frame - shown) / 60.0f, 0.0f, 1.0f);
    if (sh.counter.done_frame >= 0) a = std::min(a, std::clamp(1.0f - float(frame - sh.counter.done_frame - 61) / 60.0f, 0.0f, 1.0f));
    return a;
}
// Starts / steps the presentation's prize counter (once per frame).
void run_counter(Show& sh, const GameState& st, bool confirm, std::vector<std::string>* sounds) {
    Show::Counter& k = sh.counter;
    if (!k.started) {
        k.cur = sh.credited ? st.credits - sh.money : st.credits;
        k.target = k.cur + sh.money;
        k.started = true;
    }
    step_counter(k, confirm && k.state != 3, sounds);
    if (k.state == 3 && k.done_frame < 0) k.done_frame = g_show_frames;
}
bool is_mini_prize(const std::string& page) {
    return page == "arcade_mini" || page == "gtmode_mini" || page == "consolation_prize";
}
std::string ordinal(int place) { // 0-based
    static const char* const kSuffix[] = {"th", "st", "nd", "rd"};
    int n = place + 1, m = n % 100;
    return std::to_string(n) + ((m >= 11 && m <= 13) || n % 10 > 3 ? kSuffix[0] : kSuffix[n % 10]);
}

bool starts(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }

std::string cr(s64 v) { return format_credits(v) + " Cr."; } // log lines
// Menu money texts (as the reference screens): "Cr. 14,660" on the dealer pages, the plain
// number next to a Cr. icon elsewhere.
std::string cr_pre(s64 v) { return "Cr. " + format_credits(v); }
std::string cr_num(s64 v) { return format_credits(v); }

std::string car_name(const GtMode& gt, const std::string& label) {
    const data::Car* c = gt.db().car(label);
    if (!c) return label;
    return c->name.empty() ? label : c->name;
}

// Item name -> goods for the price widgets ("price_tbk_1") and shop items.
int goods_of_item(const std::string& item) {
    const ShopItem* s = shop_item(item);
    return s ? s->goods : -1;
}

// "bbw001_s_link" -> "bbw001_s" (the original cuts the last "_..." part).
std::string wheel_of_item(const std::string& item) {
    size_t p = item.rfind('_');
    return p == std::string::npos ? item : item.substr(0, p);
}

std::string popup_for(Check c, int license = -1) {
    if (c == Check::NoLicense && license >= 0) return std::string("lc/no_license_") + license_menu_id(License(license));
    if (c == Check::AlreadyHave) return "ts/already_you_have";
    return c == Check::Ok || c == Check::Unknown ? std::string() : std::string(check_name(c));
}

GtModeSession* S() { return g_session.get(); }

// Car settings menu: the committed car (the ride car) and the working copy.
// The ranking entry the name entry page edits (nullptr when none).
RankEntry* named_entry() {
    if (!S() || !g_show || g_show->record_key.empty()) return nullptr;
    auto& r = S()->gt().state().rankings;
    auto it = r.find(g_show->record_key);
    if (it == r.end() || g_show->record_rank < 0 || g_show->record_rank >= int(it->second.size())) return nullptr;
    return &it->second[size_t(g_show->record_rank)];
}

GarageCar* setting_real() { return S() ? S()->gt().state().ride_car() : nullptr; }
GarageCar* setting_work() {
    GtModeSession* s = S();
    GarageCar* real = setting_real();
    if (!s || !real) return nullptr;
    if (!s->setting_work) s->setting_work = *real;
    return &*s->setting_work;
}
bool is_setting_page(const std::string& page) { return starts(page, "set/") || starts(page, "chg/"); }

// The car_get reason text (a hidden text element of that page) for a prize set name.
std::string prize_reason(const std::string& set) {
    if (int lic = GtMode::license_index_of(set); lic >= 0) {
        static const char* const n[] = {"b", "a", "ib", "ia", "s", "r"};
        return std::string("license_") + n[lic] + "_all_gold";
    }
    if (set == "gtmode_amateur_complete") return "gtmode_ama_complete";
    if (set == "achieve_half") return "achieve50";
    if (set == "achieve_3_quoter") return "achieve75";
    if (set == "achieve_complete") return "achieve100";
    if (set.rfind("gtmode_", 0) == 0) return set;
    if (set.rfind("E", 0) == 0) return "enduro_win";
    if (set.size() > 5 && set.compare(set.size() - 5, 5, "_0000") == 0) return "solo_race_clear";
    return "championship_win";
}

bool saving_enabled() {
    const char* p = std::getenv("GT3R_GTMODE_NOSAVE");
    return !(p && *p && *p != '0');
}

std::string save_path() { return current_save_path(); }

void autosave(std::vector<std::string>* log) {
    if (!S() || !saving_enabled()) return;
    std::string err;
    if (!S()->save(&err)) {
        LOG_WARN("gt mode: save failed: %s", err.c_str());
        if (log) log->push_back("save failed: " + err);
    }
}

// ---------------------------------------------------------------------------
// Pre-checks: a popup page instead of following the item (spec §3 / §6.2).

std::string intercept(const std::string& page, const MblItem& it) {
    GtModeSession* s = S();
    if (!s) return {};
    // Trade: buying the guest car is refused for prize-only models, without the money,
    // with a full garage.
    if (page == "hm/hm_car_trade" && it.link == "hm/hm_buy_sure") {
        const GameState& gs = s->list_gt().state();
        if (!s->trade || s->garage_focus < 0 || s->garage_focus >= int(gs.garage.size())) return "00cmn/not_for_buy";
        switch (s->gt().can_buy_guest_car(gs.garage[size_t(s->garage_focus)])) {
        case Check::Ok: return {};
        case Check::NoMoney: return "00cmn/no_money";
        case Check::MaxCar: return "00cmn/max_car";
        default: return "00cmn/not_for_buy";
        }
    }
    if (starts(page, "ar/")) {
        // Clear Status "Goal": the endings need a complete GT league or every arcade
        // course won at one difficulty.
        if (page == "ar/goodies" && it.link == "ar/select_ending" && !s->gt().arcade_ending_open())
            return "ar/cannot_ending";
        return {};
    }
    if (is_setting_page(page) || page == "change_parts") {
        GarageCar* real = setting_real();
        if (!real) return {};
        // Part change: refused when the event's rules would be broken.
        if (int goods = setting_goods(it.action); goods >= 0 && !s->setting_event.empty()) {
            GarageCar copy = *real;
            s->gt().fit_goods(copy, goods);
            if (!s->gt().check_entry(s->setting_event, copy).ok()) return "not_to_change";
        }
        // Tyres cannot be changed for a dirt event.
        if ((it.link == "chg/change_tir_front" || it.link == "chg/change_tir_rear") && !s->setting_event.empty())
            if (auto r = s->gt().rules(s->setting_event); r && r->dirt_tires) return "dirt_tire_only";
        return {};
    }
    GtMode& gt = s->gt();
    const std::string& link = it.link;
    const std::string& act = it.action;
    if (link == "00cmn/cd_buy_sure") return popup_for(gt.can_buy_car(s->focus_car));
    if (link == "hm/hm_sell_sure") return popup_for(gt.can_sell(s->garage_focus));
    if (link == "at/at" || link == "ts/ts" || link == "ts/ts_bs" || link == "ts/ts_dr" || link == "ts/ts_eg" ||
        link == "ts/ts_ot" || link == "ts/ts_tr" || link == "ts/ts_tb" || link == "mt/mt" || link == "hm/hm_fr" ||
        act == "go_machine_test")
        return gt.state().ride_car() ? std::string() : popup_for(Check::NoRideCar);
    if (link == "at/ws/ws") {
        if (!gt.state().ride_car()) return popup_for(Check::NoRideCar);
        return gt.car_flags(gt.state().ride_car()->car) & 4 ? popup_for(Check::NotForChangeWheel) : std::string();
    }
    if (link == "hm/hm_wheel") {
        if (s->garage_focus < 0 || s->garage_focus >= int(gt.state().garage.size())) return {};
        return gt.car_flags(gt.state().garage[size_t(s->garage_focus)].car) & 4 ? popup_for(Check::NotForChangeWheel)
                                                                                 : std::string();
    }
    if (starts(link, "ts/ts_buy_sure_all")) return popup_for(gt.can_buy_all_tires());
    if (starts(link, "ts/ts_buy_sure")) {
        int g = goods_of_item(it.name);
        if (g < 0) g = s->focus_goods;
        return popup_for(gt.can_buy_part(g));
    }
    if (link == "00cmn/ws_buy_sure") {
        Check c = gt.can_buy_wheel(wheel_of_item(it.name));
        return c == Check::AlreadyHave ? std::string("00cmn/already_you_have") : popup_for(c);
    }
    if (act == "go_free_run") {
        if (!gt.state().ride_car()) return popup_for(Check::NoRideCar);
        for (const data::Course& c : gt.db().courses())
            if (c.label == it.name && c.surface == 2 && !gt.state().ride_car()->owned.test(66))
                return popup_for(Check::NeedDirtTire);
        return {};
    }
    if (act == "go_race") return popup_for(gt.can_enter(s->race_code));
    if (act == "go_license") return popup_for(gt.can_take_test(it.name));
    if (starts(link, "lc/lc_")) {
        int lic = int(license_from_menu_id(link.substr(6)));
        return popup_for(gt.can_open_license_page(lic), lic);
    }
    if (act == "oil_change") return popup_for(gt.can_change_oil());
    if (act == "wash_car") return popup_for(gt.can_wash());
    if (act == "check_ride_car" && gt.state().garage.size() > 1 && page == "00cmn/cd_bought")
        return "00cmn/cd_ride_sure";
    (void)page;
    return {};
}

bool enabled(const std::string& page, const MblItem& it) {
    GtModeSession* s = S();
    if (!s) return true;
    // Presentations: OK is accepted once the timeline is over.
    if (page == "license_get") return g_show_frames >= kLicOk;
    if (page == "trophy_get") return trophy_ok();
    if (page == "arcade_mini") return g_show_frames >= kArcadeMiniOk;
    if (page == "gtmode_mini" || page == "consolation_prize") return g_show && counter_done(*g_show);
    if (starts(page, "ar/")) {
        // The long ending: a complete amateur / pro league or all courses won at
        // Normal, Hard or Professional.
        if (page == "ar/select_ending" && it.action == "go_ending_normal") return s->gt().arcade_long_ending_open();
        return true;
    }
    if (starts(page, "set/adjast_")) {
        // Status boxes ("status_box_sus3" = the 3rd setting of the page) open only when unlocked.
        GarageCar* real = setting_real();
        size_t d = it.name.find_last_not_of("0123456789");
        if (!real || !starts(it.name, "status_box_") || d + 1 >= it.name.size()) return true;
        return setting_page_item_enabled(s->gt(), *real, page, std::atoi(it.name.c_str() + d + 1) - 1);
    }
    if (starts(page, "chg/")) {
        GarageCar* real = setting_real();
        int goods = setting_owned_goods(it.action);
        return !real || goods < 0 || real->owned.test(size_t(goods));
    }
    if (starts(page, "r/") && starts(it.name, "G")) return s->gt().series_unlocked(it.name);
    return true;
}

// ---------------------------------------------------------------------------
// Actions (the original GT mode menu's actions).

bool on_event(const MenuEvent& e, std::vector<std::string>& log) {
    GtModeSession* s = S();
    if (!s) return false;
    GtMode& gt = s->gt();
    using K = MenuEvent::Kind;
    if (e.kind == K::PageChanged) {
        s->last_page = e.page;
        // The trade session: the guest garage's list and car pages.
        if (e.page == "hm/hm_gr_trade") {
            if (!s->trade) s->garage_cursor = 0;
            s->trade = s->guest != nullptr;
        } else if (e.page == "hm/hm" || e.page == "hm/hm_gr" || e.page == "map") {
            if (s->trade) s->garage_cursor = 0, s->garage_focus = -1;
            s->trade = false;
        }
        if (e.page == "00cmn/car_reguration") s->regulated_cursor = 0;
        if (starts(e.page, "set/adjast_")) {
            // Slider markers: the value each slider had when the page opened.
            s->setting_entry.clear();
            s->setting_committed = false;
            if (GarageCar* work = setting_work())
                for (const char* item : {"slider_rate_f", "slider_rate_r", "slider_height_f", "slider_height_r",
                                         "slider_damper_f", "slider_damper_r", "slider_bound_f", "slider_bound_r",
                                         "slider_rebound_f", "slider_rebound_r", "slider_camber_f", "slider_camber_r",
                                         "slider_toe_f", "slider_toe_r", "slider_stabilizer_f", "slider_stabilizer_r",
                                         "slider_brake_f", "slider_brake_r", "slider_lsd_init_f", "slider_lsd_init_r",
                                         "slider_lsd_accl_f", "slider_lsd_accl_r", "slider_lsd_decl_f",
                                         "slider_lsd_decl_r", "slider_gear_ratio_1", "slider_gear_ratio_2",
                                         "slider_gear_ratio_3", "slider_gear_ratio_4", "slider_gear_ratio_5",
                                         "slider_gear_ratio_6", "slider_gear_ratio_7", "slider_gear_ratio_final",
                                         "slider_gear_auto_level", "slider_aero_f", "slider_aero_r",
                                         "slider_elec_aycc", "slider_elec_ascc", "slider_elec_tcsc",
                                         "slider_elec_vcdc"})
                    if (auto r = slider_range(s->gt(), *setting_real(), *work, item)) s->setting_entry[item] = r->value;
        }
        return false;
    }
    if (e.kind == K::Action && e.action == "input_text" && e.page == "name_entry") {
        // Name entry: append the character of the chosen key (at most 10).
        if (RankEntry* r = named_entry()) {
            auto ch = gt.data().name_chars.find(e.item);
            std::string c = ch != gt.data().name_chars.end() && !ch->second.empty() ? ch->second : " ";
            // The original: the box starts with the last name entered, keys append (at most 15).
            s->name_started = true;
            if (r->name.size() < 15) r->name += c;
            gt.state().player_name = r->name;
        }
        return false;
    }
    if (e.kind == K::BackAction && e.action == "retrieve_slider") {
        // Back from a slider page: every uncommitted change is discarded.
        if (GarageCar* real = setting_real()) s->setting_work = *real;
        return false;
    }
    if (e.kind == K::Action && e.action == "commit_slider") {
        if (GarageCar* real = setting_real(); real && s->setting_work) {
            *real = *s->setting_work;
            s->setting_committed = true;
            autosave(&log);
        }
        return false;
    }
    if (e.kind == K::Action && setting_goods(e.action) >= 0 && is_setting_page(e.page)) {
        // Part change: fit the owned part at once (no OK needed).
        GarageCar* real = setting_real();
        int goods = setting_goods(e.action);
        if (real && real->owned.test(size_t(setting_owned_goods(e.action)))) {
            gt.fit_goods(*real, goods);
            s->setting_work = *real;
            autosave(&log);
        }
        return false;
    }
    if (e.kind == K::Action) {
        const std::string& a = e.action;
        if (a == "set_car_code") {
            const DealerCar* d = gt.data().catalog.dealer_car(e.item);
            s->focus_car = d ? d->car : e.item;
            s->color_cursor = 0; // the paint resets when the focus car changes
        } else if (a == "clear_focus_car") {
            s->focus_car.clear();
        } else if (a == "buy_car") {
            int i = gt.buy_car(s->focus_car, s->color_cursor);
            if (i >= 0) {
                log.push_back("bought " + car_name(gt, s->focus_car) + " for " + cr(gt.car_price(s->focus_car)));
                autosave(&log);
            }
        } else if (a == "ride_car") {
            // From the dealer ("ride the new car?"): the new car is at the head of the list.
            int i = e.page == "00cmn/cd_ride_sure" ? 0 : s->garage_focus;
            if (e.item != "no1") gt.ride(i);
        } else if (a == "sell_car") {
            s64 v = gt.sell_car(s->garage_focus);
            if (v > 0) log.push_back("sold for " + cr(v));
            s->garage_cursor = std::min(s->garage_cursor, std::max(0, int(gt.state().garage.size()) - 1));
            s->garage_focus = -1;
            autosave(&log);
        } else if (a == "buy_guest_car") {
            // Trade: a copy of the friend's car (the friend keeps it), for the catalogue price.
            const GameState& gs = s->list_gt().state();
            if (s->trade && s->garage_focus >= 0 && s->garage_focus < int(gs.garage.size())) {
                const GarageCar c = gs.garage[size_t(s->garage_focus)];
                if (gt.buy_guest_car(c) >= 0) {
                    log.push_back("bought " + car_name(gt, c.car) + " for " + cr(gt.car_price(c.car)));
                    autosave(&log);
                }
            }
        } else if (a == "set_garage_car" || a == "set_garage_car_trade") {
            std::vector<int> order = s->list_gt().garage_order(s->sort_key);
            s->garage_focus = s->garage_cursor >= 0 && s->garage_cursor < int(order.size())
                                  ? order[size_t(s->garage_cursor)]
                                  : -1;
        } else if (a == "set_focus_parts") {
            s->focus_goods = goods_of_item(e.item);
        } else if (a == "buy_parts" || a == "buy_fit_parts") {
            // (The Installed page shows the power before and with the part.)
            if (a == "buy_fit_parts") s->part_power_before = gt.ride_power_ps();
            if (gt.buy_part(s->focus_goods)) {
                log.push_back("bought part for " + cr(gt.part_offer(s->focus_goods) ? gt.part_offer(s->focus_goods)->price : 0));
                if (a == "buy_fit_parts") gt.fit_part(s->focus_goods);
            }
        } else if (a == "fit_parts") {
            s->part_power_before = gt.ride_power_ps();
            gt.fit_part(s->focus_goods);
        } else if (a == "buy_all_tires") {
            gt.buy_all_tires();
        } else if (a == "set_focus_wheel_code") {
            s->focus_wheel = wheel_of_item(e.item);
        } else if (a == "buy_wheel") {
            gt.buy_wheel(s->focus_wheel);
        } else if (a == "fit_wheel") {
            gt.fit_wheel(gt.state().ride, s->focus_wheel);
        } else if (a == "set_focus_wheel") {
            // The chosen inventory wheel (cursor) for the focused garage car.
        } else if (a == "change_wheel") {
            const auto& w = gt.state().wheels;
            std::string wheel = s->wheel_cursor >= 0 && s->wheel_cursor < int(w.size()) ? w[size_t(s->wheel_cursor)] : "";
            gt.fit_wheel(s->garage_focus, wheel);
        } else if (a == "wash_car") {
            if (gt.wash()) g_sounds.push_back("car_wash");
        } else if (a == "oil_change") {
            if (gt.change_oil()) g_sounds.insert(g_sounds.end(), {"oil", "oil"}); // the animation plays it twice
        } else if (a == "set_race_code" || a == "set_race_code_info" || a == "set_race_code_spot") {
            s->race_code = a == "set_race_code_info" && e.item.size() > 5 ? e.item.substr(0, e.item.size() - 5) : e.item;
            s->champ = false;
        } else if (a == "set_race_code_champ") {
            s->race_code = e.item;
            if (!s->race_code.empty()) ++s->race_code.back();
            s->champ = true;
        } else if (a == "clear_race_code") {
            s->race_code.clear();
            s->champ = false;
        } else if (starts(a, "sort_")) {
            static const char* const keys[] = {"date", "name", "maker", "color", "price",
                                               "distance", "power", "drive", "aspiration"};
            for (int k = 0; k < 9; ++k)
                if (a.substr(5) == keys[k]) s->sort_key = k;
            s->garage_cursor = 0;
        }
        return false;
    }
    // Finish go_save / go_load (settings root): the flow opens the save page over the
    // car's setting files (save_list / save_run below).
    if (e.kind == K::Finish) {
        const std::string& a = e.action;
        if (a == "go_race") {
            gt.enter();
            if (s->champ) gt.start_championship(s->race_code);
        } else if (a == "go_license" || a == "go_machine_test" || a == "go_free_run") {
            gt.enter();
            if (a == "go_free_run") g_free_run_course = e.item;
        }
        // go_save_in_home / go_save_game_menu / go_load_game_menu / go_trade: the flow
        // opens the save page (save_list / save_run below).
    }
    return false;
}

// ---------------------------------------------------------------------------
// Races.

void fill_car(GameRequest& r, const GtMode& gt, const GarageCar* c) {
    if (!c) return;
    std::string err;
    if (auto spec = gt.car_spec(*c, &err)) r.car_spec = std::make_shared<const sim::CarSpec>(std::move(*spec));
    else LOG_WARN("gt mode: car spec of %s: %s", c->car.c_str(), err.c_str());
    r.car = c->car;
    r.car_model = gt.model_of(*c);
    r.car_color = c->color;
    r.car_wheel = c->wheel;
    const ByteSpan* gear = gt.db().record(data::TableId::Gear, c->parts[size_t(part_slot(17))]);
    r.racing_gearbox = gear && gear->size() > 0x10 && (*gear)[0x10] != 0;
}

void fill_car(GameRequest& r, const GtMode& gt) { fill_car(r, gt, gt.state().ride_car()); }

void prepare_request(GameRequest& r) {
    GtModeSession* s = S();
    if (!s) return;
    GtMode& gt = s->gt();
    if (r.mode == "go_race") {
        const Championship& ch = gt.state().championship;
        if (ch.active()) {
            r.race_code = ch.races[size_t(ch.next)];
            r.opponents = ch.entrants;
            r.champ_round = ch.next + 1;
            r.champ_rounds = int(ch.races.size());
            r.champ_last = r.champ_round == r.champ_rounds;
        } else {
            r.race_code = s->race_code;
            r.opponents = gt.pick_opponents(r.race_code);
        }
        r.event = r.race_code;
        fill_car(r, gt);
    } else if (r.mode == "go_license") {
        r.event = r.race_code;
        if (auto rules = gt.rules(r.race_code); rules && !rules->entries.empty()) r.car = rules->entries[0];
    } else if (r.mode == "go_machine_test") {
        r.event = r.race_code;
        fill_car(r, gt);
    } else if (r.mode == "go_free_run") {
        r.event = r.race_code;
        r.course = g_free_run_course;
        fill_car(r, gt);
    } else if (r.mode == "go_preview") {
        r.race_code = s->race_code;
        r.event = s->race_code;
    }
    g_last_request = r;
}

// Result lines go to the log file only: the menu screen prints the flow's log
// on screen (a developer overlay).
std::vector<std::string> to_log_file(const std::vector<std::string>& lines) {
    for (const std::string& m : lines) LOG_INFO("gt mode: %s", m.c_str());
    return {};
}

// The race prize page for a GT race result (presentation.md §F): the event's cup for places
// 1-3, the consolation prize further back; none without a trophy class.
std::optional<Show> race_prize_show(const GtMode& gt, const std::string& code, int place, s64 money) {
    auto rr = gt.rules(code);
    if (!rr || rr->trophy_class < 5 || rr->trophy_class > 8 || place < 0 || money <= 0) return std::nullopt;
    static const char* const kClass[] = {"easy", "normal", "hard", "special"};
    static const char* const kMedal[] = {"gold", "silver", "bronze"};
    Show sh{};
    sh.money = money;
    sh.rank = ordinal(place);
    if (place < 3) {
        sh.set = sh.page = "gtmode_mini";
        sh.model = std::string("race/g_") + kClass[rr->trophy_class - 5] + "_" + kMedal[place] + ".prm";
    } else {
        sh.set = sh.page = "consolation_prize";
        sh.model = "race/consolation.prm";
    }
    return sh;
}

std::vector<std::string> on_result(const GameResult& r) {
    std::vector<std::string> log;
    g_round_pending = false;
    const size_t first_show = g_shows.size();
    GtModeSession* s = S();
    if (!s) return to_log_file(log);
    GtMode& gt = s->gt();
    if (r.distance_km > 0 && (r.mode == "go_race" || r.mode == "go_machine_test" || r.mode == "go_free_run"))
        gt.drive(gt.state().ride, r.distance_km * 1000.0);
    // The dirt the race built up on the car (washed off at the car wash).
    if (r.car_dirt >= 0 && r.mode != "attract_demo")
        if (GarageCar* ride = gt.state().ride_car()) ride->cond.dirt = float(std::clamp(r.car_dirt, 0.0, 0.8));
    const int place = r.finished && r.position > 0 ? r.position - 1 : -1;
    Award a;
    if (r.mode == "go_race") {
        if (gt.state().championship.active()) {
            if (r.abandoned) {
                log.push_back("championship abandoned");
                gt.state().championship = Championship{};
            } else {
                // Finishing order as entrant indices (0 = the player, i + 1 = opponent i).
                std::vector<int> order;
                for (int o : r.order) order.push_back(o + 1);
                a = gt.record_round(place, order);
                g_round_pending = gt.state().championship.active();
                if (a.championship_over)
                    log.push_back(a.champion ? "Champion! bonus " + cr(a.bonus) : "championship lost on points");
            }
        } else {
            a = gt.record_race(r.event.empty() ? r.race_code : r.event, place);
        }
        if (place >= 0) log.push_back("finished " + std::to_string(place + 1) + (a.money ? ": " + cr(a.money) : ""));
        // The race prize the original shows before anything else (presentation.md §F):
        // a cup for places 1-3 (race/g_<class>_<gold|silver|bronze>.prm, the event's trophy
        // class +0x1D3 = 5..8) or the consolation prize (race/consolation.prm) further back,
        // each with the prize money counted into the credits; nothing without prize money.
        if (place >= 0 && a.money - a.bonus > 0 && !r.prize_shown)
            if (std::optional<Show> sh = race_prize_show(gt, r.event.empty() ? r.race_code : r.event, place, a.money - a.bonus))
                g_shows.insert(g_shows.begin() + std::ptrdiff_t(first_show), std::move(*sh));
    } else if (r.mode == "go_license") {
        if (r.license_medal >= -1) {
            // Judged race-side: take the medal as is (and cross-check with our own judgement).
            Medal ours = gt.judge(r.race_code, r.finished, r.total_time);
            if (int(ours) != r.license_medal)
                LOG_WARN("gt mode: license %s: race judged %d, GT mode %d (time %.3f)", r.race_code.c_str(),
                         r.license_medal, int(ours), r.total_time);
            a = gt.record_license_medal(r.race_code, Medal(r.license_medal), r.total_time);
        } else {
            a = gt.record_license(r.race_code, r.finished, r.total_time);
        }
        log.push_back(std::string("license test: ") + medal_name(a.medal));
        for (const std::string& m : a.messages) log.push_back(m);
    }
    // Records rankings.
    {
        const std::string car = gt.state().ride_car() ? gt.state().ride_car()->car : "";
        // A result that makes a ranking asks for a name (the original's name entry page).
        auto name_entry = [&](const std::string& key, int rank) {
            if (rank >= 0) g_shows.push_back({"name_entry", "name_entry", {}, "", "", 0, {}, key, rank, "", 0, "", true, {}});
        };
        if (r.mode == "go_license" && r.finished && r.license_medal != -1 && r.total_time > 0 && !r.records_done) {
            std::string test_car = r.race_code;
            if (auto rr = gt.rules(r.race_code); rr && !rr->entries.empty())
                if (const data::CarVariant* v = gt.db().variant(rr->entries[0])) test_car = v->car;
            name_entry("lic:" + r.race_code, gt.add_record("lic:" + r.race_code, r.total_time, test_car, true, 6));
        }
        if (r.mode == "go_machine_test" && r.finished) {
            if (r.race_code == "MMAX")
                name_entry("mt:MMAX", gt.add_record("mt:MMAX", r.top_speed_kmh, car, false, 12));
            else
                name_entry("mt:" + r.race_code, gt.add_record("mt:" + r.race_code, r.total_time, car, true, 12));
        }
    }
    if (r.mode == "go_machine_test" && r.finished) {
        // Machine test records: 0-400 m and 0-1000 m times, top speed.
        MachineRecords& m = gt.state().machine;
        const std::string car = gt.state().ride_car() ? gt.state().ride_car()->car : "";
        if (r.race_code == "M400" && r.total_time > 0 && (m.t400 < 0 || r.total_time < m.t400))
            m.t400 = r.total_time, m.car400 = car, log.push_back("0-400 m record");
        if (r.race_code == "M1000" && r.total_time > 0 && (m.t1000 < 0 || r.total_time < m.t1000))
            m.t1000 = r.total_time, m.car1000 = car, log.push_back("0-1000 m record");
        if (r.race_code == "MMAX" && r.top_speed_kmh > m.top_speed)
            m.top_speed = r.top_speed_kmh, m.car_top = car, log.push_back("top speed record");
    }
    // Endings (spec §10): Beginner league complete -> short ending, Amateur or
    // Professional complete -> long ending, each once.
    if (r.mode == "go_race" && place == 0) {
        GameState& st = gt.state();
        if (!st.ending_short_seen && gt.league_complete("beginner")) {
            st.ending_short_seen = true;
            g_movie = "stream/ending_short_full.pss";
        } else if (!st.ending_long_seen && (gt.league_complete("amateur") || gt.league_complete("pro"))) {
            st.ending_long_seen = true;
            g_movie = "stream/ending_long_full.pss";
        }
    }
    if (a.license_obtained) {
        std::string id = license_menu_id(License(GtMode::license_index_of(r.race_code)));
        // The license card (lic_b, a, ib, ia, s, d = rally) over the two messages, shown in turn.
        g_shows.push_back({"lc_get", "license_get", {id + "_mess0"}, "", "", 0, {}, "", -1, "menu/lic_" + id + ".prm", 0, "", true, {}});
    }
    // A championship won: its trophy (picked by record_round, menu/champ_NN.prm) and the
    // bonus, before the prize car.
    // Screenshots: GT3R_GTMODE_FAKE_TROPHY=<NN> shows trophy champ_NN after any race win.
    const char* fake_trophy = std::getenv("GT3R_GTMODE_FAKE_TROPHY");
    if (a.champion || (fake_trophy && *fake_trophy && r.mode == "go_race" && place == 0)) {
        // (GT3R_GTMODE_FAKE_TROPHY=<NN>:<bonus> also counts a bonus.)
        int t = a.champion && a.trophy >= 0 ? a.trophy : std::clamp(std::atoi(fake_trophy ? fake_trophy : "0"), 0, 9);
        s64 bonus = a.bonus;
        if (!a.champion && fake_trophy)
            if (const char* c = std::strchr(fake_trophy, ':')) bonus = std::atoll(c + 1);
        char trophy[32];
        std::snprintf(trophy, sizeof trophy, "menu/champ_%02d.prm", t);
        Show sh{"trophy_get", "trophy_get", {"mess0"}, "", "", 0, {}, "", -1, trophy, bonus, "", a.champion, {}};
        sh.counter.delay = kTrophyCountDelay;
        g_shows.push_back(std::move(sh));
    }
    for (int i : a.new_cars)
        if (i >= 0 && i < int(gt.state().garage.size())) {
            const GarageCar& c = gt.state().garage[size_t(i)];
            log.push_back("prize car: " + car_name(gt, c.car));
            Show sh{"car_get", "car_get", {}, prize_reason(c.origin), c.car, c.color, {}, "", -1, "", 0, "", true, {}};
            if (const PrizeSet* ps = gt.data().prize_set(c.origin))
                for (const PrizeCar& pc : ps->cars) sh.candidates.push_back(pc.car);
            g_shows.push_back(std::move(sh));
        }
    // A prize car that did not fit in the garage: "Garage is full" (as the original).
    if (gt.take_prizes_refused() > 0) {
        Show sh{};
        sh.set = "not_get_car";
        sh.page = "not_car_get";
        g_shows.push_back(std::move(sh));
    }
    autosave(&log);
    return to_log_file(log);
}

std::optional<GameRequest> next_request() {
    if (!g_round_pending || !g_last_request || !S()) return std::nullopt;
    g_round_pending = false;
    GameRequest r = *g_last_request;
    prepare_request(r);
    return r;
}

// ---------------------------------------------------------------------------
// Text widgets.

std::optional<std::string> widget_text(const std::string& page, const Element& e) {
    GtModeSession* s = S();
    if (!s || !e.widget_class) return std::nullopt;
    const std::string& c = *e.widget_class;
    const GtMode& gt = s->gt();
    const GameState& st = gt.state();
    const GarageCar* ride = st.ride_car();
    // The garage pages show the guest garage during a trade.
    const GameState& lst = s->list_gt().state();
    const GarageCar* focus_g =
        s->garage_focus >= 0 && s->garage_focus < int(lst.garage.size()) ? &lst.garage[size_t(s->garage_focus)] : nullptr;
    char buf[96];

    if (is_setting_page(page) || page == "root" || page == "change_parts" || page == "parts_setting") {
        GarageCar* real = setting_real();
        GarageCar* work = setting_work();
        if (real && work) {
            if (auto t = setting_text(gt, *real, *work, c)) return t;
            // "%d %s": the setting car's computed power (parts, condition), truncated.
            if (c == "car_current_power") return gt.power_text(gt.power_ps(*work), " ");
        }
    }
    if (c == "text_box" && page == "name_entry") {
        RankEntry* r = named_entry();
        return r ? r->name : std::string();
    }
    // Race prizes and the championship trophy: the place, the prize money and the credits
    // counting up by it. The prize counter's items: "Cr. %s" (the balance) and "%s Cr. %s"
    // ("Prize Cr. 991", what is left, counting down; it fades out after the count, see
    // remain_alpha).
    if ((is_mini_prize(page) || page == "trophy_get") && g_show) {
        if (c == "rank_disp") return g_show->rank;
        const Show::Counter& k = g_show->counter;
        s64 base = g_show->credited ? st.credits - g_show->money : st.credits;
        s64 moved = k.started ? k.cur - base : 0;
        if (c == "get_money_disp") return "Prize Cr. " + format_credits(g_show->money - moved);
        if (c == "count" || c == "count_shadow") return "Cr. " + format_credits(base + moved);
    }
    if (c == "user_money") return " " + format_credits(st.credits); // a space after the Cr. icon (reference)
    if (c == "user_day") return std::to_string(st.day);
    // The status bar's day counter: the unit widget's own template filled with the day
    // ("day %d" -> "day 12"); the hidden "user_day" widget is not shown.
    if (c == "user_day_unit") {
        std::string t = e.text ? e.text->text : std::string();
        size_t at = t.find("%d");
        if (at == std::string::npos) return std::to_string(st.day);
        return t.substr(0, at) + std::to_string(st.day) + t.substr(at + 2);
    }
    if (c == "eventname") return std::string();     // drawn from the hidden reason element (extra_draws)
    if (c == "user_carname" || c == "user_carname_full") return ride ? car_name(gt, ride->car) : std::string();
    if (c == "focused_carname") {
        if (starts(page, "hm/") && focus_g) return car_name(gt, focus_g->car);
        return s->focus_car.empty() ? std::string() : car_name(gt, s->focus_car);
    }
    if (c == "goods_car_price") {
        // Widget "<car>_price[N]" on the dealer pages.
        std::string car = e.name.substr(0, e.name.rfind("_price"));
        if (const DealerCar* d = gt.data().catalog.dealer_car(car)) car = d->car;
        return gt.db().car(car) ? cr_pre(gt.car_price(car)) : std::string();
    }
    if (c == "car_buy_price") {
        if (starts(page, "hm/") && focus_g) return cr_num(gt.car_price(focus_g->car));
        // After the Cr. icon one space in (the reference dealer car pages: " 17,000").
        return s->focus_car.empty() ? std::string() : " " + cr_num(gt.car_price(s->focus_car));
    }
    if (c == "car_sell_price") {
        // (The garage car page insets its texts instead: extra_draws.)
        const std::string pad = starts(page, "hm/hm_car") ? "" : " ";
        return focus_g ? (gt.car_flags(focus_g->car) & 2 ? std::string("----") : pad + cr_num(gt.sell_price(*focus_g)))
                       : std::string();
    }
    if (starts(c, "price_")) {
        std::string item = c.substr(6);
        // After a Cr. icon: " 3,000" (the reference shop pages).
        if (item == "wheel") return " " + cr_num(gt.data().wheel_price());
        if (item == "car_wash") return " " + cr_num(gt.data().car_wash_price());
        if (item == "oil_change") return " " + cr_num(gt.data().oil_change_price());
        int g = goods_of_item(item);
        if (g < 0 || !ride) return std::string("----");
        if (gt.owns(g)) return std::string("Purchased");
        auto o = gt.part_offer(g);
        return o ? " " + cr_num(o->price) : std::string("----");
    }
    if (c == "car_previous_torque" || c == "car_current_torque") {
        // The Installed page (ts_fit_end1 / 3): despite the class names, the power before the
        // part (kept when it was fitted) and now, "%d %s" ("281 HP").
        if (!ride) return std::string();
        return gt.power_text(c == "car_previous_torque" ? s->part_power_before : gt.ride_power_ps(), " ");
    }
    if (c == "disp_ps_befor" || c == "disp_ps_after") {
        if (!ride) return std::string();
        int goods = s->focus_goods;
        const auto& pages = gt.data().catalog.shop_item_of_page;
        if (auto it = pages.find(page); it != pages.end()) goods = goods_of_item(it->second);
        float ps = c == "disp_ps_befor" || goods < 0 ? gt.ride_power_ps() : gt.power_with(goods);
        return gt.power_text(ps); // "%d%s"
    }
    if (c == "focused_partsname") return std::string();
    if (c == "color_name_box" || c == "car_color_name") {
        if (starts(page, "hm/") && focus_g) return gt.data().color_name(gt.data().color_id(focus_g->car, focus_g->color));
        if (s->focus_car.empty()) return std::string();
        // Dealer car pages: the reference screens start the name one space in (as the price).
        return " " + gt.data().color_name(gt.data().color_id(s->focus_car, s->color_cursor));
    }
    if (c == "disp_time_gold" || c == "disp_time_silver" || c == "disp_time_bronze") {
        std::string code = e.name.substr(0, e.name.find('_'));
        auto r = gt.rules(code);
        if (!r) return std::string();
        int m = c == "disp_time_gold" ? 0 : c == "disp_time_silver" ? 1 : 2;
        return format_time_ms(r->license_ms[size_t(m)]);
    }
    if (c == "license_car_name") {
        std::string code = e.name.substr(0, e.name.find('_'));
        auto r = gt.rules(code);
        if (!r || r->entries.empty()) return std::string();
        const data::CarVariant* v = gt.db().variant(r->entries[0]);
        return car_name(gt, v ? v->car : r->entries[0]);
    }
    if (c == "license_launch_speed") {
        std::string code = e.name.substr(0, e.name.find('_'));
        const data::Event* ev = gt.db().event(code);
        if (!ev || ev->bytes.size() < 0x199) return std::string();
        // The start speed (also 0, as "0mph" on the B-1 page); NTSC-U in mph.
        if (gt.data().imperial) std::snprintf(buf, sizeof buf, "%dmph", int(std::lround(ev->bytes[0x198] * 0.621371)));
        else std::snprintf(buf, sizeof buf, "%dkm/h", ev->bytes[0x198]);
        return std::string(buf);
    }
    if (starts(c, "race_prize_")) {
        int p = std::atoi(c.c_str() + 11) - 1;
        auto r = gt.rules(s->race_code);
        if (!r || p < 0 || p > 5) return std::string();
        return r->prize[size_t(p)] ? cr_num(r->prize[size_t(p)]) : std::string("-");
    }
    if (starts(c, "regulation_")) {
        auto r = gt.rules(s->race_code);
        if (!r) return std::string();
        // No rule: "No Limitations" (the original's race information page).
        const std::string none = "No Limitations";
        if (c == "regulation_license")
            return r->required_license ? std::string(license_name(License(r->required_license - 1))) : none;
        if (c == "regulation_dr_type") {
            static const char* const n[] = {"", "FF", "FR", "4WD", "MR", "RR"};
            return r->drivetrain > 0 && r->drivetrain <= 5 ? std::string(n[r->drivetrain]) : none;
        }
        if (c == "regulation_aspiration") return r->aspiration == 1 ? "NA" : r->aspiration == 2 ? "TURBO" : none;
        if (c == "regulation_tire") return r->dirt_tires ? std::string("DIRT") : none;
        if (c == "regulation_tuning")
            return r->car_class == 1 ? "RACE CAR" : r->car_class == 2 ? "TUNED" : r->car_class == 3 ? "NORMAL" : none;
        if (c == "regulation_car") return r->regulation.empty() ? none : std::string("LIMITED");
    }
    if (starts(c, "status_")) {
        int gold = 0, silver = 0, bronze = 0;
        for (const LicenseTestRecord& t : st.license_tests) {
            gold += t.medal == Medal::Gold;
            silver += t.medal == Medal::Silver;
            bronze += t.medal == Medal::Bronze;
        }
        s64 value = 0;
        for (const GarageCar& g : st.garage) value += gt.car_price(g.car);
        if (c == "status_ttl_wins") return std::to_string(st.wins);
        if (c == "status_ttl_races") return std::to_string(st.races);
        if (c == "status_avg_win") {
            // No race yet: "0 %" (the reference status page).
            if (!st.races) return std::string("0 %");
            std::snprintf(buf, sizeof buf, "%.1f %%", 100.0 * st.wins / st.races);
            return std::string(buf);
        }
        if (c == "status_ttl_milage") {
            // NTSC-U: miles (the reference status page shows "0.0 miles").
            if (gt.data().imperial) std::snprintf(buf, sizeof buf, "%.1f miles", st.distance_m / 1609.344);
            else std::snprintf(buf, sizeof buf, "%.1f km", st.distance_m / 1000.0);
            return std::string(buf);
        }
        if (c == "status_prize_money") return " " + cr_num(st.prize_money); // after the Cr. icon
        if (c == "status_no_trophy") return std::to_string(st.trophies);
        if (c == "status_no_cars") return std::to_string(st.garage.size());
        if (c == "status_ttl_car_price") return " " + cr_num(value);
        if (c == "status_ave_prize_car") return std::to_string(st.prize_cars);
        if (c == "status_no_lc_g") return std::to_string(gold);
        if (c == "status_no_lc_s") return std::to_string(silver);
        if (c == "status_no_lc_b") return std::to_string(bronze);
        if (c == "status_complete") return gt.completion_text();
    }
    // Car pages: the focused garage car at home, else the dealer's focus car.
    const std::string car = starts(page, "hm/") && focus_g ? focus_g->car : s->focus_car;
    if (starts(c, "spec_") || starts(c, "car_")) {
        const data::Car* dc = gt.db().car(car);
        if (!dc) return std::nullopt;
        data::CarDatabase::Summary sum = gt.db().summarize(*dc);
        auto num = [&](const char* fmt, double v) {
            std::snprintf(buf, sizeof buf, fmt, v);
            return std::string(buf);
        };
        // Power. car_max_power and
        // spec_max_power_real: the built car's computed peak "%d%s/%drpm" ("325HP/6800rpm" on
        // NTSC-U): a garage car with its parts and condition, a dealer car stock. spec_max_power:
        // the catalogue figure and rpm text. All truncated after the unit conversion.
        auto peak_text = [&](const PowerPeak& p) {
            return gt.power_text(p.ps) + "/" + std::to_string(p.rpm) + "rpm";
        };
        if ((c == "car_max_power" || c == "spec_max_power_real") && starts(page, "hm/") && focus_g)
            return peak_text(gt.power_peak(*focus_g));
        if (c == "car_max_power" || c == "spec_max_power_real") {
            if (auto spec = sim::build_car_spec(gt.db(), car)) return peak_text(spec_power_peak(*spec));
            return std::string("-");
        }
        if (c == "spec_max_power")
            return gt.power_text(float(sum.power_ps)) + "/" + sum.power_rpm;
        if (c == "spec_max_torque" || c == "spec_max_torque_real") return num("%.1f kgfm", double(sum.torque_kgfm));
        if (c == "spec_displacement" || c == "spec_displacement_real") return sum.displacement;
        if (c == "spec_weight") return num("%.0f kg", sum.weight_kg);
        if (c == "spec_height") return num("%.0f mm", sum.height_mm);
        if (c == "spec_length") return num("%.0f mm", sum.length_mm);
        if (c == "spec_width") return num("%.0f mm", sum.width_mm);
        if (c == "spec_wheel_base") return num("%.0f mm", sum.wheelbase_mm);
        if (c == "spec_drive_type" || c == "car_drive_type") return sum.drive;
        if (c == "spec_engine_type") return sum.layout;
        if (c == "spec_aspiration") return sum.aspiration;
        if (c == "spec_tread_f" || c == "spec_tread_r") {
            const data::Car* dcar = gt.db().car(car);
            auto body = dcar ? gt.db().body(dcar->parts[5]) : std::nullopt;
            if (!body) return std::string("-");
            return num("%.0f mm", c == "spec_tread_f" ? body->front_tread_mm : body->rear_tread_mm);
        }
        if (c == "spec_t_w_ratio" || c == "spec_t_w_ratio_real")
            return sum.torque_kgfm > 0 ? num("%.2f kg/kgfm", double(sum.weight_kg) / sum.torque_kgfm) : std::string("-");
        if (c == "spec_p_w_ratio" || c == "spec_p_w_ratio_real")
            return sum.power_ps ? num("%.2f kg/PS", double(sum.weight_kg) / sum.power_ps) : std::string("-");
        if (c == "car_kilometrage")
            return gt.data().imperial
                       ? (focus_g ? num("%.1fmiles", focus_g->cond.odometer_m / 1609.344) : std::string("0.0miles"))
                       : (focus_g ? num("%.1f km", focus_g->cond.odometer_m / 1000.0) : std::string("0.0 km"));
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Code-drawn lists: the garage (hm/hm_gr) and the wheel inventory cursor.

// GT Mode map pointer (MapButtonList, spec menus): a direction picks the building that
// maximises cos^2 / distance among those within 90 degrees of it; the pointer eases
// there (30 % per frame) and the building takes the focus within 12 px.
struct MapPointer {
    bool active = false;
    float x = 320, y = 224;  // menu units (y up)
    int target = -1;
    bool moving = false;     // a direction chose a target the pointer has not reached yet
    menu::flow::MenuKeys prev = 0;
};
MapPointer g_map;

void map_centre(const MenuController& ctl, int i, float& x, float& y) {
    const Element* e = i >= 0 && i < int(ctl.items().size()) ? ctl.items()[size_t(i)].element : nullptr;
    x = e ? e->x + e->w * 0.5f : 320;
    y = e ? e->y + e->h * 0.5f : 224;
}

void map_input(MenuController& ctl, menu::flow::MenuKeys held) {
    if (!g_map.active) {
        g_map = MapPointer{};
        g_map.active = true;
        g_map.target = ctl.focus();
    }
    menu::flow::MenuKeys fired = held & ~g_map.prev;
    g_map.prev = held;
    float dx = (fired & key_bit(MenuKey::Right)) ? 1.0f : (fired & key_bit(MenuKey::Left)) ? -1.0f : 0.0f;
    float dy = (fired & key_bit(MenuKey::Up)) ? 1.0f : (fired & key_bit(MenuKey::Down)) ? -1.0f : 0.0f;
    if ((dx != 0 || dy != 0) && g_map.target >= 0) {
        float cx, cy;
        map_centre(ctl, g_map.target, cx, cy);
        float dl = std::sqrt(dx * dx + dy * dy), best = 0;
        int pick = -1;
        for (int i = 0; i < int(ctl.items().size()); ++i) {
            if (i == g_map.target || !ctl.items()[size_t(i)].element) continue;
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
            g_map.target = pick;
            g_map.moving = true;
            g_sounds.push_back("cursor");
        }
    }
}

std::string on_frame(MenuController& ctl) {
    // The original's menu loops step the GT-mode generator once per frame (spec §20.2).
    if (GtModeSession* rs = S()) rs->gt().tick_random();
    // GT Mode map: the pointer eases to its building, which takes the focus on arrival.
    if (ctl.page_name() == "map" && ctl.depth() == 0 && g_map.active && g_map.target >= 0) {
        // Focus set by other means (taps, returning to the page): the pointer follows it.
        if (!g_map.moving && ctl.focus() >= 0) g_map.target = ctl.focus();
        float tx, ty;
        map_centre(ctl, g_map.target, tx, ty);
        g_map.x += (tx - g_map.x) * 0.3f;
        g_map.y += (ty - g_map.y) * 0.3f;
        float ex = tx - g_map.x, ey = ty - g_map.y;
        if (ex * ex + ey * ey < 144 && g_map.moving) {
            g_map.moving = false;
            if (ctl.focus() != g_map.target && ctl.items()[size_t(g_map.target)].flow)
                ctl.set_focus(ctl.items()[size_t(g_map.target)].flow->name);
        }
    } else if (ctl.page_name() != "map") {
        g_map.active = false;
    }
    if (!g_sounds.empty()) {
        std::string se = g_sounds.front();
        g_sounds.erase(g_sounds.begin());
        return se;
    }
    GtModeSession* s = S();
    if (!s || !ctl.has_page()) return {};
    const std::string& page = ctl.page_name();
    // Code-driven lists live on base pages; the settings sliders and the permitted cars
    // are popups.
    if (ctl.depth() != 0 && !starts(page, "set/adjast_") && page != "00cmn/car_reguration") return {};
    if (page == "car_get" || page == "not_car_get" || is_mini_prize(page)) {
        ++g_show_frames;
        if (g_show && (page == "gtmode_mini" || page == "consolation_prize"))
            run_counter(*g_show, s->gt().state(),
                        (ctl.pressed() & menu::flow::key_bit(menu::flow::MenuKey::Confirm)) != 0, &g_sounds);
        return {};
    }
    // License card: "<x>_mess0" for 4 s, nothing for 2 s, then "<x>_mess1"; OK after 7 s.
    if (page == "license_get" && g_show && !g_show->layers.empty()) {
        int f = ++g_show_frames;
        std::string id = g_show->layers[0].substr(0, g_show->layers[0].find("_mess"));
        ctl.set_layer(id + "_mess0", f < kLicMess0End ? 1 : 0);
        ctl.set_layer(id + "_mess1", f >= kLicMess1 ? 1 : 0);
        return {};
    }
    // Championship trophy: "mess0" for 3 s, the bonus box 1.5 s later, then 1 s; the
    // credits count up (the prize counter, from 6 s); 15 frames after that "mess1" and OK.
    if (page == "trophy_get") {
        int f = ++g_show_frames;
        if (g_show)
            run_counter(*g_show, s->gt().state(),
                        (ctl.pressed() & menu::flow::key_bit(menu::flow::MenuKey::Confirm)) != 0, &g_sounds);
        ctl.set_layer("mess0", f < kTrophyMess0End ? 1 : 0);
        ctl.set_layer("get_mess", f >= kTrophyMoney ? 1 : 0);
        ctl.set_layer("mess1", trophy_ok() ? 1 : 0);
        return {};
    }
    if (page == "name_entry") return {}; // Back Space: GtModeHooks::name_backspace
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

const Element* find_widget_named(const menu::Page& p, const std::string& name) {
    for (const menu::Layer& l : p.layers)
        for (const Element& e : l.widgets)
            if (e.name == name) return &e;
    return nullptr;
}

const Element* hidden_element(const menu::Page& p, const std::string& name) {
    for (const menu::Layer& l : p.layers)
        if (l.name == "hidden")
            for (const Element& e : l.elements)
                if (e.name == name) return &e;
    return nullptr;
}

// Image widgets filled from the page's hidden-layer art (they have no parts of
// their own): license cups, league result marks, required-license marks, car logos.
void widget_draws(const MenuController& ctl, const Element& w, std::vector<menu::ElementDraw>& out) {
    GtModeSession* s = S();
    if (!s || !w.widget_class) return;
    const std::string& cls = *w.widget_class;
    menu::flow::MenuSet::PageRef ref = ctl.page();
    const GtMode& gt = s->gt();
    auto put = [&](const Element* e) {
        if (!e) return;
        menu::ElementDraw d;
        d.scene = ref.scene;
        d.element = e;
        d.has_box = true;
        d.box = {w.x, w.y, w.w, w.h};
        d.z = w.z;
        out.push_back(d);
    };
    // Arcade Clear Status (ar/goodies): per-course logo / name once the course is
    // open, Easy / Normal / Hard clear marks (a harder win counts for the easier
    // marks; no Professional mark) and per-group prize icons.
    if (ctl.page_name() == "ar/goodies") {
        auto num = [&](size_t from) { return std::atoi(cls.c_str() + from) - 1; };
        auto list = gt.data().arcade_courses.find("SINGLE_RACE");
        if (list == gt.data().arcade_courses.end()) return;
        const auto& courses = list->second;
        auto course_at = [&](int i) -> const GtData::ArcadeCourse* {
            return i >= 0 && i < int(courses.size()) ? &courses[size_t(i)] : nullptr;
        };
        if (starts(cls, "course_logo_") || starts(cls, "course_name_")) {
            const GtData::ArcadeCourse* c = course_at(num(12));
            if (c && gt.arcade_course_open(c->course))
                put(hidden_element(ref.layout(), c->course + (starts(cls, "course_logo_") ? "_mini" : "_doc")));
            return;
        }
        static const char* const kDiff[3] = {"easy", "normal", "hard"};
        for (int d = 0; d < 3; ++d) {
            std::string mark = std::string("mark_") + kDiff[d] + "_";
            if (starts(cls, mark.c_str())) {
                if (gt.arcade_won_at_least(num(mark.size()), d))
                    put(hidden_element(ref.layout(), std::string("clear_") + kDiff[d]));
                return;
            }
            std::string prize = std::string("prize_") + kDiff[d] + "_";
            if (starts(cls, prize.c_str()) && cls.size() == prize.size() + 1) {
                if (gt.arcade_group_won(cls.back() - 'a', d))
                    put(hidden_element(ref.layout(), std::string("prize_item_") + kDiff[d]));
                return;
            }
        }
        return;
    }
    if (starts(cls, "disp_license_cup_")) {
        std::string id = cls.substr(17);
        if (id.size() < 2) return;
        int test = id.back() - '1';
        id.pop_back();
        int lic = id == "r" ? 5 : int(license_from_menu_id(id));
        if (lic < 0 || test < 0 || test > 7) return;
        Medal m = gt.state().license_tests[size_t(lic * 8 + test)].medal;
        if (m != Medal::None) put(hidden_element(ref.layout(), std::string("cup_") + medal_name(m)));
        return;
    }
    if (cls == "event_status" || cls == "need_license") {
        // "event_status7" / "need_license7" belong to the item whose focus layer is "s_7".
        const std::string layer = "s_" + w.name.substr(12);
        const RaceSeries* series = nullptr;
        if (ref.flow)
            for (const MblItem& it : ref.flow->items)
                if (it.str80 == layer)
                    for (const RaceSeries& sr : gt.data().catalog.series)
                        if (sr.page == it.link) series = &sr;
        if (!series || series->races.empty()) return;
        if (cls == "need_license") {
            auto r = gt.rules(series->races.front());
            if (r && r->required_license > 0) {
                std::string l = license_menu_id(License(r->required_license - 1));
                put(hidden_element(ref.layout(), "lc_mark_" + (l == "d" ? std::string("r") : l)));
            }
            return;
        }
        // One slot per race, 40 px apart, centred on the widget (the reference league pages:
        // a black "frame" disc per race, its best-result mark over it).
        std::vector<int> best;
        if (series->championship())
            best.push_back(gt.state().best_position(series->races.front().substr(0, series->races.front().size() - 2) + "00"));
        else
            for (const std::string& r : series->races) best.push_back(gt.state().best_position(r));
        const Element* frame = hidden_element(ref.layout(), "frame");
        const float cx = w.x + w.w * 0.5f, n = float(best.size());
        for (size_t i = 0; i < best.size(); ++i) {
            float x = cx + (float(i) - (n - 1) * 0.5f) * 40.0f;
            auto put_at = [&](const Element* e) {
                if (!e) return;
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = e;
                d.has_box = true;
                d.box = {x - e->w * 0.5f, w.y, e->w, e->h};
                d.z = w.z + (e == frame ? 0 : 0.01f);
                out.push_back(d);
            };
            put_at(frame);
            if (best[i] >= 0 && best[i] < 6) put_at(hidden_element(ref.layout(), "mark_" + std::to_string(best[i] + 1)));
        }
        return;
    }
    // Status page: the highest license held, and the rally license.
    if (cls == "license_mark") {
        // lc_mark1..5 are in letter order (A, B, IA, IB, S), as the images show.
        static const int kMark[5] = {2, 1, 4, 3, 5}; // B, A, IB, IA, S
        if (int lic = gt.highest_license(); lic >= 0 && lic < 5)
            put(hidden_element(ref.layout(), "lc_mark" + std::to_string(kMark[lic])));
        return;
    }
    if (cls == "license_mark_rally") {
        if (gt.license_held(5)) put(hidden_element(ref.layout(), "lc_mark6"));
        return;
    }
    // License violation popup: the license the selected race needs.
    if (cls == "violated_license") {
        if (auto r = gt.rules(s->race_code); r && r->required_license > 0) {
            std::string n = r->required_license == 6 ? "DIRT" : license_name(License(r->required_license - 1));
            put(hidden_element(ref.layout(), n));
        }
        return;
    }
    // Tune shop: the maker logo of the ride car ("ASTON MARTIN" -> "astonmartin", "MG MINI" -> "mg_mini").
    if (cls == "tuner_logo") {
        const GarageCar* ride = gt.state().ride_car();
        const data::Car* car = ride ? gt.db().car(ride->car) : nullptr;
        if (!car) return;
        std::string joined, underscored;
        for (char ch : car->maker) {
            char l = char(std::tolower(u8(ch)));
            if (std::isalnum(u8(l))) joined.push_back(l), underscored.push_back(l);
            else if (ch == ' ' || ch == '-') underscored.push_back('_');
        }
        const Element* logo = hidden_element(ref.layout(), joined);
        put(logo ? logo : hidden_element(ref.layout(), underscored));
        return;
    }
    // Oil change: the oil-age gauge (empty = fresh), a bar filling from the bottom.
    if (cls == "oil_change_cylinder") {
        const GarageCar* ride = gt.state().ride_car();
        if (!ride) return;
        float o = ride->cond.oil_m;
        float level = o < 0 ? (o + 300000) / 300000 : std::min(o / 300000, 1.0f);
        menu::ElementDraw d;
        d.kind = menu::ElementDraw::Kind::Fill;
        d.has_box = true;
        d.box = {w.x, w.y, w.w, w.h * std::clamp(level, 0.0f, 1.0f)};
        d.fill = {0.85f, 0.55f, 0.1f, 0.9f};
        d.z = w.z;
        out.push_back(d);
        return;
    }
    if (cls == "gear_chart") {
        // Gear chart: engine speed (up) against road speed (right) for each gear, a
        // sawtooth from the rpm after each upshift to the rev limit; dotted lines.
        GarageCar* work = setting_work();
        if (!work) return;
        std::optional<sim::CarSpec> spec = gt.car_spec(*work);
        if (!spec || spec->gearbox.ratios.empty()) return;
        const auto& g = spec->gearbox.ratios;
        const float rev = spec->engine.rev_limit_rpm, fd = spec->gearbox.final_drive;
        const float radius = spec->axle[1].tire_radius;
        auto kmh = [&](float rpm, float ratio) { return rpm * 0.10472f * radius / (ratio * fd) * 3.6f; };
        const float vmax = kmh(rev, g.back()) * 1.05f;
        if (vmax <= 0 || rev <= 0) return;
        auto dot = [&](float v, float rpm, SDL_FColor c, float size) {
            menu::ElementDraw d;
            d.kind = menu::ElementDraw::Kind::Fill;
            d.has_box = true;
            float x = w.x + 4 + (w.w - 8) * std::clamp(v / vmax, 0.0f, 1.0f);
            float y = w.y + 4 + (w.h - 8) * std::clamp(rpm / (rev * 1.05f), 0.0f, 1.0f);
            d.box = {x - size / 2, y - size / 2, size, size};
            d.fill = c;
            d.z = w.z;
            out.push_back(d);
        };
        // Rev limit line.
        for (int k = 0; k <= 60; ++k) dot(vmax * float(k) / 60, rev, {0.6f, 0.15f, 0.1f, 0.8f}, 1.2f);
        for (size_t i = 0; i < g.size(); ++i) {
            float r0 = i == 0 ? 0.0f : rev * g[i] / g[i - 1];
            float v0 = kmh(r0, g[i]), v1 = kmh(rev, g[i]);
            for (int k = 0; k <= 40; ++k) {
                float t = float(k) / 40;
                dot(v0 + (v1 - v0) * t, r0 + (rev - r0) * t, {1.0f, 0.68f, 0.0f, 1}, 2.0f);
            }
        }
        return;
    }
    if (cls == "car_logo") {
        std::string car = s->focus_car;
        if (starts(ctl.page_name(), "hm/") && s->garage_focus >= 0 && s->garage_focus < int(gt.state().garage.size()))
            car = gt.state().garage[size_t(s->garage_focus)].car;
        const Element* e = hidden_element(ref.layout(), car);
        if (!e)
            if (const DealerCar* d = gt.data().catalog.dealer_car(car)) e = hidden_element(ref.layout(), d->item);
        put(e);
    }
}

// Code-drawn list geometry (the garage, the permitted cars; shared by drawing and touch):
// rows of the page's item template in its window widget, the cursor row centred while the
// list scrolls.
struct GarageRows {
    const Element* tmpl = nullptr;
    const Element* win = nullptr;
    float row = 0;
    int rows = 0, first = 0, n = 0;
    float y(int i) const { return win->y + win->h - float(i - first + 1) * row; }
};

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

// The race information's Cars Permitted popup.
bool regulated_rows(const MenuController& ctl, GtModeSession& s, int n, GarageRows& g) {
    return ctl.page_name() == "00cmn/car_reguration" && list_rows(ctl, "menu_regulated_car", n, s.regulated_cursor, g);
}

// A tap on a garage row moves the cursor there; a tap on the row under the
// cursor opens the car (confirms the list's item). The permitted-car rows only move
// the cursor.
// The wheel shop's tiles on a brand page: the brand's wheels in a row in the "menu_wheel_shop"
// window (WheelShopButton: 52 x 52 tiles 56.5 units apart, centred while they fit, else the
// focused one centred within the ends). While a hovering pointer moves the cursor the row holds
// still (`pin`), so the tile stays under it; a cursor key releases it.
struct WheelRow {
    const Element* win = nullptr;
    int n = 0;
    float tile = 52, step = 56.5f, off = 0;
    SDL_FRect box(int i) const { return {win->x + off + float(i) * step, win->y + (win->h - tile) * 0.5f, tile, tile}; }
};
struct WheelPin {
    std::string page;
    int focus = -1;
    float off = 0;
} g_wheel_pin;

bool wheel_row(const MenuController& ctl, WheelRow& r) {
    if (!ctl.has_page() || ctl.depth() != 0 || !starts(ctl.page_name(), "at/ws/ws_")) return false;
    r.win = find_widget_named(ctl.page().layout(), "menu_wheel_shop");
    if (!r.win) return false;
    r.n = int(ctl.items().size());
    r.tile = ctl.items().empty() || !ctl.items()[0].element ? 52.0f : ctl.items()[0].element->w;
    r.step = r.tile + 4.5f;
    const float total = float(r.n) * r.step - 4.5f;
    const int f = std::max(0, ctl.focus());
    if (g_wheel_pin.page == ctl.page_name() && g_wheel_pin.focus != f) g_wheel_pin = {}; // moved by key
    if (g_wheel_pin.page == ctl.page_name()) r.off = g_wheel_pin.off;
    else if (total <= r.win->w) r.off = (r.win->w - total) * 0.5f;
    else r.off = std::clamp(r.win->w * 0.5f - (float(f) * r.step + r.tile * 0.5f), r.win->w - total, 0.0f);
    return true;
}

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

int garage_hover(MenuController& ctl, float x, float y) {
    if (WheelRow r; wheel_row(ctl, r)) {
        int i = wheel_tile_at(r, x, y);
        if (i < 0) return 0;
        if (i == ctl.focus()) return 1;
        const float off = r.off;
        ctl.set_focus(ctl.items()[size_t(i)].flow->name);
        g_wheel_pin = {ctl.page_name(), i, off};
        return 2;
    }
    int* cursor = nullptr;
    int i = garage_row_at(ctl, x, y, cursor);
    if (i < 0) return 0;
    if (*cursor == i) return 1;
    *cursor = i;
    return 2;
}

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

void extra_draws(const MenuController& ctl, std::vector<menu::ElementDraw>& out) {
    GtModeSession* s = S();
    if (!s || !ctl.has_page()) return;
    const std::string& page = ctl.page_name();
    // The prize line ("Prize Cr. <left>", get_money_disp) fades in when its layer
    // appears (the trophy's get_mess box) and out after the count.
    if ((page == "gtmode_mini" || page == "consolation_prize" || page == "trophy_get") && g_show) {
        float a = remain_alpha(*g_show, g_show_frames, page == "trophy_get" ? kTrophyMoney : 0);
        for (menu::ElementDraw& d : out)
            if (d.element && d.element->widget_class && *d.element->widget_class == "get_money_disp") d.alpha *= a;
    }
    // Race prizes and the trophy: the OK button appears only once it is accepted (the
    // reference captures: none while the money counts, then the round OK; the trophy's OK
    // belongs to its mess1 layer).
    if ((is_mini_prize(page) || page == "trophy_get") && g_show && ctl.has_page() && !ctl.items().empty() &&
        !enabled(page, *ctl.items()[0].flow)) {
        const Element* ok = ctl.items()[0].element;
        out.erase(std::remove_if(out.begin(), out.end(),
                                 [&](const menu::ElementDraw& d) {
                                     return ok && d.element && (d.element == ok || (!ok->alt.empty() && d.element == &ok->alt[0]));
                                 }),
                  out.end());
    }
    // GT Mode map: the pointer.
    if (page == "map" && ctl.depth() == 0) {
        const menu::flow::MenuSet::PageRef ref = ctl.page(); // (a handle: the layout lives in the menu set)
        const menu::Page& lay = ref.layout();
        if (const Element* p = hidden_element(lay, "pointer")) {
            menu::ElementDraw d;
            d.scene = ctl.page().scene;
            d.element = p;
            d.has_box = true;
            float px = g_map.active ? g_map.x : 320, py = g_map.active ? g_map.y : 224;
            d.box = {px, py - p->h, p->w, p->h}; // the tip (top-left corner) at the point
            d.z = p->z + 1000;
            out.push_back(d);
        }
        // (The hidden "name_<id>" labels are not shown: the original's bar carries only the
        // description ticker, as the reference screens show.)
    }
    // Prize presentations: the 3D model goes over the page's art (its widget sits
    // under the full-page background in the layout's order).
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
    // Status bar: the day counter ("day 12", widget_text) in the unit widgets' own style
    // (grey, black shadow), right-aligned. The widget is 24 px wide; the original does not
    // squeeze the text into it, and the reference screens end it 10 px before the box's right
    // edge (M: maybe the width of the "day %d" template is what is aligned).
    if (ctl.page().flow && ctl.page().flow->str44 == "00cmn/status")
        for (menu::ElementDraw& d : out)
            if (d.element && d.element->widget_class && *d.element->widget_class == "user_day_unit") {
                const float right = d.element->x + d.element->w - 10;
                d.has_box = true;
                d.box = {right - 96, d.element->y, 96, d.element->h};
            }
    menu::flow::MenuSet::PageRef ref = ctl.page();
    if (!ref) return;
    // Garage car page: its left / right aligned texts sit 12 units in from their boxes, as
    // the original draws them (menus.md §5 "Text inset"; the reference: VALUE, FR, power,
    // mileage, colour name all within 2 units). Only the page's own texts (not the status bar).
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
    // Prize presentation: the reason line ("Championship winner", ...) is one of the
    // page's hidden text elements, shown in the "event_message" widget's place.
    if (page == "car_get" && g_show)
        if (const Element* msg = find_widget_named(ref.layout(), "event_message"))
            if (const Element* e = hidden_element(ref.layout(), g_show->reason)) {
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = e;
                d.has_box = true;
                d.box = {msg->x, msg->y, msg->w, msg->h};
                d.z = msg->z;
                out.push_back(d);
            }
    // Series race pages: each race button is a frame that code fills with the
    // course logo (hidden element named after the event's course) and, in its
    // round slot, the best-result mark (hidden "mark_<place>").
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
            if (auto rules = s->gt().rules(it.flow->name))
                if (const Element* logo = hidden_element(ref.layout(), rules->course))
                    put(logo, {box.x + 3, box.y + 2, box.h * 1.3f, box.h - 4});
            // The focused race's course name ("<course>_doc") in the page's text bar.
            if (i == ctl.focus())
                if (auto rules = s->gt().rules(it.flow->name))
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
            int best = s->gt().state().best_position(it.flow->name);
            if (best >= 0 && best < 6)
                if (const Element* mark = hidden_element(ref.layout(), "mark_" + std::to_string(best + 1)))
                    put(mark, {box.x + box.w - box.h + 2, box.y + 2, box.h - 4, box.h - 4});
        }
    }
    // Settings sliders: the value marker ("pinch"; "pinch_fix" for a fixed setting) and
    // the value the page opened with ("pinch_prev", until a commit).
    if (starts(page, "set/adjast_"))
        if (GarageCar* real = setting_real(); real && setting_work()) {
            const Element* pinch = hidden_element(ref.layout(), "pinch");
            const Element* prev = hidden_element(ref.layout(), "pinch_prev");
            const Element* fix = hidden_element(ref.layout(), "pinch_fix");
            for (int i = 0; i < int(ctl.items().size()); ++i) {
                const MenuController::ItemState& it = ctl.items()[size_t(i)];
                if (!it.flow || !it.element || !starts(it.flow->name, "slider_")) continue;
                std::optional<SliderRange> r = slider_range(s->gt(), *real, *setting_work(), it.flow->name);
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
                bool locked = !slider_enabled(s->gt(), *real, it.flow->name);
                if (r->fixed() || locked) {
                    put(fix, r->begin);
                    continue;
                }
                auto entry = s->setting_entry.find(it.flow->name);
                if (prev && !s->setting_committed && entry != s->setting_entry.end() && entry->second != r->value)
                    put(prev, entry->second);
                put(pinch, r->value);
            }
        }
    // Records menu: the ranking rows (the "record_bar" template and the hidden
    // record_* text elements) in the "menu_records" window.
    if (page == "records_free_tt" && !ctl.items().empty() && ctl.items()[0].element) {
        const Element& bar = *ctl.items()[0].element;
        const Element* win = find_widget_named(ref.layout(), "menu_records");
        auto it = s->gt().state().rankings.find(s->records_key);
        if (win && it != s->gt().state().rankings.end()) {
            const bool speed = s->records_key == "mt:MMAX";
            const int rows = std::max(1, int(win->h / bar.h));
            for (int i = 0; i < int(it->second.size()) && i < rows; ++i) {
                const RankEntry& r = it->second[size_t(i)];
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
                char buf[64];
                if (speed) std::snprintf(buf, sizeof buf, "%.1f km/h", r.value);
                else std::snprintf(buf, sizeof buf, "%s", format_time_ms(u32(std::llround(r.value * 1000))).c_str());
                put(hidden_element(ref.layout(), "record_rank_shadow"), std::to_string(i + 1));
                put(hidden_element(ref.layout(), "record_rank"), std::to_string(i + 1));
                put(hidden_element(ref.layout(), "record_name"),
                    r.name.empty() ? s->gt().state().player_name : s->gt().record_name(r.name));
                // A seeded record has no car: the original's placeholder car name (30 dashes).
                put(hidden_element(ref.layout(), "record_car"), r.car.empty() ? std::string(30, '-') : car_name(s->gt(), r.car));
                put(hidden_element(ref.layout(), "record_data"), buf);
                put(hidden_element(ref.layout(), "record_date"), "day " + std::to_string(r.day));
            }
        }
    }
    // Race information -> Cars Permitted: one row per permitted car, the "link_menu" template
    // ("active_menu" on the cursor row) in the "menu_regulated_car" window; without a list
    // the hidden "No Limitations" (in its own place, the middle of the box: M).
    if (page == "00cmn/car_reguration") {
        const std::vector<std::string> names = permitted_car_names(s->gt(), s->race_code);
        GarageRows g;
        if (names.empty()) {
            if (const Element* none = hidden_element(ref.layout(), "no_regulated_car")) {
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = none;
                d.z = none->z;
                out.push_back(d);
            }
        } else if (regulated_rows(ctl, *s, int(names.size()), g)) {
            for (int i = g.first; i < g.n && i < g.first + g.rows; ++i) {
                const Element* e = i == s->regulated_cursor && !g.tmpl->alt.empty() ? &g.tmpl->alt[0] : g.tmpl;
                menu::ElementDraw d;
                d.scene = ref.scene;
                d.element = e;
                d.has_box = true;
                d.box = {g.win->x, g.y(i), g.tmpl->w, g.tmpl->h};
                d.z = g.tmpl->z;
                d.text = std::make_shared<const std::string>(names[size_t(i)]);
                d.text_dx = menu::kTextInset; // left-aligned text 12 units in (menus.md §5 "Text inset")
                out.push_back(d);
            }
        }
    }
    // Wheel shop: the brand page's wheels as tiles in a row in the "menu_wheel_shop" window
    // (WheelShopButton: each tile is the page's hidden image named after the wheel, 52 x 52,
    // with "sold_out" over it for a wheel already owned; the reference: tiles 56.5 units
    // apart, centred while they fit, the focused one washed light grey).
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
            const std::vector<std::string>& inv = s->gt().state().wheels;
            if (!code.empty() && std::find(inv.begin(), inv.end(), code) != inv.end()) image(sold, win->z + 0.002f);
        }
    }
    // Image widgets of the layers currently shown.
    for (size_t li = 0; li < ref.layout().layers.size(); ++li) {
        if (!ctl.layer_visible(int(li))) continue;
        for (const Element& w : ref.layout().layers[li].widgets) widget_draws(ctl, w, out);
    }
    // Dealer car page: one swatch per paint of the model in a column ending one step above
    // the "car_color_box" widget, the first on top, 12.5 px apart, each over a soft drop
    // shadow (+3, +3); the chosen one is marked by a white 8 x 10 square 4 px to its left
    // (the reference screens, measured).
    if (starts(page, "cd/") && page.size() > 4 && page.compare(page.size() - 4, 4, "_car") == 0) {
        const Element* box = find_widget_named(ref.layout(), "car_color_box");
        const GtData& data = s->gt().data();
        int n = data.color_count(s->focus_car);
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
            u32 rgb = data.color_rgb(data.color_id(s->focus_car, i));
            // (Layout coordinates: y grows upwards.)
            float x = box->x, y = box->y + float(n - i) * step; // first paint on top
            fill({x + 2, y - 4, box->w + 2, box->h + 2}, 0, 0.35f, box->z - 0.002f);
            fill({x + 3, y - 3, box->w, box->h}, 0, 0.75f, box->z - 0.001f);
            if (i == s->color_cursor) fill({x - 12, y, 8, box->h}, 1, 1, box->z);
            menu::ElementDraw d;
            d.kind = menu::ElementDraw::Kind::Fill;
            d.has_box = true;
            d.box = {x, y, box->w, box->h};
            d.fill = {float(rgb & 0xFF) / 255.0f, float((rgb >> 8) & 0xFF) / 255.0f,
                      float((rgb >> 16) & 0xFF) / 255.0f, 1};
            d.z = box->z;
            out.push_back(d);
        }
        return;
    }
    // Garage list (code-drawn): rows of the "car1" template in the "menu_garage"
    // window, in the chosen sort order, with the sort column header and values.
    if ((page != "hm/hm_gr" && page != "hm/hm_gr_trade") || ctl.depth() != 0) return;
    if (ctl.items().empty() || !ctl.items()[0].element) return;
    const Element& tmpl = *ctl.items()[0].element;
    const Element* win = find_widget_named(ref.layout(), page == "hm/hm_gr_trade" ? "menu_garage_trade" : "menu_garage");
    if (!win) return;
    // Date / name order show the MAKE column (as the original's new-game garage does).
    static const char* const kHeader[] = {"maker", "maker", "maker", "color", "price", "distance", "power", "drive", "aspiration"};
    const int key = std::clamp(s->sort_key, 0, 8);
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
    GtMode& lg = s->list_gt(); // the guest garage during a trade
    const GameState& st = lg.state();
    const std::vector<int> order = lg.garage_order(s->sort_key);
    const float row = tmpl.h + 2;
    const int rows = std::max(1, int(win->h / row));
    const int n = int(order.size());
    int first = std::clamp(s->garage_cursor - rows / 2, 0, std::max(0, n - rows));
    for (int i = first; i < n && i < first + rows; ++i) {
        const GarageCar& car = st.garage[size_t(order[size_t(i)])];
        const bool focus = i == s->garage_cursor;
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
        if (!s->trade && order[size_t(i)] == st.ride)
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
        std::string v = lg.sort_value(car, s->sort_key < 2 ? 2 : s->sort_key);
        for (char& ch : v)
            if (s->sort_key < 3) ch = char(std::toupper(static_cast<unsigned char>(ch))); // "MAZDA"
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

// ---------------------------------------------------------------------------

GtModeSession::GtModeSession(std::shared_ptr<const GtData> data, std::string path)
    : gt_(std::move(data)), save_path_(std::move(path)) {}

bool GtModeSession::save(std::string* error) { return write_save(save_path_, gt_.state(), error); }

bool GtModeSession::load(std::string* error) {
    std::optional<GameState> s = read_save(save_path_, error);
    if (!s) return false;
    gt_.state() = std::move(*s);
    return true;
}

GtModeSession* gt_mode_session() { return g_session.get(); }

namespace {
std::unique_ptr<SaveLibrary> g_library;
std::optional<GameState> g_snapshot; // begin_snapshot_save
} // namespace

SaveLibrary& save_library() {
    // Made again when GT3R_GTMODE_SAVE changes (tests).
    static std::string made_for;
    const char* env = std::getenv("GT3R_GTMODE_SAVE");
    const std::string want = env ? env : "";
    if (g_library && want != made_for) g_library.reset();
    made_for = want;
    if (!g_library) {
        if (const char* p = env; p && *p) {
            std::string dir = p;
            size_t sl = dir.find_last_of("/\\");
            dir = sl == std::string::npos ? std::string(".") : dir.substr(0, sl);
            g_library = std::make_unique<SaveLibrary>(dir, false);
        } else {
            g_library = std::make_unique<SaveLibrary>(SaveLibrary::user_dir());
            if (saving_enabled())
                if (int n = g_library->migrate(SaveLibrary::legacy_path()); n > 0)
                    LOG_INFO("gt mode: %d save(s) copied into %s", n, g_library->dir().c_str());
        }
    }
    return *g_library;
}

std::string current_save_path() {
    if (const char* p = std::getenv("GT3R_GTMODE_SAVE"); p && *p) return p;
    SaveLibrary& lib = save_library();
    if (std::string c = lib.current(); !c.empty()) return c;
    for (const SaveInfo& f : lib.list())
        if (f.state) return f.path;
    // Nothing in the library yet: an earlier version's save when it was not migrated
    // (writes disabled), else the new game's file.
    if (!saving_enabled() && host_path_exists(SaveLibrary::legacy_path())) return SaveLibrary::legacy_path();
    return lib.dir() + "gtmode.sav";
}

void begin_snapshot_save(GameState s) { g_snapshot = std::move(s); }

void save_race_run(const GameResult& r) {
    GtModeSession* s = S();
    if (!s || !saving_enabled() || r.mode != "go_race" || !r.finished || r.position <= 0) return;
    if (s->gt().state().championship.active()) return;
    GtMode copy(s->gt().data_ptr());
    copy.state() = s->gt().state();
    copy.record_race(r.event.empty() ? r.race_code : r.event, r.position - 1);
    std::string err;
    if (!write_save(s->save_path(), copy.state(), &err)) LOG_WARN("gt mode: save failed: %s", err.c_str());
    else LOG_INFO("gt mode: race %s place %d saved", r.race_code.c_str(), r.position);
}

bool begin_race_game_save(const GameResult& r) {
    GtModeSession* s = S();
    if (!s) return false;
    GtMode copy(s->gt().data_ptr());
    copy.state() = s->gt().state();
    // What on_result() will record for the round: the distance, the car's dirt, the place.
    if (r.distance_km > 0) copy.drive(copy.state().ride, r.distance_km * 1000.0);
    if (r.car_dirt >= 0)
        if (GarageCar* ride = copy.state().ride_car()) ride->cond.dirt = float(std::clamp(r.car_dirt, 0.0, 0.8));
    if (r.mode == "go_race" && copy.state().championship.active() && !r.abandoned) {
        std::vector<int> order;
        for (int o : r.order) order.push_back(o + 1);
        copy.record_round(r.finished && r.position > 0 ? r.position - 1 : -1, order);
    }
    begin_snapshot_save(std::move(copy.state()));
    return true;
}

void end_race_game_save() { g_snapshot.reset(); }

const GameState* snapshot_save() { return g_snapshot ? &*g_snapshot : nullptr; }

void save_license_run(const std::string& code, int medal, double seconds) {
    GtModeSession* s = S();
    if (!s || medal < 0 || !saving_enabled()) return;
    // A copy: the live state is recorded when the result reaches the menus.
    GtMode copy(s->gt().data_ptr());
    copy.state() = s->gt().state();
    copy.record_license_medal(code, Medal(std::min(medal, 2)), seconds);
    std::string err;
    if (!write_save(s->save_path(), copy.state(), &err)) LOG_WARN("gt mode: save failed: %s", err.c_str());
    else LOG_INFO("gt mode: license %s %s saved", code.c_str(), medal_name(Medal(std::min(medal, 2))));
}

bool queue_race_prize(const std::string& code, int place) {
    GtModeSession* s = S();
    if (!s || place < 0 || place >= 6 || s->gt().state().championship.active()) return false;
    auto rr = s->gt().rules(code);
    if (!rr) return false;
    std::optional<Show> sh = race_prize_show(s->gt(), code, place, s64(rr->prize[size_t(place)]));
    if (!sh) return false;
    sh->credited = false; // the race's result adds it when the race screen closes
    g_shows.push_back(std::move(*sh));
    return true;
}

int add_license_record(const std::string& code, double seconds) {
    GtModeSession* s = S();
    if (!s || seconds <= 0) return -1;
    GtMode& gt = s->gt();
    std::string test_car = code;
    if (auto rr = gt.rules(code); rr && !rr->entries.empty())
        if (const data::CarVariant* v = gt.db().variant(rr->entries[0])) test_car = v->car;
    int rank = gt.add_record("lic:" + code, seconds, test_car, true, 6);
    if (rank >= 0) g_shows.push_back({"name_entry", "name_entry", {}, "", "", 0, {}, "lic:" + code, rank, "", 0, "", true, {}});
    return rank;
}

bool ensure_gt_mode_session(const std::function<std::optional<Bytes>(const std::string&)>& read) {
    if (!g_reader) g_reader = read;
    if (g_session) return true;
    if (!g_data) {
        auto d = std::make_shared<GtData>();
        if (!d->load(read)) return false;
        g_data = d;
    }
    g_session = std::make_unique<GtModeSession>(g_data, save_path());
    std::string err;
    if (!g_session->load(&err)) LOG_INFO("gt mode: new game (%s)", err.c_str());
    return true;
}

bool begin_records(const std::string& key) {
    GtModeSession* s = S();
    if (!s) return false;
    s->records_key = key;
    return true;
}

bool begin_setting(const std::string& event) {
    GtModeSession* s = S();
    if (!s || !s->gt().state().ride_car()) return false;
    s->setting_work = *s->gt().state().ride_car();
    s->setting_event = event;
    s->setting_entry.clear();
    s->setting_committed = false;
    return true;
}


const GarageCar* arcade_garage_car(const std::string& entry) {
    GtModeSession* s = S();
    if (!s || entry.size() < 3 || (entry.compare(0, 2, "hg") != 0 && entry.compare(0, 2, "gg") != 0)) return nullptr;
    const GtMode* g = entry[0] == 'h' ? &s->gt() : s->guest.get();
    if (!g) return nullptr;
    int i = std::atoi(entry.c_str() + 2);
    const auto& garage = g->state().garage;
    return i >= 0 && i < int(garage.size()) ? &garage[size_t(i)] : nullptr;
}

std::string arcade_entry_car(const std::string& entry) {
    GtModeSession* s = S();
    if (!s) return {};
    if (const GarageCar* gc = arcade_garage_car(entry)) return gc->car;
    const GtMode& gt = s->gt();
    auto base_of = [&](const std::string& label) {
        for (const data::CarVariant& v : gt.db().arcade_cars())
            if (v.label == label) return v.car;
        return gt.db().car(label) ? label : std::string();
    };
    // Time trials ("tt01"): the challenge's CLASS_COMPULSORY car.
    if (int i = GtMode::time_trial_index(entry); i >= 0)
        if (auto cls = gt.data().arcade_classes.find("CLASS_COMPULSORY");
            cls != gt.data().arcade_classes.end() && i < int(cls->second.size()))
            return base_of(cls->second[size_t(i)].label);
    for (const auto& [cls, cars] : gt.data().arcade_classes)
        for (const GtData::ArcadeCar& c : cars)
            if (c.image() == entry)
                for (const data::CarVariant& v : gt.db().arcade_cars())
                    if (v.label == c.label) return v.car;
    return gt.db().car(entry) ? entry : std::string();
}

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

std::string presentation_model(const std::string& page, int* serial, float* fade) {
    if (serial) *serial = g_show_serial;
    if (fade) *fade = 0;
    if (!g_show || g_show->page != page || g_show->model.empty()) return {};
    // The model fades in; its animation starts with the fade.
    int from = page == "trophy_get" ? kTrophyFadeIn : is_mini_prize(page) ? 0 : kLicFadeIn;
    int to = page == "trophy_get" ? kTrophyFadeEnd : is_mini_prize(page) ? kMiniFadeFrames : kLicFadeEnd;
    if (g_show_frames < from) return {};
    if (fade) *fade = std::min(1.0f, float(g_show_frames - from) / float(to - from));
    return g_show->model;
}

bool car_for_widget(const std::string& page, const std::string& cls, std::string& car, int& paint,
                    std::string* wheel, float* dirt) {
    GtModeSession* s = S();
    if (!s) return false;
    if (wheel) wheel->clear();
    if (dirt) *dirt = 0;
    if (cls == "render_model" && page == "car_get" && g_show) {
        // Roulette: the set's candidates flash by for 2.5 s, then the car won.
        const auto& c = g_show->candidates;
        if (c.size() > 1 && g_show_frames < 150) {
            car = c[size_t(g_show_frames / 6) % c.size()];
            paint = 0;
        } else {
            car = g_show->car;
            paint = g_show->paint;
        }
        return true;
    }
    if (!starts(cls, "car_render_")) return false;
    const GameState& st = s->gt().state();
    if (starts(page, "cd/") || cls == "car_render_from" || cls == "car_render_buy") {
        if (s->focus_car.empty()) return false;
        car = s->focus_car;
        paint = s->color_cursor;
        return true;
    }
    const GarageCar* g = nullptr;
    const GameState& lst = s->list_gt().state(); // the guest garage during a trade
    if (starts(page, "hm/") && s->garage_focus >= 0 && s->garage_focus < int(lst.garage.size()))
        g = &lst.garage[size_t(s->garage_focus)];
    if (!g) g = st.ride_car();
    if (!g) return false;
    car = g->car;
    paint = g->color;
    if (dirt) *dirt = std::clamp(float(g->cond.dirt), 0.0f, 0.8f);
    if (wheel) {
        *wheel = g->wheel;
        // Wheel shop: the wheel being looked at goes on the car (a preview).
        if (starts(page, "at/ws/ws_") && cls == "car_render_wheel_shop")
            if (GtModeSession* ss = S(); ss && !ss->focus_wheel_preview.empty()) *wheel = ss->focus_wheel_preview;
        // Garage wheel change page: the inventory wheel under the cursor.
        if (page == "hm/hm_wheel") {
            const auto& w = st.wheels;
            *wheel = s->wheel_cursor >= 0 && s->wheel_cursor < int(w.size()) ? w[size_t(s->wheel_cursor)] : "";
        }
    }
    return true;
}

namespace {

std::string date_text(s64 t) {
    if (t <= 0) return {};
    std::time_t tt = std::time_t(t);
    std::tm* tm = std::localtime(&tt);
    if (!tm) return {};
    char b[32];
    std::snprintf(b, sizeof b, "%02d/%02d/%04d", tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900);
    return b;
}

using Texts = std::vector<std::pair<std::string, std::string>>;

Texts setting_file_texts(const GtMode& gt, const SettingFile& f) {
    Texts t;
    t.push_back({"s_pt_car", car_name(gt, f.car)});
    t.push_back({"s_pt_comment", f.comment});
    t.push_back({"s_pt_date", date_text(f.time)});
    if (f.power_ps > 0)
        t.push_back({"s_pt_power", gt.power_text(float(f.power_ps))});
    return t;
}

Texts game_file_texts(const GtMode& gt, const std::string& path) {
    const GameState& st = gt.state();
    Texts t;
    static const char* const kLic[] = {"B", "A", "IB", "IA", "S"};
    int lic = gt.highest_license();
    t.push_back({"g_pt_license", lic >= 0 && lic < 5 ? kLic[lic] : ""});
    t.push_back({"g_pt_cars", format_credits(s64(st.garage.size()))});
    t.push_back({"g_pt_g_achievement", GtMode::completion_file_text(gt.completion_percent())});
    // The native file's size in KB (rounded up; the original shows its card file's).
    if (std::optional<Bytes> b = read_host_file(path)) t.push_back({"g_pt_size", std::to_string((b->size() + 1023) / 1024) + "KB"});
    s64 mtime = 0;
    if (std::optional<s64> m = host_file_mtime(path)) mtime = *m;
    t.push_back({"g_pt_date", date_text(mtime)});
    t.push_back({"g_pt_day", std::to_string(st.day)});
    t.push_back({"g_pt_money", format_credits(st.credits)});
    return t;
}

// --- The save page (menu/flow/save_page.h) ---------------------------------------

using SavePage = menu::flow::SavePage;

bool same_path(const std::string& a, const std::string& b) {
    std::string x = a, y = b;
    std::replace(x.begin(), x.end(), '\\', '/');
    std::replace(y.begin(), y.end(), '\\', '/');
    return x == y;
}

int read_result(const std::string& err) {
    return err.find("version") != std::string::npos ? SavePage::BadData : SavePage::ReadError;
}

std::vector<SavePage::File> save_page_files(SavePage::Mode m) {
    std::vector<SavePage::File> out;
    GtModeSession* s = S();
    if (!s) return out;
    if (m == SavePage::Mode::LoadSettings || m == SavePage::Mode::SaveSettings) {
        // The current game's setting files, newest first (Load: only the ride car's).
        GarageCar* real = setting_real();
        if (m == SavePage::Mode::SaveSettings) {
            SavePage::File n;
            n.kind = SavePage::File::Kind::New;
            out.push_back(n);
        }
        const auto& files = s->gt().state().setting_files;
        for (int i = int(files.size()) - 1; i >= 0; --i) {
            const SettingFile& sf = files[size_t(i)];
            const bool own = real && sf.car == real->car;
            if (m == SavePage::Mode::LoadSettings && !own) continue;
            SavePage::File f;
            f.kind = SavePage::File::Kind::Setting;
            f.id = std::to_string(i);
            f.own = own;
            f.name = sf.comment;
            f.texts = setting_file_texts(s->gt(), sf);
            out.push_back(std::move(f));
        }
        return out;
    }
    SaveLibrary& lib = save_library();
    if (m == SavePage::Mode::LoadGame || m == SavePage::Mode::SaveGame) {
        SavePage::File n;
        n.kind = SavePage::File::Kind::New;
        n.name = lib.unique_name("GAME");
        out.push_back(n);
    }
    for (SaveInfo& info : lib.list()) {
        SavePage::File f;
        f.id = info.path;
        f.current = same_path(info.path, s->save_path());
        if (f.current && m == SavePage::Mode::LoadGuest) continue; // the home garage already
        f.name = info.name;
        f.broken = !info.state;
        if (info.state) {
            // The game being played as it is now (its file may be a race behind), or the
            // snapshot a save page was opened for (begin_snapshot_save).
            GtMode view(s->gt().data_ptr());
            view.state() = f.current ? (g_snapshot ? *g_snapshot : s->gt().state()) : std::move(*info.state);
            f.texts = game_file_texts(view, info.path);
        }
        out.push_back(std::move(f));
    }
    return out;
}

// Ends in ".psu" / ".ps2" / ".mcd": that container; a path without an extension (a save
// dialog that adds none) gets ".psu"; Android's content:// URIs are written as they are.
std::string export_target(const std::string& out, bool* card) {
    auto ends = [&](const char* ext) {
        size_t n = std::strlen(ext);
        if (out.size() < n) return false;
        for (size_t i = 0; i < n; ++i)
            if (std::tolower(u8(out[out.size() - n + i])) != ext[i]) return false;
        return true;
    };
    *card = ends(".ps2") || ends(".mcd");
    if (*card || ends(".psu") || out.rfind("content://", 0) == 0) return out;
    size_t sl = out.find_last_of("/\\");
    std::string base = sl == std::string::npos ? out : out.substr(sl + 1);
    return base.find('.') == std::string::npos ? out + ".psu" : out;
}

int save_page_run(SavePage::Mode m, SavePage::Op op, const SavePage::File& f, const std::string& arg) {
    GtModeSession* s = S();
    if (!s) return SavePage::ReadError;
    GameState& live = s->gt().state();
    const bool write = saving_enabled();
    // --- Setting files of the current game.
    if (f.kind == SavePage::File::Kind::Setting || m == SavePage::Mode::SaveSettings ||
        m == SavePage::Mode::LoadSettings) {
        GarageCar* real = setting_real();
        const int idx = f.kind == SavePage::File::Kind::Setting ? std::atoi(f.id.c_str()) : -1;
        auto& files = live.setting_files;
        const bool valid = idx >= 0 && idx < int(files.size());
        switch (op) {
        case SavePage::OpLoad: {
            if (!real || !valid) return SavePage::ReadError;
            int miss = apply_setting_file(files[size_t(idx)], *real);
            if (miss < 0) return SavePage::BadData;
            s->setting_work = *real;
            if (miss > 0) g_popup = "load_warning";
            break;
        }
        case SavePage::OpSave:
        case SavePage::OpOverwrite:
            if (!real) return SavePage::WriteError;
            write_setting_file(live, *real, op == SavePage::OpSave ? -1 : idx, s64(std::time(nullptr)),
                               int(s->gt().power_ps(*real))); // the computed PS, truncated
            break;
        case SavePage::OpDelete:
            if (!delete_setting_file(live, idx)) return SavePage::WriteError;
            break;
        case SavePage::OpRename:
            if (!valid) return SavePage::WriteError;
            files[size_t(idx)].comment = arg;
            break;
        case SavePage::OpCopy:
            if (!valid) return SavePage::WriteError;
            files.push_back(files[size_t(idx)]);
            break;
        default: return SavePage::WriteError;
        }
        std::string err;
        if (write && !s->save(&err)) {
            LOG_WARN("gt mode: save failed: %s", err.c_str());
            return SavePage::WriteError;
        }
        return SavePage::Ok;
    }
    // --- Saves.
    SaveLibrary& lib = save_library();
    std::string err;
    auto save_live = [&]() {
        if (write && !s->save(&err)) LOG_WARN("gt mode: save failed: %s", err.c_str());
    };
    auto play = [&](const std::string& path) {
        s->set_save_path(path);
        lib.set_current(path);
    };
    switch (op) {
    case SavePage::OpLoad: {
        if (m == SavePage::Mode::LoadGuest) {
            // A friend's game: any save file, native or PS2 (the guest garage only).
            const std::string path = f.kind == SavePage::File::Kind::Open ? arg : f.id;
            try {
                GameState st = read_any_save(path, s->gt().data());
                s->guest = std::make_unique<GtMode>(s->gt().data_ptr());
                s->guest->state() = std::move(st);
            } catch (const std::exception& e) {
                LOG_WARN("gt mode: guest load failed: %s", e.what());
                return read_result(e.what());
            }
            return SavePage::Ok;
        }
        std::optional<GameState> st = read_save(f.id, &err);
        if (!st) {
            LOG_WARN("gt mode: load failed: %s", err.c_str());
            return read_result(err);
        }
        save_live(); // the game being left keeps its progress
        live = std::move(*st);
        s->setting_work.reset();
        play(f.id);
        LOG_INFO("gt mode: loaded %s", f.id.c_str());
        return SavePage::Ok;
    }
    case SavePage::OpNewGame: {
        save_live();
        const std::string name = lib.unique_name(arg);
        s->gt().new_game();
        live.file_comment = name;
        s->setting_work.reset();
        play(lib.path_for(name));
        save_live();
        return SavePage::Ok;
    }
    case SavePage::OpSave:
    case SavePage::OpOverwrite: {
        // The running game, or the snapshot offered by begin_snapshot_save (which leaves the
        // running game and its save alone).
        const bool snap = g_snapshot.has_value();
        GameState st = snap ? *g_snapshot : live;
        std::string path;
        if (op == SavePage::OpSave) {
            const std::string name = lib.unique_name(arg);
            if (write && !lib.create(st, name, &path, &err)) {
                LOG_WARN("gt mode: save failed: %s", err.c_str());
                return SavePage::WriteError;
            }
            if (!write) path = lib.path_for(name);
            st.file_comment = name;
        } else {
            path = f.id;
            st.file_comment = f.name;
            if (write && !write_save(path, st, &err)) {
                LOG_WARN("gt mode: save failed: %s", err.c_str());
                return SavePage::WriteError;
            }
        }
        if (!snap) {
            live.file_comment = st.file_comment;
            play(path);
        }
        LOG_INFO("gt mode: saved %s", path.c_str());
        return SavePage::Ok;
    }
    case SavePage::OpRename: {
        if (!write) return SavePage::Ok;
        const std::string name = lib.unique_name(arg, SavePage::kNameMax, f.id);
        std::string np;
        if (f.current) {
            live.file_comment = name;
            if (!lib.rename(f.id, name, &np, &live, &err)) return SavePage::WriteError;
            play(np);
        } else if (!lib.rename(f.id, name, &np, nullptr, &err)) {
            return SavePage::WriteError;
        }
        return SavePage::Ok;
    }
    case SavePage::OpCopy: {
        if (!write) return SavePage::Ok;
        if (f.current) save_live();
        std::string np;
        return lib.copy(f.id, &np, &err) ? SavePage::Ok : SavePage::WriteError;
    }
    case SavePage::OpDelete:
        if (f.current) return SavePage::WriteError; // the page never offers it
        if (!write) return SavePage::Ok;
        return lib.remove(f.id) ? SavePage::Ok : SavePage::WriteError;
    case SavePage::OpExport: {
        // To the PS2: a .psu or a memory-card image (PCSX2 .ps2) holding this save.
        std::optional<GameState> st;
        if (f.current) st = live;
        else st = read_save(f.id, &err);
        if (!st) return read_result(err);
        bool card = false;
        const std::string out = export_target(arg, &card);
        try {
            Bytes b = export_ps2_save_container(*st, s->gt().data_ptr(), g_reader, card);
            if (!write_host_file(out, b)) throw FormatError("cannot write " + out);
        } catch (const std::exception& e) {
            LOG_WARN("gt mode: export failed: %s", e.what());
            return SavePage::WriteError;
        }
        LOG_INFO("gt mode: exported %s", out.c_str());
        return SavePage::Ok;
    }
    case SavePage::OpImport: {
        // Another save into the library: a native save from elsewhere or a PS2 save.
        GameState st;
        try {
            st = read_any_save(arg, s->gt().data());
        } catch (const std::exception& e) {
            LOG_WARN("gt mode: import failed: %s", e.what());
            return read_result(e.what());
        }
        std::string base = st.file_comment;
        if (base.empty()) {
            size_t sl = arg.find_last_of("/\\");
            base = arg.rfind("content://", 0) == 0 ? std::string() : arg.substr(sl == std::string::npos ? 0 : sl + 1);
            if (size_t dot = base.rfind('.'); dot != std::string::npos && dot > 0) base.resize(dot);
            if (base.empty() || base == kPs2GameDir) base = "PS2 GAME";
        }
        if (!write) return SavePage::Ok;
        std::string np;
        return lib.create(std::move(st), lib.unique_name(base), &np, &err) ? SavePage::Ok : SavePage::WriteError;
    }
    }
    return SavePage::WriteError;
}

} // namespace

void install_mod_menu_patches(); // gt_mode_mods.cpp

void install_gt_mode_menu() {
    install_mod_menu_patches();
    menu::flow::GtModeHooks h;
    h.attach = [](const menu::flow::FileReader& read) {
        g_reader = read;
        if (!g_data) {
            auto d = std::make_shared<GtData>();
            if (!d->load(read)) {
                LOG_WARN("gt mode: no game data: %s", d->error().c_str());
                return;
            }
            g_data = d;
        }
        g_session = std::make_unique<GtModeSession>(g_data, save_path());
        // Development aid (screenshots, tests of the result path): every race is won at once.
        if (const char* fake = std::getenv("GT3R_GTMODE_FAKE_RACE"); fake && *fake && *fake != '0')
            menu::flow::set_game_request_handler([](const GameRequest& rq, App&) {
                GameResult res;
                res.mode = rq.mode;
                res.race_code = rq.race_code;
                res.event = rq.event;
                res.finished = true;
                res.position = 1;
                res.cars = int(rq.opponents.size()) + 1;
                res.total_time = 60;
                res.distance_km = 3;
                if (rq.mode == "go_license") res.license_medal = 0; // gold
                menu::flow::post_game_result(res);
                return true;
            });
        std::string err;
        if (g_session->load(&err)) LOG_INFO("gt mode: loaded %s", g_session->save_path().c_str());
        else LOG_INFO("gt mode: new game (%s)", err.c_str());
    };
    h.on_event = [](const MenuEvent& ev, std::vector<std::string>&) {
        std::vector<std::string> lines;
        bool consumed = on_event(ev, lines);
        to_log_file(lines);
        return consumed;
    };
    h.intercept = intercept;
    h.enabled = enabled;
    h.widget_text = widget_text;
    // The prize screen's "render_model" marker: its place shows the 3D car instead.
    h.hide_art = [](const std::string& page, const Element& el) {
        if (!el.widget_class) return false;
        const std::string& c = *el.widget_class;
        if ((page == "car_get" || page == "license_get" || page == "trophy_get" || is_mini_prize(page)) &&
            c == "render_model")
            return true;
        // The car renders' own image is the showroom's reflection map (car_select_env), not
        // art to draw: the 3D car is drawn in its place.
        if (starts(c, "car_render_")) return true;
        // The paint column is drawn by code (extra_draws); the widget's own white box is
        // not shown (reference screens).
        if (c == "car_color_box") return true;
        // Arcade Clear Status: the end button / message once a GT league is complete
        // or every arcade course is open.
        if (page == "ar/goodies" && (c == "end_button" || c == "end_message") && S()) {
            const GtMode& gt = S()->gt();
            return !(gt.league_complete("beginner") || gt.league_complete("amateur") || gt.league_complete("pro") ||
                     gt.state().arcade.course_level >= 6);
        }
        // Settings menu: "attach_*" marks the fitted part, "disable_*" covers a locked setting.
        if (starts(c, "attach_") && S())
            if (GarageCar* work = setting_work()) return !setting_attached(S()->gt(), *work, c);
        if (starts(c, "disable_") && S())
            if (GarageCar* real = setting_real()) return !setting_locked(S()->gt(), *real, c);
        return false;
    };
    h.on_frame = on_frame;
    h.map_input = map_input;
    h.prepare_request = prepare_request;
    h.on_result = on_result;
    h.next_request = next_request;
    h.extra_draws = extra_draws;
    h.tap = garage_tap;
    h.hover = garage_hover;
    h.take_popup = []() { return std::exchange(g_popup, std::string()); };
    // Record name entry: Triangle / Square deletes the last character (never leaves the page).
    h.name_backspace = []() {
        GtModeSession* s = S();
        RankEntry* r = named_entry();
        if (!s || !r || r->name.empty()) return false;
        r->name.pop_back();
        s->name_started = true;
        s->gt().state().player_name = r->name;
        return true;
    };
    h.take_movie = []() { return std::exchange(g_movie, std::string()); };
    // Attract mode: the title idle time (gcf section 0) and the demo races (gcf
    // section 12, cycling; the F1 demos only once the S license is held).
    h.attract_idle_frames = []() { return g_data ? g_data->title_idle_seconds * 60 : 0; };
    h.attract_demo = []() -> std::optional<GameRequest> {
        static size_t index = 0;
        if (!g_data || g_data->demos.empty()) return std::nullopt;
        bool s_license = S() && S()->gt().license_held(4);
        for (size_t tries = 0; tries < g_data->demos.size(); ++tries) {
            index = (index + 1) % g_data->demos.size();
            const GtData::Demo& d = g_data->demos[index];
            if (d.level > (s_license ? 1 : 0)) continue;
            GameRequest r;
            r.mode = "attract_demo";
            r.race_code = r.event = d.event;
            r.course = d.course;
            r.car = d.car; // "" for every demo on the disc: the pool fills slot 0 too
            r.page = "root";
            return r;
        }
        return std::nullopt;
    };
    h.on_arcade_result = [](const GameResult& r) {
        GtModeSession* s = S();
        if (!s || !r.finished) return;
        GtMode& gt = s->gt();
        // Single Race: the best place per difficulty and course, and the unlocks.
        if (r.mode == "single_race" && r.position > 0) {
            GtMode::ArcadeOutcome o = gt.arcade_single_race(r.race_code, r.course, r.position - 1);
            // A win: the difficulty's cup (race/a_<difficulty>_gold.prm),
            // OK after 3 s (presentation.md §F; the original shows it for a win only).
            if (r.position == 1 && r.race_code.size() > 1) {
                const char* cls = r.race_code[1] == 'E' ? "easy" : r.race_code[1] == 'N' ? "normal"
                                  : r.race_code[1] == 'H' ? "hard" : r.race_code[1] == 'P' ? "special" : nullptr;
                if (cls) {
                    Show sh{};
                    sh.set = sh.page = "arcade_mini";
                    sh.model = std::string("race/a_") + cls + "_gold.prm";
                    sh.rank = ordinal(0);
                    g_shows.push_back(std::move(sh));
                }
            }
            if (o.courses_opened > 0) LOG_INFO("arcade: course level %d", gt.state().arcade.course_level);
            for (int l : o.car_levels) LOG_INFO("arcade: cars of level %d unlocked", l);
        }
        // Time trials: the per-challenge ranking (race time) and its best rank.
        if (r.mode == "time_trial") {
            std::string code = r.race_code;
            if (GtMode::time_trial_index(code) < 0)
                if (int i = GtMode::time_trial_index(r.course); i >= 0) {
                    char b[24];
                    std::snprintf(b, sizeof b, "AT%02d0001", i + 1);
                    code = b;
                }
            double t = r.total_time > 0 ? r.total_time : r.best_lap;
            if (int i = GtMode::time_trial_index(code); i >= 0) {
                // The challenge's car (CLASS_COMPULSORY entry i), as its base car for the records.
                std::string car;
                if (auto cls = gt.data().arcade_classes.find("CLASS_COMPULSORY");
                    cls != gt.data().arcade_classes.end() && i < int(cls->second.size())) {
                    car = cls->second[size_t(i)].label;
                    for (const data::CarVariant& v : gt.db().arcade_cars())
                        if (v.label == car) car = v.car;
                }
                // A time that makes the ranking asks for a name, as in GT mode.
                if (int rank = gt.arcade_time_trial(code, t, car); rank >= 0)
                    g_shows.push_back({"name_entry", "name_entry", {}, "", "", 0, {}, "at:" + code, rank, "", 0, "", true, {}});
            }
        }
        if (saving_enabled()) s->save();
    };
    // Arcade picker art filled by code: the course's area (group) badge, dirt / tarmac
    // badge and length; the car's drive badge and power / torque.
    h.arcade_parts = [](const std::string& page, const std::string& entry) {
        std::vector<std::pair<std::string, std::string>> out;
        GtModeSession* s = S();
        if (!s) return out;
        const GtMode& gt = s->gt();
        if (page == "ar/select_course") {
            if (int l = gt.arcade_course_level(entry); l >= 0 && l < 6) out.push_back({std::string("area_") + char('a' + l), ""});
            out.push_back({gt.arcade_course_dirt(entry) ? "dirt" : "tarmac", ""});
            u32 len = 0;
            for (const auto& [list, courses] : gt.data().arcade_courses)
                for (const GtData::ArcadeCourse& c : courses)
                    if (c.course == entry) len = c.length_m;
            out.push_back({"total_length", ""});
            char b[32];
            // NTSC-U: miles with three decimals ("1.494miles"); else metres.
            if (!len) std::snprintf(b, sizeof b, "----");
            else if (gt.data().imperial) std::snprintf(b, sizeof b, "%.3fmiles", double(len) / 1609.344);
            else std::snprintf(b, sizeof b, "%4um", len);
            out.push_back({"length", b});
        } else if (page == "ar/select_compulsory") {
            // Time trial: the records (top three: the ranking, else the race record's
            // defaults), the car's power and weight.
            int i = GtMode::time_trial_index(entry);
            if (i < 0) return out;
            char code[24];
            std::snprintf(code, sizeof code, "AT%02d0001", i + 1);
            std::vector<double> t;
            if (auto it = gt.state().rankings.find(std::string("at:") + code); it != gt.state().rankings.end())
                for (const RankEntry& r : it->second) t.push_back(r.value);
            if (t.empty()) t = gt.time_trial_defaults(code);
            static const char* const kSlot[3] = {"first_time", "second_time", "third_time"};
            for (int k = 0; k < 3; ++k)
                out.push_back({kSlot[k], k < int(t.size()) ? format_time_ms(u32(std::llround(t[size_t(k)] * 1000)))
                                                            : std::string("--:--:--")});
            if (const data::Car* car = gt.db().car(arcade_entry_car(entry))) {
                data::CarDatabase::Summary sum = gt.db().summarize(*car);
                float ps = sum.power_ps;
                if (auto spec = sim::build_car_spec(gt.db(), arcade_entry_car(entry)); spec && ps <= 0)
                    ps = spec_power_ps(*spec);
                char b[32];
                out.push_back({"power_value", gt.power_text(ps, " ")});
                std::snprintf(b, sizeof b, "%d KG", int(sum.weight_kg));
                out.push_back({"weight_value", b});
            }
        } else if (page == "ar/select_car" || page == "ar/select_car_2p") {
            // The picture names the car ("nsx_s_zero_97"); its arcade car gives the base car.
            const data::Car* car = gt.db().car(arcade_entry_car(entry));
            if (!car) return out;
            data::CarDatabase::Summary sum = gt.db().summarize(*car);
            // A garage car shows its own computed power (parts and condition); cars without a
            // catalogue power (rally prototypes): the built spec's peak (M).
            std::string power;
            if (const GarageCar* gc = arcade_garage_car(entry)) {
                PowerPeak p = gt.power_peak(*gc);
                power = gt.power_text(p.ps) + "/" + std::to_string(p.rpm) + "rpm";
            }
            if (sum.power_ps == 0)
                if (auto spec = sim::build_car_spec(gt.db(), car->label)) {
                    PowerPeak p = spec_power_peak(*spec);
                    sum.power_ps = u16(p.ps);
                    sum.power_rpm = std::to_string(p.rpm) + "rpm";
                    sum.torque_kgfm = spec->peak_torque() / 9.80665f;
                }
            if (power.empty()) power = gt.power_text(float(sum.power_ps)) + "/" + sum.power_rpm;
            std::string drive = sum.drive;
            for (char& ch : drive) ch = char(std::tolower(static_cast<unsigned char>(ch)));
            out.push_back({drive == "4wd" ? "four_wd" : drive, ""});
            // Power and torque with their rpm: "276HP/7300rpm", "224.21ft.lb/5300rpm" (US).
            char b[64];
            if (gt.data().imperial) {
                out.push_back({"ps", power});
                std::snprintf(b, sizeof b, "%.2fft.lb/%s", double(sum.torque_kgfm) * 7.2327, sum.torque_rpm.c_str());
                out.push_back({"kgm", b});
            } else {
                out.push_back({"ps", power});
                std::snprintf(b, sizeof b, "%.1fkgm/%s", double(sum.torque_kgfm), sum.torque_rpm.c_str());
                out.push_back({"kgm", b});
            }
        }
        return out;
    };
    // Arcade car pickers: CircleGauge values. Handling and acceleration come from spec
    // bytes the builder never sets (0 -> 7 x 0.1 = 0.7 for every car, as in the
    // original); speed is the showroom top speed in km/h (truncated) / 350, at most 1
    // (spec menus §9).
    h.arcade_gauges = [](const std::string& entry) {
        std::array<float, 3> v{0.7f, 0.7f, 0.7f};
        GtModeSession* s = S();
        if (!s) return v;
        auto spec = sim::build_car_spec(s->gt().db(), arcade_entry_car(entry));
        if (!spec) return v;
        int kmh = int(sim::showroom_top_speed(*spec) * 3.6f);
        v[1] = std::min(1.0f, float(kmh > 0 ? kmh : 7) / 350.0f);
        return v;
    };
    // Arcade car pickers: the paints of the car's model (carcolor.db / .sdb).
    h.arcade_colors = [](const std::string& entry) {
        std::vector<std::pair<std::string, u32>> out;
        GtModeSession* s = S();
        if (!s) return out;
        const GtData& d = s->gt().data();
        std::string car = arcade_entry_car(entry);
        if (const GarageCar* gc = arcade_garage_car(entry)) {
            // A garage car keeps its own colour (one entry, no arrows).
            u32 id = d.color_id(car, gc->color);
            out.push_back({d.color_name(id), d.color_rgb(id)});
            return out;
        }
        for (int i = 0; i < d.color_count(car); ++i) {
            u32 id = d.color_id(car, i);
            out.push_back({d.color_name(id), d.color_rgb(id)});
        }
        return out;
    };
    // Arcade car pickers: the spec ticker (always millimetres).
    // Arcade garage classes: the car page shows the garage car's picture and strip icon by
    // its label (the arcade layout holds art for every car label).
    h.arcade_image = [](const std::string& entry) {
        const GarageCar* gc = arcade_garage_car(entry);
        return gc ? gc->car : std::string();
    };
    h.arcade_specs = [](const std::string& entry) {
        std::vector<std::pair<std::string, std::string>> out;
        GtModeSession* s = S();
        if (!s) return out;
        const data::Car* car = s->gt().db().car(arcade_entry_car(entry));
        if (!car) return out;
        data::CarDatabase::Summary sum = s->gt().db().summarize(*car);
        auto mm = [](u16 v) { return std::to_string(v) + "mm"; };
        out = {{"Length", mm(sum.length_mm)}, {"Height", mm(sum.height_mm)}, {"Width", mm(sum.width_mm)},
               {"WheelBase", mm(sum.wheelbase_mm)}};
        return out;
    };
    // Arcade races with garage cars: the car as it is in the garage (parts, settings,
    // colour); Single Race picks the race code from its weight / power ratio
    // (dirt course: A?AD0001; road: <2.1 S, <3.0 S+1, <3.5 A, <5.0 A+1, <5.8 B, <6.3 B+1,
    // <9.0 C, else C+1 in the class letter and the last character).
    h.arcade_prepare = [](GameRequest& r) {
        GtModeSession* s = S();
        if (!s) return;
        const GtMode& gt = s->gt();
        const GarageCar* g1 = arcade_garage_car(r.car);
        const GarageCar* g2 = arcade_garage_car(r.car2);
        if (g1 && r.mode == "single_race" && r.race_code.size() == 8) {
            if (gt.arcade_course_dirt(r.course)) {
                r.race_code[2] = 'A', r.race_code[3] = 'D', r.race_code[7] = '1';
            } else {
                float ps = std::max(1.0f, gt.power_ps(*g1));
                float kg = 1000;
                if (auto sp = gt.car_spec(*g1)) kg = std::max(1.0f, sp->mass - 120.0f);
                float ratio = kg / ps; // M: weight / power
                static const struct {
                    float below;
                    char cls, add;
                } kTable[] = {{2.1f, 'S', 0}, {3.0f, 'S', 1}, {3.5f, 'A', 0}, {5.0f, 'A', 1},
                              {5.8f, 'B', 0}, {6.3f, 'B', 1}, {9.0f, 'C', 0}, {1e9f, 'C', 1}};
                for (const auto& t : kTable)
                    if (ratio < t.below) {
                        r.race_code[2] = t.cls;
                        r.race_code[3] = 'T';
                        r.race_code[7] = char('1' + t.add);
                        break;
                    }
            }
        }
        if (g1) fill_car(r, gt, g1);
        // Mod cars: their class's race variant ([arcade] race_variant), as for garage cars.
        if (const data::ModCar* mc = gt.db().mod_car(r.car); mc && !g1 && r.mode == "single_race" &&
                                                                 r.race_code.size() == 8 && r.race_code[3] == 'T')
            r.race_code[7] = char('1' + mc->arcade_variant);
        if (g2) { // player 2: the garage car as tuned, like player 1's
            GameRequest p2;
            fill_car(p2, gt, g2);
            r.car2 = p2.car;
            r.car2_color = p2.car_color;
            r.car2_spec = p2.car_spec;
            r.car2_model = p2.car_model;
            r.car2_wheel = p2.car_wheel;
            r.racing_gearbox2 = p2.racing_gearbox;
        }
    };
    // The save page (menu/flow/save_page.h): the save library's files (title Load / Save /
    // File Operation, home Save, Trade, Load Garage) or the current game's setting files.
    h.save_list = [](menu::flow::SavePage::Mode m) { return save_page_files(m); };
    h.save_run = [](menu::flow::SavePage::Mode m, menu::flow::SavePage::Op op, const menu::flow::SavePage::File& f,
                    const std::string& arg) {
        const int r = save_page_run(m, op, f, arg);
        // A snapshot (begin_snapshot_save) is good for one save.
        if (r == 0 && (op == menu::flow::SavePage::OpSave || op == menu::flow::SavePage::OpOverwrite)) g_snapshot.reset();
        return r;
    };
    h.save_closed = []() { g_snapshot.reset(); };
    // Arcade class page: road classes on a dirt course, the dirt class on a road
    // course and a garage class without cars cannot be chosen.
    h.arcade_disabled = [](const std::string& page, const std::string& course, const std::string& entry) {
        GtModeSession* s = S();
        if (!s) return std::string();
        const GtMode& gt = s->gt();
        // A garage car on a dirt course needs dirt tires (owned-part bit 0x42).
        if (const GarageCar* gc = arcade_garage_car(entry))
            return gt.arcade_course_dirt(course) && !gc->owned.test(66) ? std::string("disable_dirt_doc") : std::string();
        if (page != "ar/select_class") return std::string();
        if (entry == "home_garage") return gt.state().garage.empty() ? std::string("disable_no_car_doc") : std::string();
        if (entry == "guest_garage") // until a guest garage with cars is loaded (Load Garage / Trade)
            return s->guest && !s->guest->state().garage.empty() ? std::string() : std::string("disable_no_car_doc");
        bool dirt = gt.arcade_course_dirt(course);
        if (entry == "class_dirt") return dirt ? std::string() : std::string("disable_dirt_doc");
        if (starts(entry, "class_")) return dirt ? std::string("disable_road_doc") : std::string();
        return std::string();
    };
    // Arcade pickers: only the open courses (gcf list order) and the class's open cars.
    h.arcade_entries = [](const std::string& page, const std::string& game, const std::string& car_class,
                          std::vector<std::string>& entries) {
        GtModeSession* s = S();
        if (!s) return;
        const GtMode& gt = s->gt();
        auto keep_in_order = [&](const std::vector<std::string>& wanted) {
            std::vector<std::string> out;
            for (const std::string& w : wanted)
                if (std::find(entries.begin(), entries.end(), w) != entries.end() &&
                    std::find(out.begin(), out.end(), w) == out.end())
                    out.push_back(w);
            if (!out.empty()) entries = std::move(out);
        };
        if (page == "ar/select_course") {
            const char* list = game == "time_attack" ? "TIME_ATTACK" : game == "two_player_battle" ? "TWO_PLAYER"
                                                                                                   : "SINGLE_RACE";
            keep_in_order(gt.arcade_courses(list));
        } else if (page == "ar/select_class") {
            // The game's class order; Single Race offers no guest garage.
            std::vector<std::string> order = {"class_c", "class_b", "class_a", "class_s", "class_dirt", "home_garage"};
            if (game != "single_race") order.push_back("guest_garage");
            entries.erase(std::remove_if(entries.begin(), entries.end(),
                                         [&](const std::string& e) {
                                             return std::find(order.begin(), order.end(), e) == order.end();
                                         }),
                          entries.end());
            keep_in_order(order);
        } else if ((page == "ar/select_car" || page == "ar/select_car_2p") &&
                   (car_class == "home_garage" || car_class == "guest_garage")) {
            // Garage classes: the garage's cars in list order ("hg<i>" / "gg<i>").
            const GtMode* g = car_class == "home_garage" ? &gt : s->guest.get();
            entries.clear();
            if (g)
                for (size_t i = 0; i < g->state().garage.size(); ++i)
                    entries.push_back((car_class == "home_garage" ? "hg" : "gg") + std::to_string(i));
        } else if (page == "ar/select_car" && starts(car_class, "class_")) {
            std::vector<std::string> imgs;
            for (const GtData::ArcadeCar* c : gt.arcade_cars(car_class)) imgs.push_back(c->image());
            keep_in_order(imgs);
            // Mod cars have no picture on the page (the showroom draws them): listed by label.
            for (const GtData::ArcadeCar* c : gt.arcade_cars(car_class))
                if (gt.db().mod_car(c->label)) entries.push_back(c->label);
        }
        // (ar/select_car_2p: each player's class row asks for that class's cars as
        // "ar/select_car", GameFlow::two_player_step.)
    };
    h.presentation = []() -> std::optional<menu::flow::GtModeHooks::Presentation> {
        g_show.reset();
        if (g_shows.empty()) return std::nullopt;
        g_show = g_shows.front();
        g_shows.erase(g_shows.begin());
        g_show_frames = 0;
        ++g_show_serial;
        // Screenshots: GT3R_GTMODE_PRIZE_FRAME=<n> starts presentations n frames in.
        if (const char* env = std::getenv("GT3R_GTMODE_PRIZE_FRAME")) {
            g_show_frames = std::max(0, std::atoi(env));
            if (S() && (g_show->page == "gtmode_mini" || g_show->page == "consolation_prize" ||
                         g_show->page == "trophy_get")) {
                int n = g_show_frames;
                g_show_frames = 0;
                while (g_show_frames < n) {
                    ++g_show_frames;
                    run_counter(*g_show, S()->gt().state(), false, nullptr);
                }
            }
        }
        if (S()) S()->name_started = false;
        return menu::flow::GtModeHooks::Presentation{g_show->set, g_show->page, g_show->layers};
    };
    h.resume_request = []() -> std::optional<GameRequest> {
        GtModeSession* s = S();
        if (!s || !s->gt().state().championship.active() || !s->gt().state().ride_car()) return std::nullopt;
        GameRequest r;
        r.mode = "go_race";
        r.page = "map";
        prepare_request(r);
        return r;
    };
    menu::flow::set_gt_mode_hooks(std::move(h));
}

} // namespace gt3r::game
