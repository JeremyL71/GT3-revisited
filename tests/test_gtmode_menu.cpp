// GT Mode through the interactive menus (menu::flow::GameFlow + the GT-mode
// hooks of game/gt_mode_menu.cpp), on the real menus. Never writes a save.

#include "test_framework.h"

#include "game/gt_mode_menu.h"
#include "game/gt_save.h"
#include "menu/flow/game_flow.h"
#include "menu/flow/gt_mode_hooks.h"
#include "vfs/vfs.h"

#include <filesystem>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <tuple>

using namespace gt3r;
using namespace gt3r::menu::flow;

namespace {

void settle(GameFlow& f) {
    for (int i = 0; i < 300; ++i) {
        if (!f.switching() && f.controller() && !f.controller()->busy()) break;
        f.tick(0);
    }
    f.tick(0);
}

void press(GameFlow& f, MenuKey k) {
    f.tick(key_bit(k));
    f.tick(0);
    settle(f);
}

// The race-prize presentations a result shows first (arcade / GT-mode cups, the
// consolation prize): checked and confirmed once OK is accepted. Returns the pages seen.
std::vector<std::string> skip_race_prizes(GameFlow& f) {
    std::vector<std::string> seen;
    for (int k = 0; k < 4 && f.controller(); ++k) {
        std::string p = f.controller()->page_name();
        if (p != "arcade_mini" && p != "gtmode_mini" && p != "consolation_prize") break;
        seen.push_back(p);
        for (int i = 0; i < 300; ++i) f.tick(0);
        press(f, MenuKey::Confirm);
        settle(f);
    }
    return seen;
}

// Focus an item by name and confirm it.
void choose(GameFlow& f, const char* item) {
    CHECK(f.controller()->set_focus(item));
    press(f, MenuKey::Confirm);
}

std::string widget_text(const GameFlow& f, const char* cls) {
    for (const menu::ElementDraw& d : f.draw_list())
        if (d.element && d.element->widget_class && *d.element->widget_class == cls && d.text) return *d.text;
    return "<none>";
}

// setenv is POSIX-only; the Windows CRT has _putenv_s (same effect for std::getenv).
void set_env(const char* name, const char* value) {
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

void unset_env(const char* name) {
#ifdef _WIN32
    _putenv_s(name, ""); // an empty value removes it from the CRT environment
#else
    unsetenv(name);
#endif
}

} // namespace

TEST(gtmode_menu_disc_buy_and_race) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    CHECK(s->gt().state().credits == 18000); // no save: a new game

    // Title -> GT mode map: the status bar shows the credits.
    choose(f, "gt_mode");
    settle(f);
    CHECK(f.controller()->page_name() == "map");
    CHECK(widget_text(f, "user_money") == " 18,000"); // a space after the Cr. icon

    // No car yet: the tune shop refuses with the "ride car" popup.
    choose(f, "ts_ts");
    CHECK(f.controller()->depth() == 1 && f.controller()->page_name() == "00cmn/ride_car");
    press(f, MenuKey::Cancel);

    // Dealer: Japan -> first dealer -> a car; too expensive -> no_money popup.
    choose(f, "cd_cd");
    CHECK(f.controller()->page_name() == "cd/cd");
    choose(f, "jp");
    choose(f, f.controller()->items()[0].flow->name.c_str()); // first maker
    std::string maker = f.controller()->page_name();
    choose(f, f.controller()->items()[0].flow->name.c_str()); // first car (set_car_code)
    const std::string car = s->focus_car;
    CHECK(!car.empty());
    CHECK(!widget_text(f, "car_buy_price").empty()); // the number next to a Cr. icon
    s->gt().state().credits = 0;
    choose(f, "buy");
    CHECK(f.controller()->depth() == 1 && f.controller()->page_name() == "00cmn/no_money");
    press(f, MenuKey::Cancel);
    // With money: buy it; the first car becomes the ride car.
    s->gt().state().credits = 1000000;
    CHECK(!(s->gt().car_flags(car) & 1)); // the first Japanese dealer car is for sale
    choose(f, "buy");
    CHECK(f.controller()->page_name() == "00cmn/cd_buy_sure");
    choose(f, "yes1");
    CHECK(s->gt().state().garage.size() == 1 && s->gt().state().ride == 0);
    CHECK(s->gt().state().credits == 1000000 - s->gt().car_price(car));
    CHECK(f.controller()->page_name() == "00cmn/cd_bought");
    choose(f, "yes1");
    settle(f);
    CHECK(f.controller()->page_name() == "cd/cd");

    // Race: beginner -> Sunday Cup -> race 1 -> go race.
    press(f, MenuKey::Cancel);
    settle(f);
    CHECK(f.controller()->page_name() == "map");
    u32 day = s->gt().state().day;
    choose(f, "r_r");
    choose(f, "beginner");
    choose(f, "Gsunday_e_0001");
    CHECK(f.controller()->page_name() == "r/beginner/gsunday_e");
    choose(f, "Gsunday_e_0001");
    CHECK(f.controller()->page_name() == "r/beginner/gsunday_e_entry");
    choose(f, "menu1");
    for (int i = 0; i < 80 && !f.waiting_for_game(); ++i) f.tick(0);
    std::optional<GameRequest> r = f.take_request();
    CHECK(r.has_value());
    if (!r) return;
    CHECK(r->mode == "go_race" && r->race_code == "Gsunday_e_0001" && r->event == "Gsunday_e_0001");
    CHECK(r->car_spec != nullptr && r->car == car);
    CHECK(r->opponents.size() == 5);
    CHECK(s->gt().state().day == day + 1);
    // Win it.
    s64 credits = s->gt().state().credits;
    GameResult res;
    res.mode = r->mode;
    res.race_code = r->race_code;
    res.event = r->event;
    res.finished = true;
    res.position = 1;
    res.cars = 6;
    res.distance_km = 4.5;
    post_game_result(res);
    f.game_finished();
    settle(f);
    // The original shows the event's cup (trophy class 5: race/g_easy_gold.prm) with the
    // prize money counting into the credits before the menu comes back.
    CHECK(f.controller() && f.controller()->page_name() == "gtmode_mini");
    {
        std::string model = game::presentation_model("gtmode_mini");
        CHECK(model == "race/g_easy_gold.prm");
    }
    CHECK(skip_race_prizes(f).size() == 1);
    CHECK(s->gt().state().credits == credits + 1000);
    CHECK(s->gt().state().best_position("Gsunday_e_0001") == 0);
    CHECK(s->gt().state().garage[0].cond.odometer_m == 4500.0);
    CHECK(f.controller() && f.controller()->page_name() == "r/beginner/gsunday_e");

    // Winning the last missing Sunday Cup race gives the series prize car: the
    // car_get presentation runs before the menu comes back.
    s->gt().state().race_best["Gsunday_e_0002"] = 0;
    s->gt().state().race_best["Gsunday_e_0003"] = 0;
    choose(f, "Gsunday_e_0001");
    choose(f, "menu1");
    for (int i = 0; i < 80 && !f.waiting_for_game(); ++i) f.tick(0);
    r = f.take_request();
    CHECK(r.has_value());
    size_t cars = s->gt().state().garage.size();
    post_game_result(res);
    f.game_finished();
    settle(f);
    CHECK(skip_race_prizes(f).size() == 1);
    CHECK(s->gt().state().garage.size() == cars + 1);
    CHECK(f.gt_active() && f.control() == "car_get" && f.controller()->page_name() == "car_get");
    for (int i = 0; i < 200; ++i) f.tick(0); // the roulette
    press(f, MenuKey::Confirm);
    settle(f);
    CHECK(f.control() == "gt_mode" && f.controller()->page_name() == "r/beginner/gsunday_e");
    (void)maker;
}

TEST(gtmode_menu_disc_championship_resume) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    // A save with a championship suspended after round 2.
    s->gt().state().credits = 1000000;
    CHECK(s->gt().buy_car("altezza_rs200_98") == 0);
    CHECK(s->gt().start_championship("Galtezza_e_0501"));
    s->gt().record_round(0);
    s->gt().record_round(1);
    // Entering GT mode resumes it with round 3.
    choose(f, "gt_mode");
    for (int i = 0; i < 80 && !f.waiting_for_game(); ++i) f.tick(0);
    std::optional<GameRequest> r = f.take_request();
    CHECK(r.has_value());
    if (!r) return;
    CHECK(r->mode == "go_race" && r->race_code == "Galtezza_e_0503");
    CHECK(r->champ_round == 3 && r->champ_rounds == 5 && !r->champ_last);
    CHECK(r->opponents == s->gt().state().championship.entrants);
}

TEST(gtmode_menu_disc_touch_taps) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    // Title (a scrolling selector): tapping the focused item in the window confirms it.
    MenuController* c = f.controller();
    CHECK(c->focus() == 0);
    CHECK(c->set_focus("gt_mode"));
    settle(f);
    int hit = -1;
    for (float y = 0; y < 448 && hit != c->focus(); y += 2)
        for (float x = 0; x < 640 && hit != c->focus(); x += 4)
            if (c->hit_test(x, y) == c->focus()) {
                hit = c->focus();
                CHECK(f.tap(x + 1, y + 1));
            }
    CHECK(hit == c->focus());
    settle(f);
    CHECK(f.controller()->page_name() == "map");
    // Map: a tap on another item focuses it, a second tap confirms it.
    c = f.controller();
    int dealer = -1;
    for (int i = 0; i < int(c->items().size()); ++i)
        if (c->items()[size_t(i)].flow->name == "cd_cd") dealer = i;
    CHECK(dealer >= 0);
    const menu::Element* e = c->items()[size_t(dealer)].element;
    float x = e->x + e->w / 2, y = e->y + e->h / 2;
    CHECK(c->hit_test(x, y) == dealer);
    CHECK(c->hit_test(-10, -10) == -1);
    CHECK(f.tap(x, y));
    settle(f);
    CHECK(f.controller()->focus() == dealer && f.controller()->page_name() == "map");
    CHECK(f.tap(x, y));
    settle(f);
    CHECK(f.controller()->page_name() == "cd/cd");
}

TEST(gtmode_menu_disc_settings_sliders) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow title(vfs_reader(vfs), "US");
    title.start(); // creates the GT-mode session
    settle(title);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    game::GtMode& gt = s->gt();
    gt.state().credits = 5000000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    for (int g : {53, 22, 34, 3})
        if (gt.buy_part(g)) gt.fit_part(g);
    CHECK(game::begin_setting(""));
    GameFlow f(vfs_reader(vfs), "US");
    f.start("setting", "root");
    settle(f);
    CHECK(f.controller()->page_name() == "root");
    choose(f, "item2");
    CHECK(f.controller()->page_name() == "parts_setting");
    choose(f, "item1");
    CHECK(f.controller()->page_name() == "set/adjast_sus");
    // Shock absorbers (single damper level) are locked with the racing kit.
    CHECK(!f.controller()->item_enabled(2));
    choose(f, "status_box_sus1");
    CHECK(f.controller()->page_name() == "set/adjast_sus_spring");
    int before = gt.state().ride_car()->settings.spring[0];
    CHECK(f.controller()->set_focus("slider_rate_f"));
    settle(f);
    f.take_sounds();
    press(f, MenuKey::Right);
    std::vector<std::string> se = f.take_sounds();
    CHECK(std::find(se.begin(), se.end(), "param") != se.end());
    CHECK(s->setting_work->settings.spring[0] == before + 1);
    CHECK(gt.state().ride_car()->settings.spring[0] == before); // not committed yet
    choose(f, "ok"); // commit_slider
    CHECK(gt.state().ride_car()->settings.spring[0] == before + 1);
    auto spec = gt.car_spec(*gt.state().ride_car());
    CHECK(spec.has_value());
}

TEST(gtmode_menu_disc_attract_mode) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    CHECK(f.controller()->page_name() == "root");
    // 20 s idle on the title (gcf section 0) starts a demo race.
    std::optional<GameRequest> r;
    for (int i = 0; i < 20 * 60 + 120 && !r; ++i) {
        f.tick(0);
        r = f.take_request();
    }
    CHECK(r.has_value() && f.attract());
    if (!r) return;
    CHECK(r->mode == "attract_demo" && r->event.rfind("demo_", 0) == 0 && !r->course.empty());
    // Five demos that end on their own, then the opening movie, then the title.
    int demos = 1;
    for (int k = 0; k < 10; ++k) {
        GameResult res;
        res.mode = r->mode;
        res.finished = true;
        post_game_result(res);
        f.game_finished();
        r.reset();
        for (int i = 0; i < 80 && !r; ++i) {
            f.tick(0);
            r = f.take_request();
        }
        if (!r) break;
        if (r->kind == GameRequest::Kind::PlayMovie) break;
        ++demos;
    }
    CHECK(demos == 5);
    CHECK(r && r->kind == GameRequest::Kind::PlayMovie && r->course == "stream/opening.pss");
    f.game_finished();
    settle(f);
    CHECK(!f.attract() && f.controller() && f.controller()->page_name() == "root");
    // A button during a demo ends attract mode at once.
    for (int i = 0; i < 20 * 60 + 120 && !r; ++i) f.tick(0);
    r.reset();
    for (int i = 0; i < 20 * 60 + 200 && !r; ++i) {
        f.tick(0);
        r = f.take_request();
    }
    CHECK(r.has_value());
    GameResult quit;
    quit.finished = false;
    post_game_result(quit);
    f.game_finished();
    settle(f);
    CHECK(!f.attract() && f.controller()->page_name() == "root");
}

// A licence run with a medal: the race enters the record itself and runs the name entry
// as a stand-alone presentation (the original asks right after the run); the result
// posted later (records_done) does not add it again.
// The race prize page shown from the race (before the replay): the prize counts down while the
// balance (not yet credited) counts up; OK only once the count is done.
TEST(gtmode_menu_disc_race_prize_page) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow menus(vfs_reader(vfs), "US");
    menus.start();
    settle(menus);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    s64 credits = s->gt().state().credits;
    CHECK(game::queue_race_prize("Gsunday_e_0001", 0));
    GameFlow f(vfs_reader(vfs), "US");
    CHECK(f.start_presentations());
    f.tick(0);
    CHECK(f.controller() && f.controller()->page_name() == "gtmode_mini");
    CHECK(game::presentation_model("gtmode_mini") == "race/g_easy_gold.prm");
    CHECK(widget_text(f, "rank_disp") == "1st");
    CHECK(widget_text(f, "count") == "Cr. " + game::format_credits(credits));
    CHECK(widget_text(f, "get_money_disp") == "Prize Cr. 1,000");
    for (int i = 0; i < 300; ++i) f.tick(0);
    CHECK(widget_text(f, "count") == "Cr. " + game::format_credits(credits + 1000));
    // The prize line stays at 0 for 61 frames after the count, then fades out (60 frames).
    CHECK(widget_text(f, "get_money_disp") == "Prize Cr. 0");
    press(f, MenuKey::Confirm);
    for (int i = 0; i < 60 && !f.exited(); ++i) f.tick(0);
    CHECK(f.exited());
}

TEST(gtmode_menu_disc_license_run_name_entry) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow menus(vfs_reader(vfs), "US");
    menus.start();
    settle(menus);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    s->gt().state().rankings.erase("lic:L0B0001");
    CHECK(game::add_license_record("L0B0001", 35.2) == 0);
    GameFlow f(vfs_reader(vfs), "US");
    CHECK(f.start_presentations());
    settle(f);
    CHECK(f.controller() && f.controller()->page_name() == "name_entry");
    CHECK(f.controller()->set_focus("free_command"));
    press(f, MenuKey::Confirm); // OK
    for (int i = 0; i < 60 && !f.exited(); ++i) f.tick(0);
    CHECK(f.exited());
    CHECK(s->gt().state().rankings["lic:L0B0001"].size() == 1);
    GameFlow empty(vfs_reader(vfs), "US");
    CHECK(!empty.start_presentations() && empty.exited());
}

TEST(gtmode_menu_disc_record_name_entry) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    s->gt().state().credits = 1000000;
    CHECK(s->gt().buy_car("r34_gtr_v_99") == 0);
    choose(f, "gt_mode");
    settle(f);
    choose(f, "mt_mt");
    CHECK(f.controller()->page_name() == "mt/mt");
    choose(f, "M400");
    for (int i = 0; i < 80 && !f.waiting_for_game(); ++i) f.tick(0);
    std::optional<GameRequest> r = f.take_request();
    CHECK(r.has_value() && r->race_code == "M400");
    GameResult res;
    res.mode = "go_machine_test";
    res.race_code = res.event = "M400";
    res.finished = true;
    res.total_time = 12.345;
    post_game_result(res);
    f.game_finished();
    settle(f);
    // A new 0-400 m record: the name entry page.
    CHECK(f.control() == "name_entry" && f.controller()->page_name() == "name_entry");
    MenuController* c = f.controller();
    // One key cursor over the whole grid (FreeButton rows), the column kept across rows.
    {
        auto highlighted = [&] {
            int n = 0;
            const MenuSet::PageRef ref = c->page();
            const menu::Page& pg = ref.layout();
            for (const menu::ElementDraw& d : f.draw_list())
                for (const menu::Element& it : pg.items)
                    if (it.name.size() <= 5 && !it.alt.empty() && d.element == &it.alt[0]) ++n;
            return n;
        };
        CHECK(c->page_name() == "name_entry");
        CHECK(highlighted() == 1);
        for (int i = 0; i < 3; ++i) press(f, MenuKey::Right);
        press(f, MenuKey::Down);
        CHECK(c->focused() && c->focused()->flow->name == "free_cursor1");
        CHECK(c->choice("free_cursor1") == 3); // A..P col 3 -> T
        CHECK(highlighted() == 1);
        press(f, MenuKey::Up);
        press(f, MenuKey::Up); // wraps to the OK row
        CHECK(c->focused() && c->focused()->flow->name == "free_command");
        press(f, MenuKey::Down); // back to row 0: it keeps its key
        CHECK(c->choice("free_cursor") == 3);
        CHECK(highlighted() == 1);
        c->set_focus("free_cursor");
        c->set_choice("free_cursor", 0);
    }
    auto type = [&](const char* row, int child) {
        CHECK(c->set_focus(row));
        c->set_choice(row, child);
        press(f, MenuKey::Confirm);
    };
    type("free_cursor", 6);  // G
    type("free_cursor1", 3); // T (row Q..Z)
    type("free_cursor", 2);  // C
    f.tick(key_bit(MenuKey::Cancel)); // delete the C
    f.tick(0);
    type("free_cursor1", 1); // R
    const auto& rank = s->gt().state().rankings["mt:M400"];
    CHECK(rank.size() == 1 && rank[0].name == "GTR" && rank[0].value == 12.345);
    CHECK(s->gt().state().player_name == "GTR");
    CHECK(c->set_focus("free_command"));
    press(f, MenuKey::Confirm); // OK
    settle(f);
    CHECK(f.control() == "gt_mode" && f.controller()->page_name() == "mt/mt");
}

// Arcade progression through the arcade menus: the pickers offer only the open
// courses / cars, Professional is Hard with L1 + R1, a Single Race win is
// recorded, and Clear Status shows the marks and refuses the endings.
TEST(gtmode_menu_disc_arcade_progression) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    press(f, MenuKey::Confirm); // title: Arcade
    settle(f);
    CHECK(f.control() == "arcade" && f.controller()->page_name() == "ar/top");
    choose(f, "item1"); // Single Race
    CHECK(f.controller()->page_name() == "ar/level");
    // Professional: confirm Hard while L1 and R1 are held.
    const MenuKeys lr = key_bit(MenuKey::L1) | key_bit(MenuKey::R1);
    CHECK(f.controller()->set_focus("item8"));
    auto drawn_now = [&](const char* n) {
        for (const menu::ElementDraw& d : f.draw_list())
            if (d.element && d.element->name == n) return true;
        return false;
    };
    for (int i = 0; i < 30; ++i) f.tick(0);
    CHECK(!drawn_now("pro_a") && drawn_now("text_hard"));
    f.tick(lr);
    CHECK(drawn_now("pro_a") && drawn_now("text_pro") && !drawn_now("text_hard")); // Hard shows the "pro" art
    f.tick(lr | key_bit(MenuKey::Confirm));
    for (int i = 0; i < 300 && (f.switching() || f.controller()->busy() || f.controller()->page_name() == "ar/level");
         ++i)
        f.tick(lr);
    settle(f);
    CHECK(f.controller()->page_name() == "ar/select_course");
    // New game: the six courses of group 0, in the game's order.
    const std::vector<std::string> group0 = {"super", "midfield", "smtnorth", "smtsouth", "trialmt", "r_midfield"};
    CHECK(f.carousel() && f.carousel()->entries == group0);
    press(f, MenuKey::Confirm); // Super Speedway
    CHECK(f.controller()->page_name() == "ar/select_class");
    CHECK(f.carousel() && f.carousel()->entries.size() > 1 && f.carousel()->entries[1] == "class_b");
    press(f, MenuKey::Right);
    press(f, MenuKey::Confirm); // class B
    CHECK(f.controller()->page_name() == "ar/select_car");
    const GameFlow::Carousel* cars = f.carousel();
    CHECK(cars && cars->entries.size() == 10);
    if (cars) {
        auto has = [&](const char* n) {
            return std::find(cars->entries.begin(), cars->entries.end(), n) != cars->entries.end();
        };
        CHECK(has("s2000_typev_00") && has("opel_speedster_00") && !has("tt_00") && !has("vauxhall_vx220_00"));
    }
    // A paint, then AUTOMATIC / MANUAL and RACING / DRIFT, then Back twice: the page keeps
    // taking input (it used to stop: the page's own cancel had ended the controller).
    press(f, MenuKey::Down);
    press(f, MenuKey::Confirm);
    CHECK(f.carousel() && f.carousel()->step == 2);
    press(f, MenuKey::Confirm);
    CHECK(f.carousel() && f.carousel()->step == 3);
    press(f, MenuKey::Cancel);
    CHECK(f.carousel() && f.carousel()->step == 2 && f.controller()->page_name() == "ar/select_car");
    press(f, MenuKey::Cancel);
    CHECK(f.carousel() && f.carousel()->step == 1 && f.controller()->page_name() == "ar/select_car");
    {
        int before = f.carousel()->index;
        press(f, MenuKey::Right);
        CHECK(f.carousel() && f.carousel()->index == before + 1); // input still accepted
        press(f, MenuKey::Left);
        CHECK(f.carousel() && f.carousel()->index == before);
    }
    press(f, MenuKey::Up); // back to the first paint
    // OK: AUTOMATIC / MANUAL (choose MANUAL), then RACING / DRIFT, then the race.
    press(f, MenuKey::Confirm);
    CHECK(f.carousel() && f.carousel()->step == 2 && !f.waiting_for_game());
    press(f, MenuKey::Right);
    press(f, MenuKey::Confirm);
    CHECK(f.carousel() && f.carousel()->step == 3);
    press(f, MenuKey::Confirm);
    for (int i = 0; i < 80 && !f.waiting_for_game(); ++i) f.tick(0);
    std::optional<GameRequest> r = f.take_request();
    CHECK(r && r->transmission == 1 && r->tire_mode == 0);
    CHECK(r && r->mode == "single_race" && r->difficulty == 3 && r->race_code == "APBT0001" && r->course == "super");
    GameResult res;
    res.mode = "single_race";
    res.race_code = "APBT0001";
    res.finished = true;
    res.position = 1;
    post_game_result(res);
    f.game_finished();
    settle(f);
    // A win shows the difficulty's cup (Professional: race/a_special_gold.prm), OK after 3 s.
    CHECK(f.controller()->page_name() == "arcade_mini");
    CHECK(game::presentation_model("arcade_mini") == "race/a_special_gold.prm");
    CHECK(skip_race_prizes(f).size() == 1);
    CHECK(f.controller()->page_name() == "ar/top");
    const game::ArcadeProgress& a = s->gt().state().arcade;
    CHECK(a.best[size_t(3 * game::kArcadeCourses)] == 0 && a.awards == 0x8000 && a.course_level == 0);
    // Clear Status: a Pro win has no mark; an Easy win shows the Easy mark.
    s->gt().arcade_single_race("AECT0001", "super", 0);
    choose(f, "item5");
    CHECK(f.controller()->page_name() == "ar/goodies");
    auto drawn = [&](const char* n) {
        for (const menu::ElementDraw& d : f.draw_list())
            if (d.element && d.element->name == n) return true;
        return false;
    };
    CHECK(drawn("super_mini") && drawn("clear_easy") && !drawn("clear_normal") && !drawn("akasaka_mini"));
    // No ending yet: "Goal" shows the refusal.
    choose(f, "Goal");
    CHECK(f.controller()->page_name() == "ar/cannot_ending");
}

// Arcade class page refusals and the time-trial name entry.
TEST(gtmode_menu_disc_arcade_class_and_time_trial) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    CHECK(s->gt().arcade_course_dirt("tahitidirt") && !s->gt().arcade_course_dirt("super"));
    auto drawn = [&](const char* n) {
        for (const menu::ElementDraw& d : f.draw_list())
            if (d.element && d.element->name == n) return true;
        return false;
    };
    press(f, MenuKey::Confirm); // Arcade
    choose(f, "item1");          // Single Race
    choose(f, "item1");          // Easy
    press(f, MenuKey::Confirm);  // Super Speedway (road)
    CHECK(f.controller()->page_name() == "ar/select_class");
    // The game's class order (Single Race: no guest garage).
    const std::vector<std::string> classes = {"class_c", "class_b", "class_a", "class_s", "class_dirt", "home_garage"};
    CHECK(f.carousel() && f.carousel()->entries == classes);
    CHECK(!drawn("disabled") && drawn("class_c_doc"));
    for (int i = 0; i < 4; ++i) press(f, MenuKey::Right);
    // The dirt class on a road course: refused with its reason.
    CHECK(drawn("disabled") && drawn("disable_dirt_doc") && !drawn("class_dirt_doc"));
    press(f, MenuKey::Confirm);
    CHECK(f.controller()->page_name() == "ar/select_class");
    press(f, MenuKey::Right); // home_garage: the GT garage is empty on a new game
    CHECK(drawn("disable_no_car_doc"));
    press(f, MenuKey::Cancel);
    press(f, MenuKey::Cancel);
    press(f, MenuKey::Cancel);
    CHECK(f.controller()->page_name() == "ar/top");
    // Time trial 1, a time under the record: name entry, then back to the arcade menu.
    choose(f, "item2");
    CHECK(f.controller()->page_name() == "ar/select_compulsory");
    press(f, MenuKey::Confirm);
    for (int i = 0; i < 80 && !f.waiting_for_game(); ++i) f.tick(0);
    std::optional<GameRequest> r = f.take_request();
    CHECK(r && r->mode == "time_trial" && r->course == "tt01");
    GameResult res;
    res.mode = "time_trial";
    res.finished = true;
    res.position = 1;
    res.total_time = 80.0;
    post_game_result(res);
    f.game_finished();
    settle(f);
    CHECK(f.controller()->page_name() == "name_entry");
    MenuController* c = f.controller();
    CHECK(c->set_focus("free_cursor1"));
    c->set_choice("free_cursor1", 9); // "Z" (row Q..Z)
    press(f, MenuKey::Confirm);
    CHECK(c->set_focus("free_command"));
    press(f, MenuKey::Confirm); // OK
    settle(f);
    CHECK(f.control() == "arcade" && f.controller()->page_name() == "ar/top");
    const auto& rank = s->gt().state().rankings["at:AT010001"];
    CHECK(rank.size() == 4 && rank[0].value == 80.0 && rank[0].name == "Z" &&
          rank[1].name == game::GtMode::time_trial_seed_name(0, 0));
    CHECK(s->gt().state().arcade.at_best_rank[0] == 0);
    if (!rank.empty()) CHECK(rank[0].car == "motersport_elise_00");
}

// A championship win shows the trophy presentation (a random champ_NN.prm, the bonus
// counting up, OK after the timeline), then the prize car.
TEST(gtmode_menu_disc_championship_trophy) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    unset_env("GT3R_GTMODE_PRIZE_FRAME");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    s->gt().state().credits = 1000000;
    CHECK(s->gt().buy_car("altezza_rs200_98") == 0);
    CHECK(s->gt().start_championship("Galtezza_e_0501"));
    for (int i = 0; i < 4; ++i) s->gt().record_round(0);
    choose(f, "gt_mode"); // resumes with the last round
    for (int i = 0; i < 80 && !f.waiting_for_game(); ++i) f.tick(0);
    std::optional<GameRequest> r = f.take_request();
    CHECK(r && r->champ_last);
    s64 before = s->gt().state().credits;
    GameResult res;
    res.mode = "go_race";
    res.race_code = r ? r->race_code : "";
    res.event = r ? r->event : "";
    res.finished = true;
    res.position = 1;
    post_game_result(res);
    f.game_finished();
    settle(f);
    skip_race_prizes(f);
    CHECK(f.controller()->page_name() == "trophy_get");
    float fade = 1;
    std::string model = game::presentation_model("trophy_get", nullptr, &fade);
    CHECK(model.empty() && fade == 0); // fades in from 3 s
    for (int i = 0; i < 300; ++i) f.tick(0);
    model = game::presentation_model("trophy_get", nullptr, &fade);
    CHECK(model.size() == 17 && model.rfind("menu/champ_0", 0) == 0 && fade == 1.0f);
    CHECK(s->gt().state().trophy_flags != 0);
    s64 bonus = s->gt().state().credits - before;
    CHECK(bonus > 0);
    // OK is refused until the count-up is over.
    press(f, MenuKey::Confirm);
    CHECK(f.controller()->page_name() == "trophy_get");
    // The prize counter starts 360 frames in: the bonus box shows what is left of the bonus
    // (its two text widgets, drawn from the forced get_mess layer).
    for (int i = 0; i < 70; ++i) f.tick(0);
    // (`bonus` also holds the last round's prize money, shown before.)
    int boxes = 0;
    for (const menu::ElementDraw& d : f.draw_list())
        if (d.text && d.element && d.element->widget_class && *d.element->widget_class == "get_money_disp") {
            ++boxes;
            std::string digits;
            for (char ch : *d.text)
                if (ch >= '0' && ch <= '9') digits += ch;
            CHECK(d.text->rfind("Prize Cr. ", 0) == 0 && !digits.empty());
            s64 left = digits.empty() ? 0 : std::stoll(digits);
            CHECK(left > 0 && left < bonus);
            CHECK(widget_text(f, "count") == "Cr. " + game::format_credits(s->gt().state().credits - left));
        }
    CHECK(boxes == 2);
    for (int i = 0; i < 700; ++i) f.tick(0);
    CHECK(widget_text(f, "count") == "Cr. " + game::format_credits(s->gt().state().credits));
    CHECK(widget_text(f, "get_money_disp") == "Prize Cr. 0");
    press(f, MenuKey::Confirm);
    settle(f);
    CHECK(f.controller()->page_name() != "trophy_get");
}

// The arcade pickers as the original lays them out: every available entry in the icon
// strip, the course list clamped at its ends (arrows hidden), the class list wrapping, and
// the entry's art sliding with the cross.
TEST(gtmode_menu_disc_arcade_picker_strip) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    auto count = [&](const std::string& suffix) {
        int n = 0;
        for (const menu::ElementDraw& d : f.draw_list())
            if (d.element && !d.additive && d.element->name.size() > suffix.size() &&
                d.element->name.compare(d.element->name.size() - suffix.size(), suffix.size(), suffix) == 0)
                ++n;
        return n;
    };
    auto drawn = [&](const char* name, float* dx = nullptr) {
        for (const menu::ElementDraw& d : f.draw_list())
            if (d.element && d.element->name == name) {
                if (dx) *dx = d.dx;
                return true;
            }
        return false;
    };
    press(f, MenuKey::Confirm); // Arcade
    choose(f, "item1");         // Single Race
    choose(f, "item1");         // Easy
    CHECK(f.controller()->page_name() == "ar/select_course");
    for (int i = 0; i < 40; ++i) f.tick(0);
    CHECK(count("_mini") == 6); // the six open courses, all in the strip
    CHECK(!drawn("arrow_l") && drawn("arrow_r"));
    CHECK(drawn("super_logo") && drawn("tarmac") && drawn("area_a") && drawn("length"));
    (void)f.take_sounds();
    press(f, MenuKey::Left); // clamped
    CHECK(f.carousel()->index == 0);
    std::vector<std::string> se = f.take_sounds();
    CHECK(std::find(se.begin(), se.end(), "disable") != se.end());
    f.tick(key_bit(MenuKey::Right));
    f.tick(0);
    // Mid-slide: the new art comes from the right, the old leaves to the left.
    float dx_new = 0, dx_old = 0;
    CHECK(drawn("midfield_logo", &dx_new) && drawn("super_logo", &dx_old));
    CHECK(dx_new > 300 && dx_old < 0);
    for (int i = 0; i < 40; ++i) f.tick(0);
    CHECK(drawn("midfield_logo", &dx_new) && std::fabs(dx_new) < 2 && !drawn("super_logo"));
    for (int i = 0; i < 6; ++i) press(f, MenuKey::Right);
    CHECK(f.carousel()->index == 5 && drawn("arrow_l") && !drawn("arrow_r"));
    // Class list: wraps.
    press(f, MenuKey::Confirm);
    CHECK(f.controller()->page_name() == "ar/select_class");
    int n = int(f.carousel()->entries.size());
    int start = f.carousel()->index;
    press(f, MenuKey::Left);
    CHECK(f.carousel()->index == (start + n - 1) % n);
    CHECK(drawn("arrow_l") && drawn("arrow_r") && count("_mini") == n);
}

// 2 Player Battle: the course on pad 1, then both players choose class, car, AT / MT and
// RACING / DRIFT at once on their own pads; the race starts when both are ready.
TEST(gtmode_menu_disc_arcade_two_player) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    press(f, MenuKey::Confirm); // Arcade
    choose(f, "item4");         // 2 Player Battle
    CHECK(f.controller()->page_name() == "ar/select_course");
    press(f, MenuKey::Confirm); // Super Speedway
    CHECK(f.controller()->page_name() == "ar/select_car_2p");
    for (int i = 0; i < 40; ++i) f.tick(0);
    CHECK(f.two_player(0) && f.two_player(1) && f.two_player(0)->state == 2 && f.two_player(1)->state == 2);
    // Both pads at once: player 1 class A (Right x 2), player 2 class C.
    auto both = [&](MenuKeys k1, MenuKeys k2) {
        f.set_player2_keys(k2);
        f.tick(k1);
        f.set_player2_keys(0);
        f.tick(0);
        f.tick(0);
    };
    const MenuKeys R = key_bit(MenuKey::Right), OK = key_bit(MenuKey::Confirm), D = key_bit(MenuKey::Down);
    both(R, 0);
    both(R, 0);
    CHECK(f.two_player(0)->cls == 2 && f.two_player(1)->cls == 0);
    both(OK, OK);
    CHECK(f.two_player(0)->state == 4 && f.two_player(1)->state == 4);
    CHECK(!f.two_player(0)->cars.empty() && !f.two_player(1)->cars.empty());
    both(R, D); // P1 next car, P2 next paint
    CHECK(f.two_player(0)->car == 1 && f.two_player(1)->paint == 1);
    both(OK, OK); // AT / MT
    both(0, R);   // P2 manual
    both(OK, OK); // RACING / DRIFT
    CHECK(f.two_player(0)->state == 8 && f.two_player(1)->state == 8);
    both(OK, 0);
    CHECK(f.two_player(0)->state == 9 && !f.waiting_for_game());
    both(0, OK);
    for (int i = 0; i < 80 && !f.waiting_for_game(); ++i) f.tick(0);
    std::optional<GameRequest> r = f.take_request();
    CHECK(r && r->mode == "two_player_battle" && r->car_class == "class_a" && r->car2_class == "class_c");
    CHECK(r && !r->car.empty() && !r->car2.empty() && r->transmission == 0 && r->transmission2 == 1 && r->car2_color == 1);
}

// The save page (save_page.h) over a fake library: no slot choice and no "working" /
// "Complete" windows; the operation menu, one confirm for delete / overwrite, the name
// entry for new saves and renames, the file dialog for imports, exports and guest
// garages, rows scrolling past twelve files, the error window.
TEST(save_page_flow) {
    struct Fake {
        std::string id, name;
        bool current = false;
    };
    std::vector<Fake> saves = {{"a", "A", true}, {"b", "B"}};
    std::vector<std::tuple<int, std::string, std::string>> ran; // op, file id, arg
    int fail = 0;
    SavePage::Backend b;
    b.list = [&](SavePage::Mode m) {
        std::vector<SavePage::File> out;
        if (m == SavePage::Mode::LoadGame || m == SavePage::Mode::SaveGame) {
            SavePage::File n;
            n.kind = SavePage::File::Kind::New;
            n.name = "GAME";
            out.push_back(n);
        }
        for (const Fake& f : saves) {
            if (m == SavePage::Mode::LoadGuest && f.current) continue;
            SavePage::File g;
            g.id = f.id;
            g.name = f.name;
            g.current = f.current;
            out.push_back(g);
        }
        return out;
    };
    b.run = [&](SavePage::Mode, SavePage::Op op, const SavePage::File& f, const std::string& arg) {
        ran.emplace_back(int(op), f.id, arg);
        if (fail) return fail;
        if (op == SavePage::OpDelete)
            saves.erase(std::remove_if(saves.begin(), saves.end(), [&](const Fake& x) { return x.id == f.id; }), saves.end());
        if (op == SavePage::OpCopy || op == SavePage::OpImport) saves.insert(saves.begin(), {"c", "C"});
        return 0;
    };
    std::optional<std::string> dialog_answer;
    std::vector<int> dialogs;
    FileDialogs d;
    d.show = [&](FileDialogs::Kind k, const std::string&) { dialogs.push_back(int(k)); };
    d.poll = [&]() { return std::exchange(dialog_answer, std::nullopt); };
    set_file_dialogs(d);
    std::vector<std::string> snd;
    auto press = [&](SavePage& p, MenuKey k) {
        int r = p.tick(key_bit(k), snd);
        return r ? r : p.tick(0, snd);
    };
    // Load: the cursor starts on the save being played; its menu has no Delete.
    SavePage p;
    p.open(SavePage::Mode::LoadGame, b);
    p.tick(0, snd);
    CHECK(p.files().size() == 3 && p.files(1).size() == 1 && p.cursor() == 1);
    press(p, MenuKey::Confirm);
    CHECK(p.phase() == SavePage::Phase::Operation);
    CHECK((p.ops() == std::vector<SavePage::Op>{SavePage::OpLoad, SavePage::OpRename, SavePage::OpCopy, SavePage::OpExport}));
    press(p, MenuKey::Cancel);
    // The other save: Delete asks once (CANCEL first), OK deletes and the list stays.
    press(p, MenuKey::Right);
    press(p, MenuKey::Confirm);
    CHECK(p.ops().size() == 5 && p.ops()[3] == SavePage::OpDelete);
    for (int i = 0; i < 3; ++i) press(p, MenuKey::Down);
    press(p, MenuKey::Confirm);
    CHECK(p.phase() == SavePage::Phase::Confirm);
    press(p, MenuKey::Confirm); // CANCEL
    CHECK(p.phase() == SavePage::Phase::Operation && ran.empty());
    press(p, MenuKey::Confirm);
    press(p, MenuKey::Left);
    CHECK(press(p, MenuKey::Confirm) == 0 && p.phase() == SavePage::Phase::List && saves.size() == 1);
    CHECK(std::get<0>(ran.back()) == SavePage::OpDelete && std::get<1>(ran.back()) == "b");
    // Rename: the name entry, then done (cut to 16 characters, no confirm).
    press(p, MenuKey::Left);
    press(p, MenuKey::Right); // onto "A"
    press(p, MenuKey::Confirm);
    press(p, MenuKey::Down);
    press(p, MenuKey::Confirm);
    CHECK(p.wants_name() && p.name_start() == "A");
    p.name_done(true, "Championship run 2");
    CHECK(p.phase() == SavePage::Phase::List && std::get<2>(ran.back()) == "Championship run");
    // Import: the file dialog (cancel = nothing), then the file.
    press(p, MenuKey::Right);
    press(p, MenuKey::Right);
    press(p, MenuKey::Right);
    CHECK(p.panel() == 1);
    press(p, MenuKey::Confirm);
    CHECK(p.phase() == SavePage::Phase::Dialog && dialogs.back() == int(FileDialogs::Kind::OpenSave));
    dialog_answer = "";
    p.tick(0, snd);
    CHECK(p.phase() == SavePage::Phase::List);
    press(p, MenuKey::Confirm);
    dialog_answer = "/x/BASCUS-97102GAMEDATA.psu";
    p.tick(0, snd);
    CHECK(std::get<0>(ran.back()) == SavePage::OpImport && std::get<2>(ran.back()) == "/x/BASCUS-97102GAMEDATA.psu");
    CHECK(p.panel() == 0 && p.cursor() == 1 && p.focused()->id == "c"); // the imported save
    // New game: the "New Save" entry asks for a name and ends the page.
    press(p, MenuKey::Left);
    press(p, MenuKey::Confirm);
    CHECK(p.wants_name() && p.name_start() == "GAME");
    p.name_done(true, "SECOND");
    CHECK(p.tick(0, snd) == 1 && std::get<0>(ran.back()) == SavePage::OpNewGame && std::get<2>(ran.back()) == "SECOND");

    // Save: "Save" over the current save at once; over another one after "Overwrite OK?".
    saves = {{"a", "A", true}, {"b", "B"}};
    SavePage q;
    q.open(SavePage::Mode::SaveGame, b);
    q.tick(0, snd);
    press(q, MenuKey::Confirm);
    CHECK(q.ops().front() == SavePage::OpOverwrite);
    CHECK(press(q, MenuKey::Confirm) == 1 && std::get<0>(ran.back()) == SavePage::OpOverwrite && std::get<1>(ran.back()) == "a");
    q.open(SavePage::Mode::SaveGame, b);
    q.tick(0, snd);
    press(q, MenuKey::Right);
    press(q, MenuKey::Confirm);
    press(q, MenuKey::Confirm);
    CHECK(q.phase() == SavePage::Phase::Confirm);
    CHECK(press(q, MenuKey::Confirm) == 1 && std::get<1>(ran.back()) == "b");
    // Export: the save dialog with the file's name, then back to the list.
    q.open(SavePage::Mode::Files, b);
    q.tick(0, snd);
    CHECK(q.files().size() == 2 && q.files(1).size() == 1);
    press(q, MenuKey::Confirm);
    for (int i = 0; i < 3; ++i) press(q, MenuKey::Down);
    press(q, MenuKey::Confirm);
    CHECK(q.phase() == SavePage::Phase::Dialog && dialogs.back() == int(FileDialogs::Kind::ExportPs2));
    dialog_answer = "/x/out.ps2";
    q.tick(0, snd);
    CHECK(std::get<0>(ran.back()) == SavePage::OpExport && std::get<2>(ran.back()) == "/x/out.ps2");
    CHECK(q.phase() == SavePage::Phase::List);
    // A failing operation: the error window, OK back to the list.
    fail = SavePage::WriteError;
    press(q, MenuKey::Confirm);
    press(q, MenuKey::Down);
    press(q, MenuKey::Confirm); // Copy
    CHECK(q.phase() == SavePage::Phase::Error);
    CHECK(press(q, MenuKey::Confirm) == 0 && q.phase() == SavePage::Phase::List);
    fail = 0;
    CHECK(press(q, MenuKey::Cancel) == -1);

    // Guest garage: a save loads at once (not the current one), or a file from elsewhere.
    SavePage g;
    g.open(SavePage::Mode::LoadGuest, b, "Trade");
    g.tick(0, snd);
    CHECK(g.files().size() == 1 && g.files()[0].id == "b");
    CHECK(press(g, MenuKey::Confirm) == 1 && std::get<0>(ran.back()) == SavePage::OpLoad && std::get<1>(ran.back()) == "b");
    g.open(SavePage::Mode::LoadGuest, b);
    g.tick(0, snd);
    press(g, MenuKey::Right);
    press(g, MenuKey::Confirm);
    dialog_answer = "/x/card.ps2";
    CHECK(g.tick(0, snd) == 1 && std::get<0>(ran.back()) == SavePage::OpLoad && std::get<2>(ran.back()) == "/x/card.ps2");

    // Many saves: rows scroll (three shown).
    saves.clear();
    for (int i = 0; i < 20; ++i) saves.push_back({std::to_string(i), "S" + std::to_string(i)});
    SavePage m;
    m.open(SavePage::Mode::Files, b);
    m.tick(0, snd);
    for (int i = 0; i < 4; ++i) press(m, MenuKey::Down);
    CHECK(m.cursor() == 16 && m.top_row() == 2);
    press(m, MenuKey::Down);
    CHECK(m.cursor() == 16); // the last row holds 16..19
    for (int i = 0; i < 4; ++i) press(m, MenuKey::Up);
    CHECK(m.cursor() == 0 && m.top_row() == 0);
    // Without file dialogs: no right panel, no Export.
    set_file_dialogs({});
    m.open(SavePage::Mode::Files, b);
    m.tick(0, snd);
    press(m, MenuKey::Confirm);
    CHECK(m.files(1).empty() && m.ops().back() == SavePage::OpDelete);
}

// Settings files on the save page: the car's files (Load), "New Save" first (Save) and the
// other cars' files dimmed; no slots.
TEST(save_page_settings) {
    SavePage::Backend b;
    std::vector<int> ran;
    b.list = [&](SavePage::Mode m) {
        std::vector<SavePage::File> out;
        if (m == SavePage::Mode::SaveSettings) out.push_back({SavePage::File::Kind::New, "", true, false, false, "", {}});
        out.push_back({SavePage::File::Kind::Setting, "1", true, false, false, "grip", {}});
        if (m == SavePage::Mode::SaveSettings) out.push_back({SavePage::File::Kind::Setting, "0", false, false, false, "other", {}});
        return out;
    };
    b.run = [&](SavePage::Mode, SavePage::Op op, const SavePage::File&, const std::string&) {
        ran.push_back(int(op));
        return 0;
    };
    std::vector<std::string> snd;
    auto press = [&](SavePage& p, MenuKey k) {
        int r = p.tick(key_bit(k), snd);
        return r ? r : p.tick(0, snd);
    };
    SavePage p;
    p.open(SavePage::Mode::LoadSettings, b);
    p.tick(0, snd);
    CHECK(p.files().size() == 1 && p.files(1).empty());
    press(p, MenuKey::Confirm);
    CHECK(p.ops().front() == SavePage::OpLoad);
    CHECK(press(p, MenuKey::Confirm) == 1 && ran.back() == SavePage::OpLoad);
    p.open(SavePage::Mode::SaveSettings, b);
    p.tick(0, snd);
    CHECK(p.cursor() == 1); // the first file, not "New Save"
    press(p, MenuKey::Right);
    press(p, MenuKey::Confirm);
    CHECK(p.ops().front() == SavePage::OpRename); // another car's file: no Overwrite
    press(p, MenuKey::Cancel);
    press(p, MenuKey::Left);
    press(p, MenuKey::Left);
    CHECK(press(p, MenuKey::Confirm) == 1 && ran.back() == SavePage::OpSave); // "New Save": at once
}

// Options ([options] in config.ini): values, defaults and the preview widgets.
TEST(options_values_and_previews) {
    IniDocument cfg;
    CHECK(option_value(&cfg, "wide_mode") == "off" && option_value(&cfg, "menu_bgm") == "on");
    CHECK(option_value(&cfg, "split_boost") == "low" && option_laps(&cfg) == 2 && option_handicap(&cfg) == 0);
    // Four-widget choices: the lit art of the current value only.
    CHECK(option_preview_visible(&cfg, "wide_mode_off_on") && !option_preview_visible(&cfg, "wide_mode_off_off"));
    CHECK(!option_preview_visible(&cfg, "wide_mode_on_on") && option_preview_visible(&cfg, "wide_mode_on_off"));
    CHECK(option_preview_visible(&cfg, "assist_type1_on_1p") && !option_preview_visible(&cfg, "assist_type2_on_2p"));
    CHECK(option_preview_visible(&cfg, "split_boost_low_on") && !option_preview_visible(&cfg, "split_boost_none_on"));
    // On/off pairs: the widget of the current state.
    CHECK(option_preview_visible(&cfg, "gear_auto_on") && !option_preview_visible(&cfg, "gear_auto_off"));
    CHECK(option_preview_visible(&cfg, "force_feedback_on_2p") && !option_preview_visible(&cfg, "force_feedback_off_2p"));
    // Other classes are not previews.
    CHECK(option_preview_visible(&cfg, "song_list") && option_preview_visible(&cfg, "slider_split_race_laps"));
    // Changing a value.
    const GameOption* o = find_game_option_by_item("view/view", "multi_wide_mode");
    CHECK(o != nullptr);
    if (o) set_option(cfg, *o, 1);
    CHECK(option_value(&cfg, "wide_mode") == "on" && option_preview_visible(&cfg, "wide_mode_on_on"));
    cfg.set("options", "menu_se", "garbage");
    CHECK(option_value(&cfg, "menu_se") == "on"); // invalid -> default
}

// License B end to end through the menus: the license page raises the GameRequest, the
// race's judged result (gold) comes back, GT mode records the medal, writes the save, the
// B page shows the cup; test 8 opens after tests 1-7; all 8 -> the license card.
TEST(gtmode_menu_disc_license_b_progress) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    const std::string path = (std::filesystem::temp_directory_path() / "gt3r_test_license_b.sav").string();
    std::remove(path.c_str());
    set_env("GT3R_GTMODE_SAVE", path.c_str());
    set_env("GT3R_GTMODE_NOSAVE", "0");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    choose(f, "gt_mode");
    settle(f);
    choose(f, "lc_lc");
    settle(f);
    CHECK(f.controller()->page_name() == "lc/lc");
    choose(f, "b");
    settle(f);
    CHECK(f.controller()->page_name() == "lc/lc_b");
    auto run_test = [&](const char* code, int medal, double t) {
        CHECK(f.controller()->set_focus(code));
        press(f, MenuKey::Confirm);
        for (int i = 0; i < 120 && !f.waiting_for_game(); ++i) f.tick(0);
        std::optional<GameRequest> rq = f.take_request();
        CHECK(rq.has_value());
        if (!rq) return;
        CHECK(rq->mode == "go_license" && rq->race_code == code);
        // What the race posts for a judged run (src/app/screen_race.cpp post_result).
        GameResult res;
        res.mode = rq->mode;
        res.race_code = rq->race_code;
        res.event = rq->event;
        res.finished = medal >= 0;
        res.position = medal >= 0 ? 1 : 0;
        res.cars = 1;
        res.total_time = medal >= 0 ? t : -1;
        res.license_medal = medal;
        post_game_result(res);
        f.game_finished();
        settle(f);
        // A new record asks for a name: confirm it (OK) and come back to the page.
        if (f.controller() && f.controller()->page_name() == "name_entry") {
            // The original's controls: Triangle / Square only delete (never leave), Start jumps
            // to the OK row, OK ends the page; OK is drawn inside the "menu" window (on screen).
            press(f, MenuKey::Cancel);
            press(f, MenuKey::Cancel);
            CHECK(f.controller()->page_name() == "name_entry");
            press(f, MenuKey::Start);
            const auto* it = f.controller()->focused();
            CHECK(it && it->flow && it->flow->name == "free_command");
            bool ok_shown = false;
            for (const menu::ElementDraw& d : f.draw_list())
                if (d.element && d.element->text && d.element->text->text == "OK" && d.element->y + d.dy >= 30 &&
                    d.element->y + d.dy + d.element->h <= 448)
                    ok_shown = true;
            CHECK(ok_shown);
            press(f, MenuKey::Confirm);
            settle(f);
        }
    };
    run_test("L0B0001", 0, 34.9);
    std::printf("  after the result: control %s page %s\n", f.control().c_str(),
                f.controller() ? f.controller()->page_name().c_str() : "-");
    const auto& lt = s->gt().state().license_tests;
    CHECK(lt[0].medal == game::Medal::Gold && std::fabs(lt[0].best_time - 34.9) < 1e-6);
    CHECK(f.controller()->page_name() == "lc/lc_b");
    // The B page shows the gold cup for test 1.
    {
        bool cup = false;
        for (const menu::ElementDraw& d : f.draw_list())
            if (d.element && d.element->name == "cup_gold") cup = true;
        CHECK(cup);
    }
    // The save was written with the medal.
    {
        std::optional<game::GameState> saved = game::read_save(path);
        CHECK(saved.has_value());
        if (saved) CHECK(saved->license_tests[0].medal == game::Medal::Gold);
    }
    // Test 8 is refused until tests 1-7 hold a medal.
    CHECK(s->gt().can_take_test("L0B0008") == game::Check::NoLastExam);
    const char* codes[] = {"L0B0002", "L0B0003", "L0B0004", "L0B0005", "L0B0006", "L0B0007"};
    for (const char* c : codes) run_test(c, 2, 60.0);
    CHECK(s->gt().can_take_test("L0B0008") == game::Check::Ok);
    CHECK(!s->gt().license_held(0));
    run_test("L0B0008", 1, 60.0);
    CHECK(s->gt().license_held(0));
    std::remove(path.c_str());
    set_env("GT3R_GTMODE_NOSAVE", "1");
}

// Arcade garage classes: the car page shows the garage car's own picture (its label), and
// both players race their garage cars as tuned (2 Player Battle: player 2's spec, model and
// wheel too). A championship round's "Save game" writes the game with the round recorded.
TEST(gtmode_menu_disc_arcade_garage_cars) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    set_env("GT3R_GTMODE_SAVE", "/nonexistent-gt3r-test/gtmode.sav");
    set_env("GT3R_GTMODE_NOSAVE", "1");
    game::install_gt_mode_menu();
    Vfs vfs(&disc->volume());
    GameFlow f(vfs_reader(vfs), "US");
    f.start();
    settle(f);
    game::GtModeSession* s = game::gt_mode_session();
    CHECK(s != nullptr);
    if (!s) return;
    s->gt().state().credits = 1000000;
    CHECK(s->gt().buy_car("r34_gtr_v_99") == 0);
    CHECK(s->gt().buy_car("altezza_rs200_98") == 0);
    const GtModeHooks& h = gt_mode_hooks();
    CHECK(h.arcade_image && h.arcade_prepare);
    if (!h.arcade_image || !h.arcade_prepare) return;
    const game::GarageCar* g0 = game::arcade_garage_car("hg0");
    const game::GarageCar* g1 = game::arcade_garage_car("hg1");
    CHECK(g0 && g1);
    if (!g0 || !g1) return;
    CHECK(h.arcade_image("hg1") == g1->car);
    CHECK(h.arcade_image("class_c").empty());
    GameRequest r;
    r.mode = "two_player_battle";
    r.car = "hg0";
    r.car2 = "hg1";
    h.arcade_prepare(r);
    CHECK(r.car == g0->car && r.car_spec != nullptr);
    CHECK(r.car2 == g1->car && r.car2_spec != nullptr && !r.car2_model.empty() && r.car2_color == g1->color);
    CHECK(!r.racing_gearbox && !r.racing_gearbox2); // stock gearboxes: no whine
    // Player 2's garage car with a non-stock transmission gets the gear whine as player 1's does.
    {
        game::GarageCar& p2 = s->gt().state().garage[1];
        const int gearbox = game::goods_id(17, 3); // full-custom transmission
        CHECK(gearbox >= 0 && s->gt().fit_goods(p2, gearbox));
        GameRequest r2;
        r2.mode = "two_player_battle";
        r2.car = "hg0";
        r2.car2 = "hg1";
        h.arcade_prepare(r2);
        CHECK(!r2.racing_gearbox && r2.racing_gearbox2);
        r2 = GameRequest{};
        r2.mode = "two_player_battle";
        r2.car = "hg1";
        r2.car2 = "hg0";
        h.arcade_prepare(r2);
        CHECK(r2.racing_gearbox && !r2.racing_gearbox2);
        CHECK(s->gt().fit_goods(p2, game::goods_id(17, 0))); // back to stock
    }

    // Save game during a championship: the saved state has the round.
    CHECK(s->gt().start_championship("Galtezza_e_0501"));
    GameResult res;
    res.mode = "go_race";
    res.finished = true;
    res.position = 1;
    CHECK(game::begin_race_game_save(res));
    CHECK(game::snapshot_save() && game::snapshot_save()->championship.next == 1);
    CHECK(s->gt().state().championship.next == 0); // the running game is unchanged
    game::end_race_game_save();
    CHECK(!game::snapshot_save());

    // The save page on its own in Save game mode (the race screen's make_game_save_screen):
    // Cancel ends it, and closing it drops the snapshot.
    CHECK(game::begin_race_game_save(res));
    GameFlow g(vfs_reader(vfs), "US");
    g.start_save_game();
    settle(g);
    CHECK(g.controller() && g.controller()->page_name() == "file_operation" && !g.exited());
    CHECK(g.save_page() && g.save_page()->mode() == SavePage::Mode::SaveGame);
    press(g, MenuKey::Cancel);
    for (int i = 0; i < 60 && !g.exited(); ++i) g.tick(0);
    CHECK(g.exited() && !game::snapshot_save());
    game::end_race_game_save();
}
