// GT Mode: state model, rules, saves (docs/dev/GT_MODE.md).
// Logic tests run without the disc; the rest read the real data via GT3R_DISC.

#include "test_framework.h"

#include "disc/volume.h"
#include "game/gt_mode.h"
#include "game/gt_save.h"
#include "game/gt_setting.h"
#include "race/pool_draw.h"
#include "base/file.h"
#include "formats/ps2/memcard.h"
#include "vfs/vfs.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>

using namespace gt3r;
using namespace gt3r::game;

namespace {

std::shared_ptr<const GtData> disc_data() {
    static std::shared_ptr<GtData> data;
    static bool tried = false;
    if (!tried) {
        tried = true;
        if (Disc* disc = test::test_disc()) {
            auto d = std::make_shared<GtData>();
            const Volume& vol = disc->volume();
            bool ok = d->load([&](const std::string& p) -> std::optional<Bytes> {
                if (p == Vfs::kProgramFile) return vol.read_program();
                const Volume::Entry* e = vol.find(p);
                if (!e) return std::nullopt;
                return vol.read(*e);
            });
            if (ok) data = d;
            else std::printf("  GtData: %s\n", d->error().c_str());
        }
    }
    return data;
}

#define NEED_DATA()                                                                                                    \
    std::shared_ptr<const GtData> data = disc_data();                                                                  \
    if (!data) SKIP("needs GT3R_DISC")

} // namespace

// --- Logic (no disc) ----------------------------------------------------------------

TEST(gtmode_display_power_rule) {
    // The menus truncate after the unit conversion (NTSC-U: HP = PS x 0.98619), never round.
    CHECK(display_power(329.6f, true) == 325);  // a tuned R34's 329.6 PS shows "325HP"
    CHECK(display_power(329.6f, false) == 329); // and "329PS" in PS
    CHECK(display_power(280.0f, true) == 276);
    CHECK(display_power(275.99f, false) == 275);
    CHECK(std::string(power_unit(true)) == "HP" && std::string(power_unit(false)) == "PS");
}

TEST(gtmode_power_peak_sweep) {
    // Sampled every 100 rpm from the first curve point to the rev limit; PS = kgf*m * rpm / 716.2.
    sim::CarSpec s;
    s.engine.curve_rpm = {1000, 4000, 7000};
    s.engine.curve_torque = {98.0f, 392.0f, 196.0f}; // 10 / 40 / 20 kgf*m
    s.engine.rev_limit_rpm = 6500;
    PowerPeak p = spec_power_peak(s);
    // P(r) on 4000..7000: T = 40 - 20 (r - 4000) / 3000 kgf*m; P = T r / 716.2 peaks at r = 5000.
    CHECK(p.rpm == 5000);
    CHECK(std::fabs(p.ps - (40.0f - 20.0f / 3.0f) * 5000.0f / 716.2f) < 0.01f);
    s.engine.rev_limit_rpm = 4500; // the limit cuts the sweep
    CHECK(spec_power_peak(s).rpm == 4500);
}

TEST(gtmode_engine_factor_floor) {
    // Break-in holds 97 % below E = -300 km (physics spec §10.3); oil +5 % below -100 km.
    CarCondition c;
    c.engine_wear_m = -400000, c.oil_m = 0;
    CHECK(std::fabs(GtMode::engine_factor(c) - 0.97f) < 1e-6f);
    c.engine_wear_m = -150000;
    CHECK(std::fabs(GtMode::engine_factor(c) - 0.985f) < 1e-6f);
    c.engine_wear_m = 0, c.oil_m = -200000;
    CHECK(std::fabs(GtMode::engine_factor(c) - 1.05f) < 1e-6f);
}

TEST(gtmode_completion_text_truncates) {
    // File lists: the thumbnail's per cent truncated to one decimal (2.0992 -> "2.0%").
    CHECK(GtMode::completion_file_text(2.0992f) == "2.0%");
    CHECK(GtMode::completion_file_text(0.19f) == "0.1%");
    CHECK(GtMode::completion_file_text(99.0f) == "99.0%");
    CHECK(GtMode::completion_file_text(100.0f) == "100.0%");
}

TEST(gtmode_format_credits) {
    CHECK(format_credits(0) == "0");
    CHECK(format_credits(18000) == "18,000");
    CHECK(format_credits(1500000) == "1,500,000");
    CHECK(format_credits(-2500) == "-2,500");
    CHECK(format_time_ms(62400) == "1'02.400");
}

TEST(gtmode_goods_table) {
    // Goods ranges (spec §6.1).
    CHECK(goods_info(1).table == 0 && goods_info(1).stage == 1);   // racing brakes
    CHECK(goods_info(51).table == 18 && goods_info(51).stage == 1); // sports suspension
    CHECK(goods_info(66).table == 28 && goods_info(66).stage == 12); // dirt tires
    CHECK(goods_info(77).table == 29 && goods_info(77).stage == 12);
    CHECK(goods_info(82).table == 12 && goods_info(82).stage == 4); // turbo stage 4
    CHECK(goods_info(86).table == 13 && goods_info(86).stage == 1); // VCD
    CHECK(goods_info(87).table == -1);
    for (int g = 0; g < kGoodsCount; ++g) {
        Goods x = goods_info(g);
        CHECK(x.table >= 0);
        CHECK(goods_id(x.table, x.stage) == g);
    }
    CHECK(rear_tire_goods(57) == 68);
    CHECK(rear_tire_goods(66) == -1);
    CHECK(shop_item("lsd_15") && shop_item("lsd_15")->goods == 33);
    CHECK(shop_item("ptp_on") && shop_item("ptp_on")->flow == ShopItem::Flow::BuyAndFit);
    CHECK(shop_item("tir_dirt") && shop_item("tir_dirt")->flow == ShopItem::Flow::BuyOnly);
}

TEST(gtmode_engine_factor) {
    CarCondition c;
    CHECK(std::fabs(GtMode::engine_factor(c) - 0.97f) < 1e-5f); // new car
    c.engine_wear_m = 0;
    CHECK(std::fabs(GtMode::engine_factor(c) - 1.0f) < 1e-5f);
    c.engine_wear_m = 600000;
    CHECK(std::fabs(GtMode::engine_factor(c) - 0.98f) < 1e-5f);
    c.engine_wear_m = 0;
    c.oil_m = -300000; // fresh oil
    CHECK(std::fabs(GtMode::engine_factor(c) - 1.05f) < 1e-5f);
    c.oil_m = -50000;
    CHECK(std::fabs(GtMode::engine_factor(c) - 1.025f) < 1e-5f);
}

TEST(gtmode_save_text_roundtrip) {
    GameState s;
    s.credits = 123456789012LL;
    s.day = 42;
    s.races = 7;
    s.wins = 3;
    s.prize_money = 9000;
    s.distance_m = 12345.5;
    s.next_serial = 5;
    s.rng = 0xDEADBEEF;
    GarageCar c;
    c.serial = 4;
    c.car = "r34_gtr_v_99";
    c.color = 2;
    c.wheel = "bbw004_g";
    c.origin = "Gsunday_e_0000";
    c.parts[0] = 0x123456789ABCDEFull;
    c.parts[12] = 42;
    c.owned.set(0), c.owned.set(57), c.owned.set(86);
    c.settings.gears = {3280, 3827, 2360, 1685, 1312, 1000, 793, -999};
    c.settings.final_drive = 3545;
    c.settings.lsd = {0, 7, 0, 30, 0, 15};
    c.cond.odometer_m = 98765.25;
    c.cond.oil_m = -300000;
    s.garage.push_back(c);
    s.ride = 0;
    s.wheels = {"bbw004_g", "with space"};
    s.license_tests[3] = {Medal::Silver, 31.25};
    s.license_tests[47] = {Medal::Gold, 118.7};
    s.race_best["Gsunday_e_0001"] = 0;
    s.race_best["Gff_e_0002"] = 3;
    s.prizes_given.insert("Gsunday_e_0000");
    s.championship.series = "Galtezza_e_0500";
    s.championship.races = {"Galtezza_e_0501", "Galtezza_e_0502"};
    s.championship.next = 1;
    s.championship.entrants = {"a", "b"};
    s.championship.points = {10, 6, 4};

    s.machine.t400 = 12.345;
    s.machine.car400 = "r34_gtr_v_99";
    s.machine.top_speed = 301.5;
    s.machine.car_top = "zz2_00";
    s.ending_short_seen = true;
    // Setting files with their memory-card info (written / overwritten / deleted by the list).
    write_setting_file(s, c, -1, 1760000000, 280);
    write_setting_file(s, c, -1, 1760000100, 290);
    s.setting_files[0].comment = "Grip set";
    write_setting_file(s, c, 0, 1760000200, 300); // overwrite: keeps the name, becomes the newest
    CHECK(s.setting_files.size() == 2 && s.setting_files[1].comment == "Grip set" && s.setting_files[1].power_ps == 300);
    std::string text = save_to_text(s);
    GameState t = load_from_text(text);
    CHECK(t.machine.t400 == 12.345 && t.machine.car400 == "r34_gtr_v_99" && t.machine.t1000 < 0);
    CHECK(t.machine.top_speed == 301.5 && t.machine.car_top == "zz2_00");
    CHECK(t.ending_short_seen && !t.ending_long_seen);
    CHECK(t.credits == s.credits && t.day == 42 && t.races == 7 && t.wins == 3 && t.prize_money == 9000);
    CHECK(t.distance_m == 12345.5 && t.next_serial == 5 && t.rng == 0xDEADBEEF && t.ride == 0);
    CHECK(t.garage.size() == 1);
    if (!t.garage.empty()) {
        const GarageCar& d = t.garage[0];
        CHECK(d.car == c.car && d.color == 2 && d.wheel == c.wheel && d.origin == c.origin && d.serial == 4);
        CHECK(d.parts == c.parts && d.owned == c.owned);
        CHECK(d.settings.gears == c.settings.gears && d.settings.final_drive == 3545 && d.settings.lsd == c.settings.lsd);
        CHECK(d.cond.odometer_m == 98765.25 && d.cond.oil_m == -300000);
    }
    CHECK(t.wheels == s.wheels);
    CHECK(t.setting_files.size() == 2);
    if (t.setting_files.size() == 2) {
        CHECK(t.setting_files[1].comment == "Grip set" && t.setting_files[1].time == 1760000200);
        CHECK(t.setting_files[0].power_ps == 290 && t.setting_files[0].comment.empty());
        GameState u = t;
        CHECK(delete_setting_file(u, 0) && u.setting_files.size() == 1 && !delete_setting_file(u, 5));
    }
    CHECK(t.license_tests[3].medal == Medal::Silver && t.license_tests[3].best_time == 31.25);
    CHECK(t.license_tests[47].medal == Medal::Gold);
    CHECK(t.race_best == s.race_best && t.prizes_given == s.prizes_given);
    CHECK(t.championship.active() && t.championship.races == s.championship.races &&
          t.championship.points == s.championship.points && t.championship.entrants == s.championship.entrants);
    // Stable output.
    CHECK(save_to_text(t) == text);
    // Damaged / foreign files are rejected.
    bool threw = false;
    try {
        load_from_text("GT3R-GTMODE 99\nend\n");
    } catch (const FormatError&) {
        threw = true;
    }
    CHECK(threw);
    threw = false;
    try {
        load_from_text(text.substr(0, text.size() / 2));
    } catch (const FormatError&) {
        threw = true;
    }
    CHECK(threw);
}

// Seeded time-trial records keep the original's name marks (0xFF - 3 i - k for the k-th
// record of challenge i); older native saves named them "-" / nothing, or 0xFF / 0xFE /
// 0xFD in every challenge, and get the challenge's marks on load.
TEST(gtmode_save_time_trial_marks) {
    std::string text = save_to_text(GameState{});
    const size_t end = text.rfind("end\n");
    CHECK(end != std::string::npos);
    if (end == std::string::npos) return;
    text.insert(end, "rank at:AT020001 107.968 - 0 %2D\n"
                     "rank at:AT020001 109.021 - 0 -\n"
                     "rank at:AT020001 109.5 nsx_s_zero_97 3 MATT\n"
                     "rank at:AT020001 110.756 - 0 %FD\n"
                     "rank at:AT030001 122.27 - 0 %F9\n"
                     "rank at:AT030001 123.839 - 0 %FF\n"
                     "rank lic:A-1 30.5 - 0 -\n");
    GameState s = load_from_text(text);
    const std::vector<RankEntry>& a = s.rankings["at:AT020001"];
    CHECK(a.size() == 4);
    if (a.size() == 4) {
        CHECK(a[0].name == GtMode::time_trial_seed_name(1, 0) && u8(a[0].name[0]) == 0xFC);
        CHECK(a[1].name == GtMode::time_trial_seed_name(1, 1) && a[2].name == "MATT");
        CHECK(a[3].name == GtMode::time_trial_seed_name(1, 2) && u8(a[3].name[0]) == 0xFA);
    }
    const std::vector<RankEntry>& b = s.rankings["at:AT030001"];
    CHECK(b.size() == 2);
    if (b.size() == 2) CHECK(u8(b[0].name[0]) == 0xF9 && u8(b[1].name[0]) == 0xF8);
    CHECK(s.rankings["lic:A-1"].size() == 1 && s.rankings["lic:A-1"][0].name.empty()); // only time trials
    // The marks survive the native save as they are.
    GameState t = load_from_text(save_to_text(s));
    CHECK(t.rankings["at:AT020001"].size() == 4 && t.rankings["at:AT020001"][3].name == a[3].name);
}

// --- Disc data -----------------------------------------------------------------------

TEST(gtmode_disc_constants) {
    NEED_DATA();
    CHECK(data->starting_credits() == 18000); // NTSC-U new game
    CHECK(data->car_wash_price() == 50);
    CHECK(data->oil_change_price() == 250);
    CHECK(data->wheel_price() == 500);
    CHECK(data->race_codes.size() == 364);
    GtMode gt(data);
    CHECK(gt.state().credits == 18000 && gt.state().day == 1 && gt.state().garage.empty() && gt.state().ride == -1);
}

TEST(gtmode_disc_catalog) {
    NEED_DATA();
    const Catalog& c = data->catalog;
    CHECK(c.dealers.size() == 151);
    int buyable = 0, cheap = 0;
    GtMode gt(data);
    for (const DealerCar& d : c.dealers) {
        CHECK(data->db.car(d.car) != nullptr);
        if (!(gt.car_flags(d.car) & 1)) {
            ++buyable;
            if (gt.car_price(d.car) <= 18000) ++cheap;
        }
    }
    CHECK(buyable == 124);
    CHECK(cheap == 11);
    // Leagues: 20 beginner, 23 amateur, 22 professional, 10 rally, 10 endurance series.
    std::map<std::string, int> per_tier;
    for (const RaceSeries& s : c.series) ++per_tier[s.tier];
    CHECK(per_tier["beginner"] == 20 && per_tier["amateur"] == 23 && per_tier["pro"] == 22);
    CHECK(per_tier["dirt"] == 10 && per_tier["end"] == 10);
    for (const RaceSeries& s : c.series)
        for (const std::string& r : s.races) CHECK(data->db.event(r) != nullptr);
    CHECK(c.licenses.size() == 6);
    for (const LicenseCourse& l : c.licenses) CHECK(l.tests.size() == 8);
    // Prize sets: every candidate car exists.
    int cars = 0;
    for (const PrizeSet& p : data->prizes)
        for (const PrizeCar& pc : p.cars) {
            CHECK(data->db.car(pc.car) != nullptr);
            ++cars;
        }
    CHECK(cars >= 200);
    const PrizeSet* sunday = data->prize_set("Gsunday_e_0000");
    CHECK(sunday && sunday->cars.size() == 1 && sunday->cars[0].car == "trueno_gt_apex_85");
}

TEST(gtmode_disc_dealer_and_garage) {
    NEED_DATA();
    GtMode gt(data);
    // Too expensive at the start.
    CHECK(gt.can_buy_car("r34_gtr_v_99") == Check::NoMoney);
    CHECK(gt.can_buy_car("_787b_91") == Check::NotForBuy);
    // The cheapest dealer car.
    std::string cheapest;
    for (const DealerCar& d : data->catalog.dealers)
        if (!(gt.car_flags(d.car) & 1) && (cheapest.empty() || gt.car_price(d.car) < gt.car_price(cheapest)))
            cheapest = d.car;
    CHECK(gt.car_price(cheapest) == 11140);
    CHECK(gt.buy_car(cheapest, 1) == 0);
    CHECK(gt.state().credits == 18000 - 11140);
    CHECK(gt.state().ride == 0); // the first car becomes the ride car
    const GarageCar& car = gt.state().garage[0];
    CHECK(car.cond.engine_wear_m == -300000);
    // Stock parts are owned (e.g. the stock tires, goods 56).
    CHECK(car.owned.test(56) && car.owned.test(67));
    // A second purchase goes to the head of the list, the ride car stays the same car.
    gt.state().credits += 100000;
    CHECK(gt.buy_car(cheapest) == 0);
    CHECK(gt.state().garage.size() == 2 && gt.state().ride == 1);
    // Sell: a quarter of the catalogue price; selling the ride car leaves no car.
    s64 before = gt.state().credits;
    CHECK(gt.sell_car(1) == 11140 / 4);
    CHECK(gt.state().credits == before + 2785);
    CHECK(gt.state().ride == -1);
    CHECK(gt.can_enter("Gsunday_e_0001") == Check::NoRideCar);
}

TEST(gtmode_disc_trade) {
    NEED_DATA();
    // The guest garage: a friend's game with an R34 (odometer, fitted wheel).
    GtMode guest(data);
    guest.state().credits = 1000000;
    CHECK(guest.buy_car("r34_gtr_v_99") == 0);
    GarageCar& theirs = guest.state().garage[0];
    theirs.cond.odometer_m = 12345;
    theirs.wheel = "some_wheel";
    GtMode gt(data);
    // Guard order: not for sale (model bit 1), money, garage full.
    CHECK(gt.can_buy_guest_car(theirs) == Check::NoMoney);
    GarageCar prize = theirs;
    prize.car = "roadster_sspl_96";
    CHECK(gt.car_flags(prize.car) & 2);
    CHECK(gt.can_buy_guest_car(prize) == Check::NotForBuy);
    gt.state().credits = 100000;
    const s64 price = gt.car_price("r34_gtr_v_99");
    CHECK(gt.buy_guest_car(theirs) == 0);
    // A copy at the full catalogue price, with its condition but the stock wheel.
    CHECK(gt.state().credits == 100000 - price);
    CHECK(gt.state().garage.size() == 1 && gt.state().ride == 0);
    CHECK(gt.state().garage[0].car == "r34_gtr_v_99");
    CHECK(gt.state().garage[0].cond.odometer_m == 12345);
    CHECK(gt.state().garage[0].wheel.empty());
    // The friend keeps the car.
    CHECK(guest.state().garage.size() == 1 && guest.state().garage[0].wheel == "some_wheel");
}

TEST(gtmode_disc_tune_shop) {
    NEED_DATA();
    GtMode gt(data);
    gt.state().credits = 1000000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    // Turbo stage prices (R34: 4,800 / 13,500 / 42,500 / 80,000).
    const int tbk[4] = {4800, 13500, 42500, 80000};
    for (int i = 0; i < 4; ++i) {
        auto o = gt.part_offer(79 + i);
        CHECK(o && o->price == u32(tbk[i]));
    }
    CHECK(gt.can_buy_part(41) == Check::NotExist); // no NA tune for a turbo car
    float ps0 = gt.ride_power_ps();
    s64 c0 = gt.state().credits;
    CHECK(gt.buy_part(81));
    CHECK(gt.state().credits == c0 - 42500);
    CHECK(gt.can_buy_part(81) == Check::AlreadyHave);
    CHECK(gt.power_with(81) > ps0 * 1.2f);
    CHECK(gt.fit_part(81));
    CHECK(gt.ride_power_ps() > ps0 * 1.2f);
    CHECK(gt.tuned(*gt.state().ride_car()));
    // Back to stock: the stage-0 goods.
    CHECK(gt.fit_part(78));
    CHECK(std::fabs(gt.ride_power_ps() - ps0) < 0.5f);
    // Racing tires: front price only, the rear set comes with it and is fitted with it.
    auto soft = gt.part_offer(63);
    CHECK(soft && soft->price == 35000);
    CHECK(gt.buy_part(63) && gt.owns(74));
    CHECK(gt.fit_part(63));
    const GarageCar& r = *gt.state().ride_car();
    CHECK(r.parts[size_t(part_slot(28))] == find_part(data->db, "r34_gtr_v_99", 63)->id);
    CHECK(r.parts[size_t(part_slot(29))] == find_part(data->db, "r34_gtr_v_99", 74)->id);
    // Buy all racing tires: the six not owned yet.
    s64 all = gt.all_tires_price();
    CHECK(all == 10500 + 13500 + 16500 + 22500 + 27500 + 47500);
    CHECK(gt.buy_all_tires());
    CHECK(gt.can_buy_all_tires() == Check::AlreadyHaveTires);
    // Wash / oil / wheels.
    s64 c1 = gt.state().credits;
    CHECK(gt.wash() && gt.change_oil());
    CHECK(gt.state().credits == c1 - 300);
    CHECK(gt.buy_wheel("bbw004_g"));
    CHECK(gt.can_buy_wheel("bbw004_g") == Check::AlreadyHave);
    CHECK(gt.fit_wheel(0, "bbw004_g"));
}

TEST(gtmode_disc_races_and_prizes) {
    NEED_DATA();
    GtMode gt(data);
    // Sunday Cup pays 1,000 / 900 / 800 / 700 / 600 / 500.
    auto rules = gt.rules("Gsunday_e_0001");
    CHECK(rules && rules->prize[0] == 1000 && rules->prize[5] == 500);
    gt.state().credits = 100000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    CHECK(gt.can_enter("Gsunday_e_0001") == Check::Ok);
    const RaceSeries* s = data->catalog.series_for("Gsunday_e_0001");
    CHECK(s != nullptr && !s->championship());
    s64 c0 = gt.state().credits;
    Award a = gt.record_race("Gsunday_e_0001", 2);
    CHECK(a.money == 800 && gt.state().credits == c0 + 800 && gt.state().best_position("Gsunday_e_0001") == 2);
    // The cup (3rd, beginner class 5): a trophy and award bit 1 << (0 x 3 + 2); 4th: the
    // consolation prize's bit 0x10000, no trophy.
    CHECK(rules->trophy_class == 5 && gt.state().trophies == 1 && gt.state().arcade.awards == 0x4);
    gt.record_race("Gsunday_e_0001", 4);
    CHECK(gt.state().trophies == 1 && gt.state().arcade.awards == 0x10004);
    // Winning every race of the series gives the series prize car once.
    size_t cars = gt.state().garage.size();
    for (const std::string& r : s->races) a = gt.record_race(r, 0);
    CHECK(gt.state().garage.size() == cars + 1);
    CHECK(gt.state().garage[0].car == "trueno_gt_apex_85");
    CHECK(gt.state().prizes_given.count("Gsunday_e_0000") == 1);
    for (const std::string& r : s->races) gt.record_race(r, 0);
    CHECK(gt.state().garage.size() == cars + 1); // only once
    CHECK(gt.state().wins == u32(2 * s->races.size()));
}

TEST(gtmode_disc_entry_rules) {
    NEED_DATA();
    GtMode gt(data);
    gt.state().credits = 10000000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    const GarageCar& r34 = *gt.state().ride_car();
    // The amateur FF Challenge needs the A license (0x19B = 2).
    auto amateur = gt.rules("Gff_n_0001");
    CHECK(amateur && amateur->required_license == 2);
    EntryResult e = gt.check_entry("Gff_n_0001", r34);
    CHECK(e.license);   // no license yet
    CHECK(e.drivetrain); // FF series
    // NA-only series.
    auto na = gt.rules("Gna_e_0001");
    CHECK(na && na->aspiration == 1);
    CHECK(gt.check_entry("Gna_e_0001", r34).aspiration);
    // Regulation list.
    auto altezza = gt.rules("Galtezza_e_0501");
    CHECK(altezza && altezza->regulation == "reg_altz");
    CHECK(gt.check_entry("Galtezza_e_0501", r34).regulation);
    CHECK(gt.can_enter("Gff_n_0001") == Check::LicenseViolation);
    CHECK(gt.can_enter("Galtezza_e_0501") == Check::LicenseViolation); // needs B too: the license popup wins
    CHECK(gt.can_enter("Gevolution_e_0001") == Check::RegulationViolation); // reg_evo, no license
    // Dirt events need dirt tires.
    auto dirt = gt.rules("Dtahiticircuit_0001");
    CHECK(dirt && dirt->dirt_tires);
}

TEST(gtmode_disc_licenses) {
    NEED_DATA();
    GtMode gt(data);
    auto b1 = gt.rules("L0B0001");
    CHECK(b1 && b1->license_ms[0] == 35100 && b1->license_ms[1] == 35400 && b1->license_ms[2] == 35800);
    CHECK(gt.judge("L0B0001", true, 35.0) == Medal::Gold);
    CHECK(gt.judge("L0B0001", true, 35.3) == Medal::Silver);
    CHECK(gt.judge("L0B0001", true, 35.8) == Medal::Bronze);
    CHECK(gt.judge("L0B0001", true, 36.0) == Medal::None);
    CHECK(gt.judge("L0B0001", false, 30.0) == Medal::None);
    CHECK(gt.can_open_license_page(1) == Check::NoLicense);
    CHECK(gt.can_open_license_page(5) == Check::Ok); // rally always open
    CHECK(gt.can_take_test("L0B0008") == Check::NoLastExam);
    Award a;
    for (int t = 1; t <= 7; ++t) {
        std::string code = "L0B000" + std::to_string(t);
        auto r = gt.rules(code);
        a = gt.record_license(code, true, r->license_ms[2] / 1000.0); // bronze
        CHECK(a.medal == Medal::Bronze);
    }
    CHECK(!gt.license_held(0));
    CHECK(gt.can_take_test("L0B0008") == Check::Ok);
    a = gt.record_license("L0B0008", true, gt.rules("L0B0008")->license_ms[2] / 1000.0);
    CHECK(a.license_obtained && gt.license_held(0) && gt.highest_license() == 0);
    CHECK(gt.can_open_license_page(1) == Check::Ok);
    // All gold: the B license prize car, once.
    size_t cars = gt.state().garage.size();
    for (int t = 1; t <= 8; ++t) {
        std::string code = "L0B000" + std::to_string(t);
        a = gt.record_license(code, true, gt.rules(code)->license_ms[0] / 1000.0);
    }
    CHECK(gt.license_all_gold(0));
    CHECK(gt.state().garage.size() == cars + 1);
    CHECK(!gt.state().garage.empty() && gt.state().garage[0].car == "roadster_rs_00_ext");
    a = gt.record_license("L0B0001", true, 30.0);
    CHECK(gt.state().garage.size() == cars + 1);
}

TEST(gtmode_disc_championship) {
    NEED_DATA();
    GtMode gt(data);
    gt.state().credits = 10000000;
    const RaceSeries* s = data->catalog.series_for("Galtezza_e_0501");
    CHECK(s && s->championship() && s->races.size() == 5);
    CHECK(gt.start_championship("Galtezza_e_0501"));
    CHECK(gt.championship_field().size() == 5);
    s64 c0 = gt.state().credits;
    Award a;
    for (size_t i = 0; i < s->races.size(); ++i) a = gt.record_round(0);
    CHECK(a.championship_over && a.champion);
    auto r = gt.rules(s->races.back());
    CHECK(a.bonus == s64(r->champ_bonus) && a.bonus > 0);
    s64 rounds = 0;
    for (const std::string& code : s->races) rounds += gt.rules(code)->prize[0];
    CHECK(gt.state().credits == c0 + rounds + a.bonus);
    CHECK(gt.state().best_position("Galtezza_e_0500") == 0);
    CHECK(a.new_cars.size() == 1);
    CHECK(!gt.state().championship.active());
    // 2nd in every round: the round winner takes the title.
    CHECK(gt.start_championship("Galtezza_e_0501"));
    for (size_t i = 0; i < s->races.size(); ++i) a = gt.record_round(1);
    CHECK(a.championship_over && !a.champion);
}

TEST(gtmode_disc_car_spec_bridge) {
    NEED_DATA();
    GtMode gt(data);
    gt.state().credits = 1000000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    GarageCar& c = *gt.state().ride_car();
    auto stock = sim::build_car_spec(data->db, "r34_gtr_v_99");
    auto mine = gt.car_spec(c);
    CHECK(stock && mine);
    if (stock && mine) {
        // A new car runs at 97 % (break-in).
        CHECK(std::fabs(mine->peak_torque() / stock->peak_torque() - 0.97f) < 1e-3f);
        // User settings reach the spec: final drive.
        c.cond.engine_wear_m = 0;
        c.settings.final_drive = 4000;
        auto tuned = gt.car_spec(c);
        CHECK(tuned && std::fabs(tuned->gearbox.final_drive - 4.0f) < 1e-3f);
        CHECK(tuned && std::fabs(tuned->peak_torque() - stock->peak_torque()) < 1e-2f);
    }
    // Odometer, totals and wear.
    gt.drive(0, 1500.0);
    CHECK(gt.state().garage[0].cond.odometer_m == 1500.0 && gt.state().distance_m == 1500.0);
}

TEST(gtmode_disc_ps2_save_roundtrip) {
    NEED_DATA();
    GtMode gt(data);
    gt.state().credits = 5000000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    CHECK(gt.buy_part(81) && gt.fit_part(81));
    gt.buy_wheel("bbw004_g");
    gt.fit_wheel(0, "bbw004_g");
    gt.drive(0, 12346);
    gt.record_race("Gsunday_e_0001", 0);
    gt.record_license("L0B0001", true, 35.0);
    for (const std::string& r : data->catalog.series_for("Gsunday_e_0001")->races) gt.record_race(r, 0);
    gt.state().day = 17;
    Bytes file = export_ps2_save(gt.state(), *data);
    CHECK(file.size() % 64 == 0);
    CHECK(rd32(file, 4) == 0x112BC + 0x204 * gt.state().garage.size() + 0x44 * (gt.state().wheels.size() + 1));
    GameState s = import_ps2_save(file, *data);
    const GameState& o = gt.state();
    CHECK(s.credits == o.credits && s.day == 17 && s.races == o.races && s.wins == o.wins);
    CHECK(s.garage.size() == o.garage.size() && s.ride == o.ride);
    for (size_t i = 0; i < s.garage.size() && i < o.garage.size(); ++i) {
        CHECK(s.garage[i].car == o.garage[i].car && s.garage[i].parts == o.garage[i].parts);
        CHECK(s.garage[i].owned == o.garage[i].owned && s.garage[i].wheel == o.garage[i].wheel);
        CHECK(s.garage[i].settings.gears == o.garage[i].settings.gears);
        CHECK(std::fabs(s.garage[i].cond.odometer_m - o.garage[i].cond.odometer_m) <= 2);
    }
    CHECK(s.wheels == o.wheels);
    CHECK(s.license_tests[0].medal == Medal::Gold);
    CHECK(s.race_best == o.race_best);
    CHECK(s.prizes_given == o.prizes_given);
    // A flipped byte fails the CRC.
    file[0x80] ^= 1;
    bool threw = false;
    try {
        import_ps2_save(file, *data);
    } catch (const FormatError&) {
        threw = true;
    }
    CHECK(threw);
}

// The PS2 payload (header removed, bytes un-inverted).
static Bytes ps2_payload(const Bytes& file) {
    Bytes p(file.begin() + 0x40, file.begin() + 0x40 + rd32(file, 4));
    for (u8& b : p) b = u8(~b);
    return p;
}

// Verified against a new game saved by the original (PCSX2): exporting the imported
// save reproduces the original file byte for byte; this checks the parts we write
// for a game that never came from the PS2 (built-in time-trial records, empty
// rankings, the idle championship) and that the export is stable.
TEST(gtmode_disc_ps2_new_game_layout) {
    NEED_DATA();
    GtMode gt(data);
    Bytes file = export_ps2_save(gt.state(), *data);
    CHECK(file.size() == 70464 && rd32(file, 4) == 70400);
    Bytes p = ps2_payload(file);
    const size_t at = 0x531C; // payload offset of the arcade time-trial rankings (no cars, 1 wheel)
    // AT01: 1'25.553, 1'26.952, 1'27.355, built-in names 0xFF / 0xFE / 0xFD, no best rank.
    CHECK(rd32(p, at) == 85553 && rd32(p, at + 4) == 86952 && rd32(p, at + 8) == 87355);
    CHECK(rd32(p, at + 12) == 0x157529FF);
    CHECK(p[at + 0x30] == 0xFF && p[at + 0x40] == 0xFE && p[at + 0x50] == 0xFD && p[at + 0x60] == 0);
    CHECK(rd32(p, at + 0xF0) == 0x3264FD64 && rd32(p, at + 0xFC) == 0);
    CHECK(p[at + 0x120] == '-' && p[at + 0x13D] == '-' && p[at + 0x13E] == 0);
    CHECK(rd32(p, at + 0x2A0) == 8);
    // The marks count down over the challenges: AT02 0xFC / 0xFB / 0xFA ... AT10 0xE4 / 0xE3 / 0xE2.
    for (size_t i = 0; i < 10; ++i)
        for (size_t k = 0; k < 3; ++k) CHECK(p[at + 0x2A4 * i + 0x30 + 16 * k] == 0xFF - 3 * i - k);
    // A new game's options (all sound on, the race songs in the music list): a zeroed
    // block turned the original's sound off and emptied its music list (PCSX2 check).
    const size_t opt = p.size() - 0x888;
    CHECK(rd32(p, opt) == 0 && rd32(p, opt + 4) == 1 && rd32(p, opt + 0x18) == 1);
    CHECK(!data->race_songs.empty() && rd32(p, opt + 0x228) == data->race_songs.size());
    CHECK(rd64(p, opt + 0x238) == db::id_hash(data->race_songs[0]) && p[opt + 0x241] == 1);
    // The idle championship and the ride index copy (+0x2C, -1 without a car).
    CHECK(rd32(p, 0x2C) == 0xFFFFFFFFu && rd64(p, 0xFB4C + 0x94) == ~0ull);
    // Import gives the built-in records back as rankings with their marks; a second export
    // is identical, also through the native save.
    GameState s = import_ps2_save(file, *data);
    CHECK(s.rankings["at:AT010001"].size() == 3 && s.rankings["at:AT010001"][0].name == GtMode::time_trial_seed_name(0, 0));
    CHECK(s.rankings["at:AT050001"].size() == 3 && s.rankings["at:AT050001"][2].name == GtMode::time_trial_seed_name(4, 2) &&
          s.rankings["at:AT050001"][2].car.empty());
    {
        const std::vector<RankEntry> seeds = GtMode::time_trial_seed_ranking(data->db, "AT050001");
        CHECK(seeds.size() == 3);
        for (size_t k = 0; k < seeds.size() && k < 3; ++k)
            CHECK(seeds[k].name == s.rankings["at:AT050001"][k].name &&
                  std::fabs(seeds[k].value - s.rankings["at:AT050001"][k].value) < 1e-6);
    }
    CHECK(export_ps2_save(load_from_text(save_to_text(s)), *data) == file);
    CHECK(s.ps2_options.size() == 0x888);
    CHECK(export_ps2_save(s, *data) == file);
    GtMode with_car(data);
    with_car.state().credits = 1000000;
    CHECK(with_car.buy_car("r34_gtr_v_99") >= 0);
    CHECK(rd32(ps2_payload(export_ps2_save(with_car.state(), *data)), 0x2C) == 0);
    CHECK(load_from_text(save_to_text(s)).ps2_options == s.ps2_options);
}

TEST(gtmode_disc_ps2_rankings_and_options) {
    NEED_DATA();
    GtMode gt(data);
    gt.state().player_name = "MATT";
    CHECK(gt.arcade_time_trial("AT010001", 84.5, "nsx_s_zero_97") == 0);
    CHECK(gt.state().arcade.at_best_rank[0] == 0);
    Bytes opt(0x888);
    for (size_t i = 0; i < opt.size(); ++i) opt[i] = u8(i * 13);
    gt.state().ps2_options = opt;
    GameState s = import_ps2_save(export_ps2_save(gt.state(), *data), *data);
    const std::vector<RankEntry>& r = s.rankings["at:AT010001"];
    CHECK(r.size() == 4 && std::fabs(r[0].value - 84.5) < 1e-6 && r[0].name == "MATT");
    CHECK(r[1].name == GtMode::time_trial_seed_name(0, 0) && r[1].car.empty() && std::fabs(r[1].value - 85.553) < 1e-6);
    CHECK(s.arcade.at_best_rank[0] == 0);
    CHECK(s.ps2_options == opt);
    // The player's record names the car by its catalogue name on the card.
    Bytes p = ps2_payload(export_ps2_save(gt.state(), *data));
    const data::Car* c = data->db.car(r[0].car);
    CHECK(c && std::string(reinterpret_cast<const char*>(&p[0x531C + 0x120])) == c->name);
}

TEST(gtmode_disc_ps2_save_containers) {
    NEED_DATA();
    Disc* disc = test::test_disc();
    const Volume& vol = disc->volume();
    GtData::Reader read = [&](const std::string& path) -> std::optional<Bytes> {
        const Volume::Entry* e = vol.find(path);
        if (!e) return std::nullopt;
        return vol.read(*e);
    };
    GtMode gt(data);
    gt.state().credits = 1000000;
    CHECK(gt.buy_car("r34_gtr_v_99") >= 0);
    gt.state().file_comment = "MY GAME";
    std::vector<Ps2SaveFile> files = export_ps2_save_files(gt.state(), data, read);
    CHECK(files.size() == 4 && files[0].name == "icon0.ico" && files[1].name == "icon.sys" &&
          files[2].name == kPs2GameDir && files[3].name == "thumbnail");
    // icon.sys: the title "GT3 GAME DATA" in full-width Shift-JIS on one line, icon names.
    const Bytes& sys = files[1].data;
    CHECK(sys.size() == 964 && rd16(sys, 6) == 26 && sys[0xC0] == 0x82 && sys[0xC1] == 0x66);
    CHECK(std::memcmp(&sys[0x104], "icon0.ico", 10) == 0 && std::memcmp(&sys[0x184], "icon0.ico", 10) == 0);
    // Thumbnail: credits, cars, license (-1 none), day, completion, comment.
    const Bytes& th = files[3].data;
    CHECK(th.size() == 0x60 && s64(rd64(th, 8)) == gt.state().credits && rd32(th, 0x10) == 1);
    CHECK(s32(rd32(th, 0x14)) == -1 && rd32(th, 0x18) == gt.state().day);
    gt.record_license("L0B0001", true, 35.0); // completion in per cent (1 of 524 = 0.19 %)
    float pc;
    std::memcpy(&pc, &ps2_thumbnail(gt.state(), data)[0x1C], 4);
    CHECK(std::fabs(pc - 100.0f / 524) < 0.01f);
    CHECK(std::memcmp(&th[0x28], "MY GAME", 8) == 0);

    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "gt3r_test_ps2_containers";
    fs::remove_all(dir);
    fs::create_directories(dir);
    for (const char* out : {"card", "game.psu", "game.ps2"}) {
        const std::string path = (dir / out).string();
        export_ps2_save_path(gt.state(), data, read, path);
        GameState s = import_ps2_save_path(path, *data);
        CHECK(s.garage.size() == 1 && s.garage[0].car == "r34_gtr_v_99" && s.credits == gt.state().credits);
    }
    // The folder export is a PCSX2 folder memory card; its save folder and data file import too.
    for (const fs::path& p : {dir / "card" / kPs2GameDir, dir / "card" / kPs2GameDir / kPs2GameDir})
        CHECK(import_ps2_save_path(p.string(), *data).garage.size() == 1);
    CHECK(read_host_file((dir / "card" / kPs2GameDir / "icon.sys").string())->size() == 964);
    fs::remove_all(dir);
}

// Writes a mid-career demo save for screenshots when GT3R_GTMODE_DEMO_SAVE names a path.
TEST(gtmode_disc_demo_save) {
    const char* out = std::getenv("GT3R_GTMODE_DEMO_SAVE");
    if (!out || !*out) SKIP("set GT3R_GTMODE_DEMO_SAVE to write a demo save");
    NEED_DATA();
    GtMode gt(data);
    CHECK(gt.buy_car("vitz_euro_00") >= 0 || gt.buy_car("mira_tr_xx_avar_97") >= 0);
    for (const std::string& r : data->catalog.series_for("Gsunday_e_0001")->races) gt.enter(), gt.record_race(r, 0);
    for (int t = 1; t <= 8; ++t) {
        std::string code = "L0B000" + std::to_string(t);
        gt.enter();
        gt.record_license(code, true, gt.rules(code)->license_ms[t % 3] / 1000.0);
    }
    gt.state().credits += 60000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    gt.buy_part(79);
    gt.fit_part(79);
    gt.drive(0, 123400);
    gt.drive(1, 45600);
    CHECK(write_save(out, gt.state()));
}

// Writes a new-game save with a red Miata bought (the reference captures' state: day 1,
// 1,000 Cr.) when GT3R_GTMODE_MIATA_SAVE names a path.
TEST(gtmode_disc_miata_save) {
    const char* out = std::getenv("GT3R_GTMODE_MIATA_SAVE");
    if (!out || !*out) SKIP("set GT3R_GTMODE_MIATA_SAVE to write a screenshot save");
    NEED_DATA();
    GtMode gt(data);
    std::string miata;
    for (const data::Car& c : data->db.cars())
        if (c.name.find("Miata") != std::string::npos && gt.car_price(c.label) == 17000) miata = c.label;
    CHECK(!miata.empty() && gt.buy_car(miata) == 0);
    CHECK(gt.state().credits == 1000);
    CHECK(write_save(out, gt.state()));
}

// Writes a demo save whose ride car has adjustable parts (settings menu screenshots)
// when GT3R_GTMODE_DEMO_TUNED names a path.
TEST(gtmode_disc_demo_tuned_save) {
    const char* out = std::getenv("GT3R_GTMODE_DEMO_TUNED");
    if (!out || !*out) SKIP("set GT3R_GTMODE_DEMO_TUNED to write a demo save");
    NEED_DATA();
    GtMode gt(data);
    gt.state().credits = 2000000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    for (int g : {53, 22, 34, 3, 86, 80})
        if (gt.buy_part(g)) gt.fit_part(g);
    CHECK(write_save(out, gt.state()));
}

TEST(gtmode_disc_setting_files) {
    NEED_DATA();
    GtMode gt(data);
    gt.state().credits = 5000000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    for (int g : {53, 34})
        if (gt.buy_part(g)) gt.fit_part(g);
    GarageCar& car = *gt.state().ride_car();
    car.settings.spring = {100, 120};
    car.settings.lsd[1] = 33;
    save_setting_file(gt.state(), car);
    CHECK(gt.state().setting_files.size() == 1);
    // Change things, then load: everything comes back.
    car.settings.spring = {60, 60};
    car.settings.lsd[1] = 10;
    const SettingFile* f = latest_setting_file(gt.state(), car.car);
    CHECK(f && apply_setting_file(*f, car) == 0);
    CHECK(car.settings.spring[0] == 100 && car.settings.lsd[1] == 33);
    // A different suspension kit: the suspension group is skipped (load_warning).
    CHECK(gt.buy_part(52) && gt.fit_part(52));
    int spring = car.settings.spring[0];
    CHECK(apply_setting_file(*latest_setting_file(gt.state(), car.car), car) == 1);
    CHECK(car.settings.spring[0] == spring && car.settings.lsd[1] == 33);
    // Another car's file applies nothing.
    SettingFile other = *f;
    other.car = "nsx_r_92";
    CHECK(apply_setting_file(other, car) == -1);
    // At most 10 per car.
    for (int i = 0; i < 12; ++i) save_setting_file(gt.state(), car);
    CHECK(gt.state().setting_files.size() == size_t(kSettingFilesPerCar));
    // Native save and the original's 'S' file both keep them.
    GameState t = load_from_text(save_to_text(gt.state()));
    CHECK(t.setting_files.size() == gt.state().setting_files.size());
    CHECK(t.setting_files.back().settings.lsd == gt.state().setting_files.back().settings.lsd);
    CHECK(t.setting_files.back().parts == gt.state().setting_files.back().parts);
    Bytes ps2 = export_ps2_setting(gt.state().setting_files.back(), *data);
    CHECK(rd16(ps2, 0x0A) == 1 && rd16(ps2, 0x08) == 0x827 && rd32(ps2, 4) == 0x1D0);
    SettingFile back = import_ps2_setting(ps2, *data);
    CHECK(back.car == car.car && back.parts == gt.state().setting_files.back().parts);
    CHECK(back.settings.spring == gt.state().setting_files.back().settings.spring);
    CHECK(back.ref_final == gt.state().setting_files.back().ref_final);
}

// --- Arcade progression (spec §16) ---------------------------------------------------

namespace {

// Wins every Single Race course of group `g` at difficulty letter `d` (E/N/H/P).
void arcade_win_group(GtMode& gt, const GtData& data, int g, char d) {
    for (const GtData::ArcadeCourse& c : data.arcade_courses.at("SINGLE_RACE"))
        if (c.level == g) gt.arcade_single_race(std::string("A") + d + "CT0001", c.course, 0);
}

size_t open_cars(const GtMode& gt, const char* cls) { return gt.arcade_cars(cls).size(); }

} // namespace

TEST(gtmode_disc_arcade_data) {
    NEED_DATA();
    const auto& sr = data->arcade_courses.at("SINGLE_RACE");
    CHECK(sr.size() == 34);
    int per_level[6] = {};
    for (const GtData::ArcadeCourse& c : sr)
        if (c.level >= 0 && c.level < 6) ++per_level[c.level];
    CHECK(per_level[0] == 6 && per_level[1] == 6 && per_level[2] == 6 && per_level[3] == 6 && per_level[4] == 5 &&
          per_level[5] == 5);
    CHECK(sr[0].course == "super" && sr[5].course == "r_midfield" && sr[6].course == "r_smtnorth" &&
          sr[33].course == "r_route5");
    CHECK(data->arcade_courses.at("TIME_ATTACK").size() == 36 && data->arcade_courses.at("TWO_PLAYER").size() == 35);
    CHECK(data->arcade_classes.at("CLASS_A").size() == 14 && data->arcade_classes.at("CLASS_S").size() == 21 &&
          data->arcade_classes.at("CLASS_COMPULSORY").size() == 10);
    const auto& b = data->arcade_classes.at("CLASS_B");
    CHECK(b.back().label == "arm_tt_00" && b.back().level == 2 && b.back().image() == "tt_00");
    CHECK(b[0].label == "s2000_typev_00_247" && b[0].image() == "s2000_typev_00" && b[0].level == 0);
    CHECK(data->arcade_classes.at("CLASS_COMPULSORY")[0].race_code == "AT010001");
    GtMode gt(data);
    // New game: group 0 of the courses, the level-0 cars minus the region-hidden ones.
    CHECK(gt.arcade_courses("SINGLE_RACE").size() == 6);
    CHECK(gt.arcade_course_open("trialmt") && !gt.arcade_course_open("akasaka"));
    CHECK(open_cars(gt, "class_c") == 10 && open_cars(gt, "class_b") == 10 && open_cars(gt, "class_a") == 10 &&
          open_cars(gt, "class_s") == 11 && open_cars(gt, "class_dirt") == 9);
    bool vx = false;
    for (const GtData::ArcadeCar* c : gt.arcade_cars("class_b")) vx = vx || c->label == "vauxhall_vx220_00_147";
    CHECK(!vx);
    // The region test: Vauxhall labels have no car record; base-car region bytes 4 (not in
    // the US: the European Oreca Viper) and 2 (not in Europe: the Opels).
    CHECK(gt.arcade_car_region_hidden("vauxhall_vx220_00_147") && gt.arcade_car_region_hidden("ar_viper_gtsr_oreca_00"));
    CHECK(!gt.arcade_car_region_hidden("opel_speedster_00_147") && !gt.arcade_car_region_hidden("viper_gtsr_oreca_00_597"));
    // Time trial defaults (race record +0x1C4..): AT01 1'25.553 / 1'26.952 / 1'27.355.
    std::vector<double> t = gt.time_trial_defaults("AT010001");
    CHECK(t.size() == 3 && std::fabs(t[0] - 85.553) < 1e-6 && std::fabs(t[2] - 87.355) < 1e-6);
    CHECK(GtMode::time_trial_index("AT100001") == 9 && GtMode::time_trial_index("tt02") == 1);
    // The seeded records are named by the program's built-in name table (read from the
    // disc's CORE.GT3, three names per challenge): challenge i's marks 0xFF - 3 i - k show
    // entries 3 i + k. A new time goes between them.
    CHECK(data->builtin_names.size() == 30);
    for (const std::string& n : data->builtin_names) CHECK(n.size() >= 5 && n[1] == '.');
    CHECK(gt.arcade_time_trial("AT010001", 86.0, "nsx_s_zero_97") == 1);
    const std::vector<RankEntry>& rk = gt.state().rankings.at("at:AT010001");
    CHECK(rk.size() == 4 && rk[1].car == "nsx_s_zero_97" && rk[0].car.empty());
    if (rk.size() == 4 && data->builtin_names.size() == 30) {
        CHECK(gt.record_name(rk[0].name) == data->builtin_names[0]);
        CHECK(gt.record_name(rk[2].name) == data->builtin_names[1]);
        CHECK(gt.record_name(rk[3].name) == data->builtin_names[2]);
    }
    CHECK(gt.arcade_time_trial("AT100001", 400.0, "nsx_s_zero_97") == 3);
    const std::vector<RankEntry>& rk10 = gt.state().rankings.at("at:AT100001");
    CHECK(rk10.size() == 4 && rk10[2].name == std::string(1, '\xE2'));
    if (rk10.size() == 4 && data->builtin_names.size() == 30) {
        CHECK(gt.record_name(rk10[0].name) == data->builtin_names[27]);
        CHECK(gt.record_name(rk10[2].name) == data->builtin_names[29]);
    }
    // A byte past the table (no seed has it) shows "-".
    CHECK(gt.record_name(std::string(1, '\x80')) == "-");
    // Without the program's table the seeds show "-"; entered names pass through.
    {
        GtMode g2(std::make_shared<GtData>()); // nothing loaded: no name table
        CHECK(g2.record_name(GtMode::time_trial_seed_name(0, 0)) == "-" && g2.record_name("ABC") == "ABC");
    }
}

TEST(gtmode_disc_arcade_unlocks) {
    NEED_DATA();
    GtMode gt(data);
    // A win records the place, a trophy and the difficulty's award bit.
    GtMode::ArcadeOutcome o = gt.arcade_single_race("AECT0001", "super", 2);
    CHECK(o.recorded && o.improved && gt.state().arcade.best[0] == 2 && gt.state().trophies == 0);
    o = gt.arcade_single_race("AECT0001", "super", 0);
    CHECK(o.improved && gt.state().trophies == 1 && gt.state().arcade.awards == 0x1000);
    CHECK(!gt.arcade_single_race("AECT0001", "super", 0).improved && gt.state().trophies == 2);
    CHECK(!gt.arcade_single_race("AECT0001", "akasaka_unknown", 0).recorded);
    // Professional wins do not open courses.
    arcade_win_group(gt, *data, 0, 'P');
    CHECK(gt.state().arcade.course_level == 0 && gt.state().arcade.awards == 0x9000);
    // Group 0 won at Easy: group 1 opens, no car.
    arcade_win_group(gt, *data, 0, 'E');
    CHECK(gt.state().arcade.course_level == 1 && gt.arcade_course_open("akasaka"));
    CHECK(open_cars(gt, "class_c") == 10);
    // Group 0 won at Normal: car level 1 (arm_roadster_93, class C).
    o = {};
    for (const GtData::ArcadeCourse& c : data->arcade_courses.at("SINGLE_RACE"))
        if (c.level == 0) {
            GtMode::ArcadeOutcome x = gt.arcade_single_race("ANBT0001", c.course, 0);
            o.car_levels.insert(o.car_levels.end(), x.car_levels.begin(), x.car_levels.end());
        }
    CHECK(o.car_levels == std::vector<int>{1} && open_cars(gt, "class_c") == 11);
    // Group 1 split between Normal and Hard: neither flag 2 nor flag 8 (as the original).
    const auto& sr = data->arcade_courses.at("SINGLE_RACE");
    for (int i = 6; i < 12; ++i) gt.arcade_single_race(i < 9 ? "ANAT0001" : "AHAT0001", sr[size_t(i)].course, 0);
    CHECK(gt.state().arcade.course_level == 2 && !gt.arcade_car_level_open(2) && !gt.arcade_car_level_open(8));
    // Group 1 all at Hard: flags 2 and 8 (arm_tt_00 in B, the Delta in Dirt).
    arcade_win_group(gt, *data, 1, 'H');
    CHECK(gt.arcade_car_level_open(2) && gt.arcade_car_level_open(8));
    CHECK(open_cars(gt, "class_b") == 11 && open_cars(gt, "class_dirt") == 10);
    // Completion (Clear Status count): group 0 E+N (6 courses x 2) + group 1 hard (6 x 3)
    // + 3 courses of group 1 at Normal (already counted by Hard).
    CHECK(gt.arcade_completion() == 12 + 18);
    CHECK(!gt.arcade_ending_open() && !gt.arcade_long_ending_open());
    // Every course at Normal: all six groups, the six Normal car levels, the endings.
    for (int g = 0; g < 6; ++g) arcade_win_group(gt, *data, g, 'N');
    CHECK(gt.state().arcade.course_level == 6 && gt.arcade_courses("TIME_ATTACK").size() == 36);
    for (int l = 1; l <= 6; ++l) CHECK(gt.arcade_car_level_open(l));
    CHECK(!gt.arcade_car_level_open(7) && gt.arcade_ending_open() && gt.arcade_long_ending_open());
    CHECK(gt.arcade_all_won_at(1) && !gt.arcade_all_won_at(0));
}

TEST(gtmode_disc_arcade_time_trials_and_save) {
    NEED_DATA();
    GtMode gt(data);
    // Slower than the record: rank 1 (between the defaults), not cleared.
    CHECK(gt.arcade_time_trial("AT010001", 86.0, "") == 1);
    CHECK(gt.state().rankings["at:AT010001"].size() == 4 && gt.state().arcade.at_best_rank[0] == 1);
    // A time equal to a record ranks before it (rank = the strictly faster times).
    CHECK(gt.arcade_time_trial("AT010001", 86.952, "") == 2);
    // Faster than the record: rank 0, the challenge is cleared.
    CHECK(gt.arcade_time_trial("AT010001", 80.0, "") == 0 && gt.state().arcade.at_best_rank[0] == 0);
    // All ten cleared: the arcade_compulsory_complete prize car (once).
    size_t cars = gt.state().garage.size();
    for (int i = 2; i <= 10; ++i) {
        char code[16];
        std::snprintf(code, sizeof code, "AT%02d0001", i);
        gt.arcade_time_trial(code, 1.0, "");
    }
    CHECK(gt.state().garage.size() == cars + 1 && gt.state().prizes_given.count("arcade_compulsory_complete"));
    if (gt.state().garage.size() == cars + 1) CHECK(gt.state().garage[0].car == "panoz_esperante_race_98");
    gt.arcade_time_trial("AT020001", 0.5, "");
    CHECK(gt.state().garage.size() == cars + 1);
    // Progress survives both save formats.
    arcade_win_group(gt, *data, 0, 'H');
    gt.arcade_single_race("APST0001", "rome", 3);
    const ArcadeProgress& a = gt.state().arcade;
    CHECK(a.course_level == 1 && a.car_flags[0] && a.car_flags[6]);
    GameState t = load_from_text(save_to_text(gt.state()));
    CHECK(t.arcade.course_level == a.course_level && t.arcade.car_flags == a.car_flags &&
          t.arcade.awards == a.awards && t.arcade.best == a.best && t.arcade.at_best_rank == a.at_best_rank);
    GameState p = import_ps2_save(export_ps2_save(gt.state(), *data), *data);
    CHECK(p.arcade.course_level == a.course_level && p.arcade.car_flags == a.car_flags &&
          p.arcade.awards == a.awards && p.arcade.best == a.best && p.arcade.at_best_rank == a.at_best_rank);
}

TEST(gtmode_disc_trophy_pick) {
    NEED_DATA();
    GtMode gt(data);
    int count[10] = {};
    for (int i = 0; i < 4000; ++i) {
        int t = gt.pick_trophy();
        CHECK(t >= 0 && t < 10);
        if (t >= 0 && t < 10) ++count[t];
    }
    // Weights 400 350 250 450 2 160 1 500 25 120 (completion 0): champ_07 most often,
    // champ_04 / champ_06 rarely.
    CHECK(count[7] > count[0] && count[3] > count[1] && count[4] < 20 && count[6] < 15);
    CHECK((gt.state().trophy_flags & 0x40u << 7) != 0);
    GameState t = load_from_text(save_to_text(gt.state()));
    CHECK(t.trophy_flags == gt.state().trophy_flags);
    GameState p = import_ps2_save(export_ps2_save(gt.state(), *data), *data);
    CHECK(p.trophy_flags == gt.state().trophy_flags);
}

// --- Display figures as the original's menus show them ------------------------------

TEST(gtmode_disc_completion_texts) {
    NEED_DATA();
    GtMode gt(data);
    CHECK(gt.completion_text() == "0.0 %");
    // 11 of 524 (2.0992 %): the status page and the file lists truncate, "2.0", never "2.1".
    for (int i = 0; i < 11; ++i) gt.state().license_tests[size_t(i)].medal = Medal::Gold;
    CHECK(std::fabs(gt.completion() - 11.0f / 524.0f) < 1e-6f);
    CHECK(gt.completion_text() == "2.0 %");
    CHECK(GtMode::completion_file_text(gt.completion_percent()) == "2.0%");
    float pc;
    std::memcpy(&pc, &ps2_thumbnail(gt.state(), data)[0x1C], 4);
    CHECK(pc == gt.completion_percent());
}

TEST(gtmode_disc_garage_power) {
    NEED_DATA();
    GtMode gt(data);
    gt.state().credits = 1000000;
    CHECK(gt.buy_car("r34_gtr_v_99") == 0);
    const GarageCar& c = *gt.state().ride_car();
    // A new car runs in at 97 %: the figure is the computed peak of its built engine, not the catalogue's.
    PowerPeak stock = gt.power_peak(c);
    std::optional<sim::CarSpec> full = sim::build_car_spec(gt.db(), c.car);
    CHECK(full.has_value());
    if (!full) return;
    CHECK(std::fabs(stock.ps - 0.97f * spec_power_ps(*full)) < 0.05f);
    CHECK(stock.rpm >= 5000 && stock.rpm <= 8000 && stock.rpm % 100 == 0);
    // With a turbo kit the garage shows the tuned figure ("NNNHP/NNNNrpm" on NTSC-U).
    CHECK(gt.buy_part(81) && gt.fit_part(81));
    PowerPeak tuned = gt.power_peak(*gt.state().ride_car());
    CHECK(tuned.ps > stock.ps * 1.2f);
    CHECK(gt.power_text(tuned.ps) == std::to_string(display_power(tuned.ps, data->imperial)) + (data->imperial ? "HP" : "PS"));
    CHECK(gt.sort_value(*gt.state().ride_car(), 6) == gt.power_text(tuned.ps, " "));
}

TEST(gtmode_disc_ranking_top) {
    NEED_DATA();
    GtMode gt(data);
    // An arcade time trial without records: its three seeded entries (the quick menu's Top 3).
    std::vector<RankEntry> at = gt.ranking_top("at:AT010001", 3);
    CHECK(at.size() == 3);
    if (at.size() == 3) {
        CHECK(at[0].value <= at[1].value && at[1].value <= at[2].value);
        CHECK(gt.ranking_name(at[0]) == gt.record_name(GtMode::time_trial_seed_name(0, 0)));
    }
    // Machine test: the records so far, best first.
    CHECK(gt.ranking_top("mt:M400", 3).empty());
    gt.state().player_name = "ME";
    gt.add_record("mt:M400", 13.5, "r34_gtr_v_99", true, 12);
    gt.add_record("mt:M400", 12.9, "r34_gtr_v_99", true, 12);
    gt.add_record("mt:M400", 14.1, "r34_gtr_v_99", true, 12);
    gt.add_record("mt:M400", 15.0, "r34_gtr_v_99", true, 12);
    std::vector<RankEntry> mt = gt.ranking_top("mt:M400", 3);
    CHECK(mt.size() == 3 && mt[0].value == 12.9 && mt[2].value == 14.1 && gt.ranking_name(mt[0]) == "ME");
}

TEST(gtmode_disc_attract_demo_field) {
    NEED_DATA();
    // Every demo of the disc has no car of its own (gcf section 12 word +8 = ""), so the
    // pool draw fills all of its slots (the race screen's attract demo).
    CHECK(!data->demos.empty());
    for (const GtData::Demo& d : data->demos) {
        CHECK(d.car.empty());
        const data::Event* ev = data->db.event(d.event);
        CHECK(ev != nullptr);
        if (!ev) continue;
        GameRandom rng;
        std::vector<std::string> field = race::draw_opponents(data->db, *ev, std::max(int(ev->cars_in_race), 1), rng);
        CHECK(int(field.size()) == std::max(int(ev->cars_in_race), 1));
        for (const std::string& f : field) CHECK(std::find(ev->entries.begin(), ev->entries.end(), f) != ev->entries.end());
    }
}
