// Event prize money checked against known retail payouts (NTSC-U).

#include "test_framework.h"

#include "game/data/car_database.h"

using namespace gt3r;

TEST(event_prize_money) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    data::CarDatabase db;
    CHECK(db.load(disc->volume()));
    // Sunday Cup pays 1,000 Cr for 1st in the international releases.
    for (const char* label : {"Gsunday_e_0001", "Gsunday_e_0002", "Gsunday_e_0003"}) {
        const data::Event* e = db.event(label);
        CHECK(e != nullptr);
        if (e) CHECK(e->prize_credits(0) == 1000);
    }
}
