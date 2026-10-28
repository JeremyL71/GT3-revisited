// Event prize money checked against known retail payouts (NTSC-U), through the GT Mode rules.

#include "test_framework.h"

#include "game/gt_mode.h"
#include "vfs/vfs.h"

#include <memory>

using namespace gt3r;

TEST(event_prize_money) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    auto data = std::make_shared<game::GtData>();
    CHECK(data->load([&](const std::string& p) { return vfs.read(p); }));
    game::GtMode gt(data);
    // Sunday Cup pays 1,000 Cr for 1st in the international releases (stored 10 .. 5 hundreds).
    for (const char* label : {"Gsunday_e_0001", "Gsunday_e_0002", "Gsunday_e_0003"}) {
        std::optional<game::EventRules> r = gt.rules(label);
        CHECK(r && r->prize[0] == 1000 && r->prize[5] == 500);
    }
}
