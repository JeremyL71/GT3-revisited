#pragma once

// What the game's language decides in the GT Mode and licence rules, beyond the data set and
// the menus (disc/region.h): prize-car labels, the Opel / Vauxhall dealers, the licence test
// car and its demonstration replay, and which name a paint colour shows. Car names follow the
// language in the database itself (data::car_name_fields). The language index is the game's
// (JP 0, US 1, BR 2 British English, FR 3, GR 4, IT 5, SP 6).

#include "base/bytes.h"
#include "disc/region.h"

#include <functional>
#include <string>
#include <string_view>

namespace gt3r::data {
class CarDatabase;
struct Event;
} // namespace gt3r::data

namespace gt3r::game {

// A prize set's candidate car (gcf tag 15; gt_mode §23) in the language (docs/dev/GT_MODE.md
// "Language"): the label is looked up in a fixed table, in order, the first entry whose language
// mask has the language's bit (1 << index) and whose label matches replacing it:
//   opel_astra_race_00 -> vauxhall_astra_race_00, opel_speedster_00 -> vauxhall_vx220_00,
//   opel_calibra_race_95 -> vauxhall_calibra_race_95                    (mask 0x04: BR)
//   viper_gtsr_oreca_00 -> dodge_viper_gtsr_oreca_00, s2000_99 -> s2000_a_99,
//   s2000_99_ext -> s2000_a_99_ext                                        (mask 0x02: US)
//   s2000_99 -> s2000_e_99, s2000_99_ext -> s2000_e_99_ext                (mask 0x7C: BR..SP)
// Other labels, and every label in Japanese, stay. `swapped` tells whether it changed: a
// swapped candidate has no fixed colour (a random paint, gt_mode §20.3).
std::string prize_car_label(std::string_view label, Language lang, bool* swapped = nullptr);

// Whether a GT Mode dealer region page (cd/) lists an item (gt_mode §7; docs/dev/GT_MODE.md
// "Language"): in British English its list leaves out the item "opel", in every other language
// the item "vauxhall"; the buttons after it close up.
bool dealer_item_listed(std::string_view item, Language lang);

// The car a licence test is driven in (gt_mode §21, docs/dev/GT_MODE.md "Language"): the event's
// entry 0 in its slot-0 colour (event +0x110; 0xFFFFFFFF = none). In British English, when entry 0's base car is not offered
// in the language (its region byte, data::car_region_offered), entry 1 in its slot-1 colour
// instead; when entry 1 names no car, entry 0 stays, in the slot-1 colour if it has one.
struct LicenseCar {
    std::string entry;     // event entry label (a car variant or a car), "" when the event has none
    u32 color = 0xFFFFFFFFu;
    int slot = 0;          // 0 or 1
};
LicenseCar license_car(const data::CarDatabase& db, const data::Event& ev, Language lang);

// The demonstration replay of a race code, demo/<name> (docs/dev/GT_MODE.md "Language"): the code
// itself, or in British English "L0S0001_uk" for licence test S-1 (its car is the Vauxhall there).
std::string demo_replay_name(std::string_view code, Language lang);
// Its file: demo/<demo_replay_name>, or demo/<code> when the disc has no such file.
std::string demo_replay_path(std::string_view code, Language lang,
                             const std::function<bool(const std::string&)>& exists);

// The carcolor.db colour definition {u32 id, u32 name, u32 second name, u32 rgb}: the offset
// of the name a language shows (gt_mode §7; docs/dev/GT_MODE.md "Language"): the second name (+8,
// kana for some colours) in Japanese, the first (+4) in the other languages.
size_t color_name_field(Language lang);

} // namespace gt3r::game
