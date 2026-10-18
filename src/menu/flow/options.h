#pragma once

// The game options of the title "Options" menu (menu/option.mbl): their values,
// stored in config.ini section [options] as `key = value` (e.g. `menu_bgm = off`,
// `wide_mode = on`), and the option pages' preview art. The option top page's
// layers ("m1".."m11") preview each page with widgets the game fills by class:
//   "<base>_<value>_<on|off>[_1p|_2p]"  drawn "on" when the option has that value,
//                                        "off" otherwise (four widgets per choice pair);
//   "<base>_<on|off>[_1p|_2p]"         drawn when the on/off option is in that state.
// SDL-free.

#include "base/config.h"

#include <string>
#include <vector>

namespace gt3r::menu::flow {

struct GameOption {
    std::string key;                 // [options] key and preview base ("menu_bgm", "assist_1p")
    std::string page, multi;         // option.mbl page and multi item
    std::vector<std::string> values; // the multi item's children in order
    int def = 0;                     // default (index into values)
};

const std::vector<GameOption>& game_options();
const GameOption* find_game_option(const std::string& key);
const GameOption* find_game_option_by_item(const std::string& page, const std::string& multi);

// The option's value (its default when unset or invalid); "" for an unknown key.
std::string option_value(const IniDocument* cfg, const std::string& key);
int option_index(const IniDocument* cfg, const GameOption& o);
void set_option(IniDocument& cfg, const GameOption& o, int index);
bool option_on(const IniDocument* cfg, const std::string& key); // value "on" (on/off options)
// 2P battle laps (1..99, default 2) and handicap (metres, -150..150 in steps of 5,
// default 0; negative = the 1P line): [options] split_laps / split_handicap.
int option_laps(const IniDocument* cfg);
int option_handicap(const IniDocument* cfg);

// Preview widgets: false when a widget of that class must not be drawn for the
// current values; true for classes that are not option previews.
bool option_preview_visible(const IniDocument* cfg, const std::string& widget_class);

} // namespace gt3r::menu::flow
