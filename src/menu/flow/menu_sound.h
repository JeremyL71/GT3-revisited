#pragma once

// Menu sound effects: the named effects of sound/se.inf (bank 0,
// sound/gt3sys.ins: "cursor", "ok", "cancel", "disable", ...) played through
// the SPU2-style voice mixer the way the game's menu SE call does (volume
// 1.0, pan 0x40; see docs/formats/sound.md "GTSE"):
//   - an effect with one region is keyed twice, hard left and hard right
//     (pan 0 and 0x7F), so it sounds centred at the region's full level;
//   - an effect with a region pair (e.g. "next" 2/3, "return" 7/8) plays only
//     its first region, centred with the sine pan law;
//   - earlier instances are not stopped (sounds may overlap);
//   - nothing plays while the "menu SE" option is off (set_enabled).

#include "formats/sound/inst.h"

#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace gt3r {
class Vfs;
class VoiceMixer;
} // namespace gt3r

namespace gt3r::menu::flow {

class MenuSounds {
public:
    // Loads se.inf and the banks it refers to. Returns false (and plays
    // nothing later) when the index is missing.
    bool load(const Vfs& vfs);
    bool loaded() const { return !effects_.empty(); }
    bool has(std::string_view name) const { return effects_.count(std::string(name)) != 0; }

    // Plays a named effect; unknown names are ignored.
    void play(VoiceMixer& mixer, std::string_view name);

    // The options' menu sound-effect switch (on by default).
    void set_enabled(bool on) { enabled_ = on; }
    bool enabled() const { return enabled_; }

private:
    std::map<std::string, sound::SoundEffect> effects_;
    std::map<int, std::shared_ptr<sound::InstBank>> banks_;
    bool enabled_ = true;
};

} // namespace gt3r::menu::flow
