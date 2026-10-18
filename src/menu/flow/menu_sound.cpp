#include "menu/flow/menu_sound.h"

#include "audio/voice_mixer.h"
#include "base/log.h"
#include "formats/sound/spu.h"
#include "vfs/vfs.h"

#include <algorithm>

namespace gt3r::menu::flow {

bool MenuSounds::load(const Vfs& vfs) {
    effects_.clear();
    banks_.clear();
    auto inf = vfs.read("sound/se.inf");
    if (!inf) return false;
    try {
        for (sound::SoundEffect& e : sound::parse_se_index(*inf)) {
            if (!banks_.count(e.bank)) {
                std::shared_ptr<sound::InstBank> bank;
                if (const char* path = sound::se_bank_path(e.bank))
                    if (auto data = vfs.read(path)) bank = std::make_shared<sound::InstBank>(sound::InstBank::parse(*data));
                banks_[e.bank] = bank;
            }
            if (banks_[e.bank]) effects_[e.name] = e;
        }
    } catch (const std::exception& e) {
        LOG_WARN("menu sounds: %s", e.what());
        effects_.clear();
    }
    return loaded();
}

void MenuSounds::play(VoiceMixer& mixer, std::string_view name) {
    if (!enabled_) return;
    auto it = effects_.find(std::string(name));
    if (it == effects_.end()) return;
    const sound::SoundEffect& e = it->second;
    const sound::InstBank& b = *banks_.at(e.bank);
    if (e.first >= b.regions.size()) return;
    const sound::InstRegion& reg = b.regions[e.first];
    auto key = [&](int pan) {
        int l, r;
        sound::pan_gains(pan, l, r);
        s32 v = std::min<s32>(reg.volume, 0x3FFF); // volume 1.0 x region gain, clamped like the game
        VoiceParams p{b.adpcm, reg.address, reg.pitch(), s16((v * l) >> 14), s16((v * r) >> 14),
                      reg.adsr1, reg.adsr2, reg.reverb(), reg.priority};
        mixer.key_on(p);
    };
    if (e.first == e.last) {
        key(0);
        key(0x7F);
    } else {
        key(0x40);
    }
}

} // namespace gt3r::menu::flow
