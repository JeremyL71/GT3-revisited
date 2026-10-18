#include "audio/bgm.h"

#include "audio/audio_engine.h"
#include "base/log.h"
#include "vfs/vfs.h"

namespace gt3r::bgm {

namespace {

// The game's id -> music.inf name table (filled at init by name).
constexpr const char* kSeqNames[0x12] = {"arcade01", "battle2p01", "dealer01", "free01",   "gorace01", "gtmenu01",
                                         "home01",   "license01",  "license02", "machtest01", "main01", "mini01",
                                         "option01", "theater01",  "timetrial01", "tune01", "wash01", "wheelshop01"};

} // namespace

const char* sequence_name(int id) { return is_sequence(id) ? kSeqNames[id] : nullptr; }
bool is_sequence(int id) { return id >= 0 && id < 0x12; }
bool is_stream(int id) { return id >= 0x12 && id <= 0x22; }

bool Player::load(Vfs& vfs) {
    if (loaded_) return !mads_.groups.empty();
    loaded_ = true;
    try {
        if (auto inf = vfs.read("bgm/ads.inf")) mads_ = sound::MadsIndex::parse(*inf);
    } catch (const std::exception& e) {
        LOG_ERROR("bgm: %s", e.what());
    }
    return !mads_.groups.empty();
}

std::string Player::identity(AudioEngine& audio, Vfs& vfs, int id, int index) {
    if (is_sequence(id)) {
        for (const sound::MusicEntry& e : audio.music_index(vfs))
            if (e.name == kSeqNames[id]) return "seq:" + std::to_string(e.sequence);
        return std::string("seq:") + kSeqNames[id];
    }
    if (!is_stream(id) || !load(vfs)) return {};
    const auto& g = mads_.groups[size_t(id - 0x12)];
    int i = index & 0xFFF;
    if (index < 0 || i >= int(g.size())) return {}; // random pick: never "the same"
    return std::string((index & kIntro) ? "bgm/i_" : "bgm/") + g[size_t(i)].file;
}

bool Player::play(AudioEngine& audio, Vfs& vfs, int id, int index, bool loop) {
    if (is_sequence(id)) {
        if (!audio.play_sequence(vfs, std::string(kSeqNames[id]))) return false;
        id_ = id;
        index_ = index;
        what_ = identity(audio, vfs, id, index);
        return true;
    }
    if (!is_stream(id) || !load(vfs)) {
        stop(audio);
        return false;
    }
    const auto& g = mads_.groups[size_t(id - 0x12)];
    if (g.empty()) return false;
    int i = index & 0xFFF;
    if (index < 0 || i >= int(g.size())) i = int(std::uniform_int_distribution<size_t>(0, g.size() - 1)(rng_));
    std::string path = std::string((index >= 0 && (index & kIntro)) ? "bgm/i_" : "bgm/") + g[size_t(i)].file;
    if (!vfs.exists(path)) {
        // A missing stream: the game plays sound effect 4 ("disable") instead.
        LOG_WARN("bgm: %s not found", path.c_str());
        audio.stop_music();
        id_ = -1;
        return false;
    }
    if (!audio.play_music(vfs, path, loop)) return false;
    id_ = id;
    index_ = index;
    what_ = path;
    return true;
}

bool Player::play_if_different(AudioEngine& audio, Vfs& vfs, int id, int index, bool loop) {
    // The game restarts for a different index, a random or intro request,
    // or a different piece (sequence / stream file).
    bool same = id_ >= 0 && index == index_ && index >= 0 && !(index & kIntro) &&
                is_sequence(id) == is_sequence(id_) && audio.music_playing() && identity(audio, vfs, id, index) == what_;
    if (same) return true;
    return play(audio, vfs, id, index, loop);
}

void Player::stop(AudioEngine& audio) {
    audio.stop_music();
    id_ = index_ = -1;
    what_.clear();
}

bool Player::playing(const AudioEngine& audio) const { return id_ >= 0 && audio.music_playing(); }

} // namespace gt3r::bgm
