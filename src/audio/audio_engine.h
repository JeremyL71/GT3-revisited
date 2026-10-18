#pragma once

#include "audio/bgm.h"
#include "audio/sequencer.h"
#include "audio/voice_mixer.h"
#include "formats/ads.h"
#include "formats/sound/seq.h"

#include <SDL3/SDL_audio.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace gt3r {

class Vfs;

// Audio output: streamed music (.ads) plus the SPU2-style voice mixer used
// for engine notes, sound effects and the sequenced menu music. The stream
// and the mixer each run on their own SDL audio stream. The mixer starts in
// the game's boot state: reverb "studio C" with return volume 0x0FFF.
// Background music is either a stream or a sequence; starting one stops the
// other, and the music_* queries cover both.
class AudioEngine {
public:
    AudioEngine();
    ~AudioEngine();

    bool ok() const { return music_stream_ != nullptr; }

    // Streams an .ads file through the VFS (mods may replace it).
    bool play_music(Vfs& vfs, const std::string& path, bool loop);
    // Plays sequence `index` of music/music.seq with music/music.ins (loaded
    // on first use through the VFS). Sequences loop forever.
    bool play_sequence(Vfs& vfs, int index);
    // Same, by music.inf name (e.g. "gtmenu01", the ids the game uses).
    // A music.inf name, or "bgm:<id>[:once]" for a BGM-layer cue (bgm::Player).
    bool play_sequence(Vfs& vfs, const std::string& name);
    // music.inf entries (loads the sequenced music if needed; empty if absent).
    const std::vector<sound::MusicEntry>& music_index(Vfs& vfs);
    bool sequence_playing() const { return sequencer_.active(); }
    Sequencer& sequencer() { return sequencer_; }
    // The game's BGM layer (ids, restart rules). Shared
    // by menus, race and presentations so "play if different" sees one state.
    bgm::Player& bgm() { return bgm_; }

    void stop_music(); // stream and sequence
    // Pauses / resumes the streamed music where it is (the race pause).
    void pause_music();
    void resume_music();
    bool music_playing() const;
    double music_position() const;
    double music_length() const; // stream length; for a sequence, one pass of its loop
    void set_music_volume(float v);
    float music_volume() const { return music_volume_; }

    // Voice mixer for sound effects and engine notes (thread-safe).
    VoiceMixer& voices() { return voices_; }
    void set_sfx_volume(float v);
    float sfx_volume() const { return sfx_volume_; }

private:
    bool load_sequences(Vfs& vfs);
    void stop_stream();

    static void SDLCALL feed(void* user, SDL_AudioStream* stream, int additional, int total);
    static void SDLCALL feed_voices(void* user, SDL_AudioStream* stream, int additional, int total);

    SDL_AudioStream* music_stream_ = nullptr;
    mutable std::mutex mutex_;
    std::unique_ptr<AdsStream> music_;
    std::shared_ptr<Bytes> music_data_; // backing store when not streamed from disc
    double music_length_ = 0.0;
    float music_volume_ = 0.8f;
    bool music_finished_ = false;

    VoiceMixer voices_;
    Sequencer sequencer_;
    bgm::Player bgm_;
    std::vector<sound::MusicEntry> music_index_;
    std::shared_ptr<const sound::SeqBank> seqs_;
    std::vector<double> seq_loop_seconds_;
    bool seq_load_tried_ = false;
    SDL_AudioStream* sfx_stream_ = nullptr;
    float sfx_volume_ = 0.8f;
};

} // namespace gt3r
