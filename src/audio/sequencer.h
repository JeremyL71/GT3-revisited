#pragma once

// Sequenced music player: plays SEQG sequences (music/music.seq) with the
// programs of an INST bank (music/music.ins) on the VoiceMixer, the way the
// game's sequencer does:
//
//   - update() is one game frame. The tick clock is the game's: every
//     update adds 480,000,000 / update_rate to an accumulator that is
//     drained by the tempo word (us per 480 ticks), giving the ticks that
//     elapsed this update. All 16 tracks then run their due events; then
//     every sounding note counts down, follows its channel's pitch bend,
//     volume and pan, and is keyed off when its length has run out.
//   - a note plays one voice per region of the channel's program whose key
//     range holds the key; 47 note slots are shared by all channels.
//   - volume = channel volume x program volume x velocity x region volume
//     (all Q14, linear 0..127 curves), squared, then split with the sine
//     pan law; pan = channel + program + region pan - 0x80.
//
// Timing is sample-exact for offline renders (call update() then mix
// kRate / update_rate frames) and in the app (AudioEngine drives update()
// from VoiceMixer's tick hook). Thread-safe: one internal mutex.

#include "audio/voice_mixer.h"
#include "formats/sound/inst.h"
#include "formats/sound/seq.h"

#include <array>
#include <memory>
#include <mutex>
#include <vector>

namespace gt3r {

class Sequencer {
public:
    static constexpr int kNoteSlots = 47;
    static constexpr int kGameUpdateRate = 60; // the game's sound thread runs at the NTSC field rate

    Sequencer() = default;

    void set_instruments(std::shared_ptr<const sound::InstBank> bank);
    void set_sequences(std::shared_ptr<const sound::SeqBank> seqs);
    bool ready() const;

    // Starts sequence `index` from the top, stopping the current one.
    bool play(VoiceMixer& mixer, int index);
    // Stops the sequence; its notes enter their release phase.
    void stop(VoiceMixer& mixer);
    bool active() const;  // a sequence is playing
    bool running() const; // ... and at least one of its tracks has events left
    int current() const;  // its index, or -1

    // The game's global music volume (Q14, 0..0x3FFF; the options setting).
    void set_master_volume(int q14);
    int master_volume() const;

    // Update rate in Hz (default 60 like the game). Must divide
    // VoiceMixer::kRate for sample-exact renders.
    void set_update_rate(int hz);
    int update_rate() const;
    u32 samples_per_update() const { return VoiceMixer::kRate / u32(update_rate()); }

    // One update (game frame).
    void update(VoiceMixer& mixer);

    // Progress of the current sequence.
    u64 ticks() const;           // ticks played since play()
    u64 updates() const;         // updates since play()
    int loops_completed() const; // passes through the endless loop's end
    int sounding_notes() const;  // occupied note slots
    u32 tempo() const;           // current tempo word

private:
    struct Track {
        size_t pos = 0;      // read position in the file; 0 = finished/unused
        s32 wait = 0;        // ticks until the next event
        u16 vol_curve = 0x4000; // channel volume from the curve (Q14)
        u16 volume = 0;      // channel volume x sequence volume (Q14)
        s16 bend = 0;        // -8192..8191
        u8 pan = 0;
        u8 program = 0;
        int sp = -1;         // loop stack
        std::array<size_t, 4> loop_pos{};
        std::array<u8, 4> loop_count{};
    };
    struct Slot {
        VoiceId voice;
        int track = -1;
        int region = -1;
        u8 program = 0;
        u8 key = 0, velocity = 0;
        s16 bend = 0;
        u16 volume = 0; // channel volume snapshot
        u8 pan = 0;     // channel pan snapshot
        s32 remaining = 0;
    };

    void start_locked(int index);
    void run_track(VoiceMixer& mixer, int t);
    void note_on(VoiceMixer& mixer, int t, u8 key, u8 velocity, s32 length);
    void update_notes(VoiceMixer& mixer);
    void apply_master_locked();
    void release_all(VoiceMixer& mixer);
    u16 note_pitch(const Slot& s, s16 bend) const;
    void note_volume(const Slot& s, const Track& t, s16& l, s16& r) const;

    mutable std::mutex mutex_;
    std::shared_ptr<const sound::InstBank> bank_;
    std::shared_ptr<const sound::SeqBank> seqs_;
    int update_rate_ = kGameUpdateRate;
    int master_ = 0x3FFF;

    int current_ = -1;
    u32 seq_volume_ = 0x4000; // from the sequence record
    u16 volume_ = 0;          // seq_volume_ x master_
    u32 tempo_ = 0;
    s64 acc_ = 0;
    s32 elapsed_ = 0;         // ticks this update
    std::array<Track, sound::kSeqTracks> tracks_{};
    std::array<Slot, kNoteSlots> slots_{};
    int next_slot_ = 0;
    u64 ticks_ = 0, updates_ = 0;
    int loops_ = 0;
    int loop_track_ = -1; // track whose endless loop counts loops_
};

// Offline render of a sequence through a private VoiceMixer, for tools and
// tests: plays until the endless loop has completed `loops` times (or the
// sequence ends, or max_seconds), then stops it and renders `tail_seconds`
// of release and reverb.
struct SequenceRenderOptions {
    int loops = 1;
    double max_seconds = 900;
    double tail_seconds = 3;
    int reverb_mode = SpuReverb::StudioC; // the game's setting
    s16 reverb_depth = 0x0FFF;
    VoiceMixer::Interpolation interpolation = VoiceMixer::Interpolation::Gaussian;
    float master = 0.5f; // VoiceMixer output trim
    int update_rate = Sequencer::kGameUpdateRate;
};

struct SequenceRender {
    std::vector<s16> pcm;            // interleaved stereo, VoiceMixer::kRate
    std::vector<u64> loop_frames;    // output frame of each pass through the loop end
    u64 end_frame = 0;               // frame where the sequence was stopped
    int max_voices = 0;              // most voices sounding at once
    int peak = 0;                    // largest |sample|
    size_t clipped = 0;              // samples at full scale
};

SequenceRender render_sequence(std::shared_ptr<const sound::InstBank> bank,
                               std::shared_ptr<const sound::SeqBank> seqs, int index,
                               const SequenceRenderOptions& opt = {});

} // namespace gt3r
