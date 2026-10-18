#include "audio/sequencer.h"

#include "base/log.h"

#include <algorithm>
#include <cmath>

namespace gt3r {

namespace {

// The game's 0..127 -> Q14 curve, used for channel volume and velocity.
u16 level_curve(u32 i) { return u16(std::lround(std::min<u32>(i, 127) * 16384.0 / 127.0)); }

// The tick accumulator gains this much per update at `rate` Hz: 480 ticks
// per tempo unit (us per beat) times 1,000,000 us, per update.
s64 clock_step(int rate) { return 480000000 / rate; }

constexpr int kMaxEventsPerUpdate = 1 << 16; // guards against zero-delta endless loops

} // namespace

void Sequencer::set_instruments(std::shared_ptr<const sound::InstBank> bank) {
    std::lock_guard lock(mutex_);
    bank_ = std::move(bank);
}

void Sequencer::set_sequences(std::shared_ptr<const sound::SeqBank> seqs) {
    std::lock_guard lock(mutex_);
    seqs_ = std::move(seqs);
}

bool Sequencer::ready() const {
    std::lock_guard lock(mutex_);
    return bank_ && seqs_;
}

bool Sequencer::play(VoiceMixer& mixer, int index) {
    std::lock_guard lock(mutex_);
    if (!bank_ || !seqs_ || index < 0 || index >= int(seqs_->sequences.size())) return false;
    release_all(mixer);
    start_locked(index);
    return true;
}

void Sequencer::stop(VoiceMixer& mixer) {
    std::lock_guard lock(mutex_);
    release_all(mixer);
    current_ = -1;
    tracks_ = {};
}

bool Sequencer::active() const {
    std::lock_guard lock(mutex_);
    return current_ >= 0;
}

bool Sequencer::running() const {
    std::lock_guard lock(mutex_);
    return current_ >= 0 && std::any_of(tracks_.begin(), tracks_.end(), [](const Track& t) { return t.pos != 0; });
}

int Sequencer::current() const {
    std::lock_guard lock(mutex_);
    return current_;
}

void Sequencer::set_master_volume(int q14) {
    std::lock_guard lock(mutex_);
    master_ = std::clamp(q14, 0, 0x3FFF);
    apply_master_locked();
}

int Sequencer::master_volume() const {
    std::lock_guard lock(mutex_);
    return master_;
}

void Sequencer::set_update_rate(int hz) {
    std::lock_guard lock(mutex_);
    update_rate_ = std::clamp(hz, 1, int(VoiceMixer::kRate));
}

int Sequencer::update_rate() const {
    std::lock_guard lock(mutex_);
    return update_rate_;
}

u64 Sequencer::ticks() const {
    std::lock_guard lock(mutex_);
    return ticks_;
}

u64 Sequencer::updates() const {
    std::lock_guard lock(mutex_);
    return updates_;
}

int Sequencer::loops_completed() const {
    std::lock_guard lock(mutex_);
    return loops_;
}

u32 Sequencer::tempo() const {
    std::lock_guard lock(mutex_);
    return tempo_;
}

int Sequencer::sounding_notes() const {
    std::lock_guard lock(mutex_);
    return int(std::count_if(slots_.begin(), slots_.end(), [](const Slot& s) { return s.voice.valid(); }));
}

void Sequencer::start_locked(int index) {
    const sound::Sequence& s = seqs_->sequences[size_t(index)];
    const ByteSpan d = *seqs_->data;
    current_ = index;
    seq_volume_ = s.volume;
    volume_ = u16((u32(master_) * seq_volume_) >> 14);
    tempo_ = std::max<u32>(1, s.tempo);
    acc_ = clock_step(update_rate_);
    elapsed_ = 0;
    ticks_ = updates_ = 0;
    loops_ = 0;
    loop_track_ = -1;
    next_slot_ = 0;
    for (int t = 0; t < sound::kSeqTracks; ++t) {
        Track& tr = tracks_[size_t(t)];
        tr = Track{};
        tr.volume = volume_;
        if (!s.tracks[size_t(t)]) continue;
        try {
            size_t p = s.tracks[size_t(t)];
            tr.wait = s32(sound::read_seq_number(d, p));
            tr.pos = p;
        } catch (const FormatError& e) {
            LOG_ERROR("sequence %d track %d: %s", index, t, e.what());
        }
    }
}

void Sequencer::apply_master_locked() {
    volume_ = u16((u32(master_) * seq_volume_) >> 14);
    for (Track& t : tracks_) t.volume = u16((u32(volume_) * t.vol_curve) >> 14);
}

void Sequencer::release_all(VoiceMixer& mixer) {
    for (Slot& s : slots_) {
        if (s.voice.valid()) mixer.key_off(s.voice);
        s = Slot{};
    }
}

void Sequencer::update(VoiceMixer& mixer) {
    std::lock_guard lock(mutex_);
    if (current_ < 0) return;
    ++updates_;
    // Ticks this update: how many tempo units fit before the accumulator
    // runs out (at least one while it is positive), then refill it.
    elapsed_ = 0;
    if (acc_ > 0) {
        s64 n = (acc_ + tempo_ - 1) / tempo_;
        acc_ -= n * tempo_;
        elapsed_ = s32(n);
    }
    acc_ += clock_step(update_rate_);
    ticks_ += u64(elapsed_);
    for (int t = 0; t < sound::kSeqTracks; ++t) run_track(mixer, t);
    update_notes(mixer);
}

void Sequencer::run_track(VoiceMixer& mixer, int t) {
    Track& tr = tracks_[size_t(t)];
    if (!tr.pos) return;
    tr.wait -= elapsed_;
    if (tr.wait > 0) return;
    const ByteSpan d = *seqs_->data;
    try {
        for (int guard = 0; guard < kMaxEventsPerUpdate; ++guard) {
            size_t p = tr.pos;
            sound::SeqEvent e = sound::read_seq_event(d, p);
            switch (e.op) {
            case sound::SeqOp::Note:
                // The game hands the note its length plus this update's
                // ticks plus the (non-positive) overshoot of the event.
                note_on(mixer, t, e.key, e.velocity, s32(e.length) + elapsed_ + tr.wait);
                break;
            case sound::SeqOp::PitchBend: tr.bend = s16(e.bend); break;
            case sound::SeqOp::LoopStart:
                if (tr.sp < 3) {
                    ++tr.sp;
                    tr.loop_pos[size_t(tr.sp)] = p;
                    tr.loop_count[size_t(tr.sp)] = e.value;
                }
                break;
            case sound::SeqOp::LoopEnd:
                if (tr.sp >= 0) {
                    u8& c = tr.loop_count[size_t(tr.sp)];
                    if (c == 0) {
                        --tr.sp;
                    } else {
                        if (c != sound::kSeqLoopEndless) --c;
                        else {
                            if (loop_track_ < 0) loop_track_ = t;
                            if (loop_track_ == t) ++loops_;
                        }
                        p = tr.loop_pos[size_t(tr.sp)];
                    }
                }
                break;
            case sound::SeqOp::Program: tr.program = e.value; break;
            case sound::SeqOp::Volume:
                tr.vol_curve = level_curve(e.value);
                tr.volume = u16((u32(volume_) * tr.vol_curve) >> 14);
                break;
            case sound::SeqOp::Pan: tr.pan = e.value; break;
            case sound::SeqOp::Tempo: tempo_ = std::max<u32>(1, e.tempo); break;
            case sound::SeqOp::Nop: break;
            case sound::SeqOp::End: tr.pos = 0; return;
            }
            tr.wait += s32(sound::read_seq_number(d, p));
            tr.pos = p;
            if (tr.wait > 0) return;
        }
        LOG_ERROR("sequence %d track %d: no time passes in its loop; stopped", current_, t);
    } catch (const FormatError& e) {
        LOG_ERROR("sequence %d track %d: %s", current_, t, e.what());
    }
    tr.pos = 0;
}

u16 Sequencer::note_pitch(const Slot& s, s16 bend) const {
    const sound::InstRegion& r = bank_->regions[size_t(s.region)];
    int note = s.key << 8;
    if (bend) note += (bend * (bend < 0 ? r.bend_down : r.bend_up)) >> 5;
    return sound::note_to_pitch(note, r.center);
}

void Sequencer::note_volume(const Slot& s, const Track& t, s16& l, s16& r) const {
    const sound::InstProgram& prog = bank_->programs[s.program];
    const sound::InstRegion& reg = bank_->regions[size_t(s.region)];
    s32 v = (s32(t.volume) * prog.volume) >> 14;
    v = (v * level_curve(s.velocity)) >> 14;
    v = (v * reg.volume) >> 14;
    v = (v * v) >> 14;
    int pan = std::clamp(int(t.pan) + prog.pan + reg.pan - 0x80, 0, 127);
    int gl, gr;
    sound::pan_gains(pan, gl, gr);
    l = s16(std::clamp((v * gl) >> 14, 0, 0x3FFF));
    r = s16(std::clamp((v * gr) >> 14, 0, 0x3FFF));
}

void Sequencer::note_on(VoiceMixer& mixer, int t, u8 key, u8 velocity, s32 length) {
    const Track& tr = tracks_[size_t(t)];
    if (!bank_ || tr.program >= bank_->programs.size()) return;
    const sound::InstProgram& prog = bank_->programs[tr.program];
    for (u8 ri : prog.regions) {
        const sound::InstRegion& r = bank_->regions[ri];
        if (key < r.key_lo || key > r.key_hi) continue;
        // Next free slot after the last one used (a slot frees when its
        // voice has finished or was taken by another note).
        int idx = -1;
        for (int k = 1; k <= kNoteSlots && idx < 0; ++k) {
            int i = (next_slot_ + k) % kNoteSlots;
            Slot& s = slots_[size_t(i)];
            if (s.voice.valid() && !mixer.playing(s.voice)) s = Slot{};
            if (!s.voice.valid()) idx = i;
        }
        if (idx < 0) continue;
        next_slot_ = idx;
        Slot& s = slots_[size_t(idx)];
        s = Slot{};
        s.track = t;
        s.region = ri;
        s.program = tr.program;
        s.key = key;
        s.velocity = velocity;
        s.bend = tr.bend;
        s.volume = tr.volume;
        s.pan = tr.pan;
        s.remaining = length + elapsed_;
        VoiceParams p;
        p.data = bank_->adpcm;
        p.start = r.address;
        p.pitch = note_pitch(s, s.bend);
        note_volume(s, tr, p.vol_l, p.vol_r);
        p.adsr1 = r.adsr1;
        p.adsr2 = r.adsr2;
        p.reverb = r.reverb();
        p.priority = r.priority;
        s.voice = mixer.key_on(p);
    }
}

void Sequencer::update_notes(VoiceMixer& mixer) {
    for (Slot& s : slots_) {
        if (!s.voice.valid()) continue;
        if (!mixer.playing(s.voice)) {
            s = Slot{};
            continue;
        }
        const Track& tr = tracks_[size_t(s.track)];
        if (s.remaining > 0) {
            s.remaining -= elapsed_;
            if (s.remaining <= 0) mixer.key_off(s.voice);
        }
        if (tr.bend != s.bend) {
            mixer.set_pitch(s.voice, note_pitch(s, tr.bend));
            s.bend = tr.bend;
        }
        if (tr.volume != s.volume || tr.pan != s.pan) {
            s16 l, r;
            note_volume(s, tr, l, r);
            mixer.set_volume(s.voice, l, r);
            s.volume = tr.volume;
            s.pan = tr.pan;
        }
    }
}

SequenceRender render_sequence(std::shared_ptr<const sound::InstBank> bank,
                               std::shared_ptr<const sound::SeqBank> seqs, int index,
                               const SequenceRenderOptions& opt) {
    SequenceRender out;
    VoiceMixer mixer;
    mixer.set_master_volume(opt.master);
    mixer.set_interpolation(opt.interpolation);
    mixer.set_reverb(opt.reverb_mode, opt.reverb_depth, opt.reverb_depth);
    Sequencer seq;
    seq.set_update_rate(opt.update_rate);
    seq.set_instruments(std::move(bank));
    seq.set_sequences(std::move(seqs));
    if (!seq.play(mixer, index)) return out;
    const u32 spu = seq.samples_per_update();
    const u64 max_updates = u64(opt.max_seconds * opt.update_rate);
    std::vector<s16> buf(size_t(spu) * 2);
    int loops = 0;
    for (u64 u = 0; u < max_updates; ++u) {
        seq.update(mixer);
        u64 frame = out.pcm.size() / 2;
        while (loops < seq.loops_completed()) {
            out.loop_frames.push_back(frame);
            ++loops;
        }
        out.max_voices = std::max(out.max_voices, mixer.active_voices());
        if (loops >= opt.loops || (!seq.running() && seq.sounding_notes() == 0)) break;
        mixer.mix(buf.data(), spu);
        out.pcm.insert(out.pcm.end(), buf.begin(), buf.end());
    }
    out.end_frame = out.pcm.size() / 2;
    seq.stop(mixer);
    const u64 tail = u64(opt.tail_seconds * opt.update_rate);
    for (u64 u = 0; u < tail; ++u) {
        mixer.mix(buf.data(), spu);
        out.pcm.insert(out.pcm.end(), buf.begin(), buf.end());
    }
    for (s16 v : out.pcm) {
        int a = std::abs(int(v));
        out.peak = std::max(out.peak, a);
        out.clipped += a >= 32767;
    }
    return out;
}

} // namespace gt3r
