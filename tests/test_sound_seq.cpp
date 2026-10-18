// Sequenced music: SEQG/MSEQ decoding, the sequencer's timing, the SPU2
// envelope and reverb models, and (with GT3R_DISC) every sequence on the disc.

#include "test_framework.h"

#include "audio/sequencer.h"
#include "audio/spu_reverb.h"
#include "audio/voice_mixer.h"
#include "formats/sound/inst.h"
#include "formats/sound/seq.h"

#include <cmath>
#include <cstdio>
#include <cstring>

using namespace gt3r;
using namespace gt3r::sound;

namespace {

void put32(Bytes& b, size_t o, u32 v) {
    if (b.size() < o + 4) b.resize(o + 4);
    for (int i = 0; i < 4; ++i) b[o + i] = u8(v >> (8 * i));
}

void put_number(Bytes& b, u32 v) {
    u8 tmp[5];
    int n = 0;
    do {
        tmp[n++] = u8(v & 0x7F);
        v >>= 7;
    } while (v);
    while (n--) b.push_back(u8(tmp[n] | (n ? 0x80 : 0)));
}

// A SEQG file with one sequence whose track 0 is `track` (tracks 1..15 empty).
Bytes make_seqg(const Bytes& track, u32 tempo, u32 volume = 0x4000) {
    Bytes b(0x0C + 0x48);
    std::memcpy(b.data(), "SEQG", 4);
    put32(b, 8, 1);
    put32(b, 0x0C, volume);
    put32(b, 0x10, tempo);
    put32(b, 0x14, u32(b.size()));
    b.insert(b.end(), track.begin(), track.end());
    return b;
}

// One looping constant-level sample (value 4096) and one program/region
// covering every key, centred at key 60.
std::shared_ptr<InstBank> make_bank(u16 adsr1 = 0x000F, u16 adsr2 = 0x0000) {
    auto bank = std::make_shared<InstBank>();
    Bytes adpcm;
    for (int f = 0; f < 4; ++f) {
        adpcm.push_back(0x00);
        adpcm.push_back(f == 0 ? 0x06 : f == 3 ? 0x03 : 0x02);
        for (int i = 0; i < 14; ++i) adpcm.push_back(0x11);
    }
    bank->adpcm = std::make_shared<const Bytes>(adpcm);
    InstRegion r;
    r.volume = 0x4000;
    r.center = 60 << 8;
    r.pan = 0x40;
    r.key_lo = 0;
    r.key_hi = 127;
    r.bend_down = r.bend_up = 2;
    r.adsr1 = adsr1;
    r.adsr2 = adsr2;
    bank->regions.push_back(r);
    InstProgram p;
    p.pan = 0x40;
    p.volume = 0x4000;
    p.regions = {0};
    bank->programs.push_back(p);
    return bank;
}

// Renders `updates` 60 Hz updates and returns the left channel.
std::vector<s16> run(Sequencer& seq, VoiceMixer& mixer, int updates) {
    std::vector<s16> left, buf(800 * 2);
    for (int u = 0; u < updates; ++u) {
        seq.update(mixer);
        mixer.mix(buf.data(), 800);
        for (int i = 0; i < 800; ++i) left.push_back(buf[2 * i]);
    }
    return left;
}

int first_loud(const std::vector<s16>& v, size_t from = 0) {
    for (size_t i = from; i < v.size(); ++i)
        if (std::abs(v[i]) > 200) return int(i);
    return -1;
}

int first_quiet(const std::vector<s16>& v, size_t from) {
    for (size_t i = from; i < v.size(); ++i)
        if (std::abs(v[i]) < 20) return int(i);
    return -1;
}

} // namespace

TEST(sound_seq_decode) {
    Bytes t;
    put_number(t, 0);          // initial delta
    t.push_back(0x03), t.push_back(5), put_number(t, 0);      // program 5
    t.push_back(0x04), t.push_back(100), put_number(t, 0);    // volume
    t.push_back(0x05), t.push_back(0x40), put_number(t, 0);   // pan
    t.push_back(0x01), t.push_back(0xFF), put_number(t, 200); // endless loop
    t.push_back(0x80 | 60), t.push_back(127), put_number(t, 300), put_number(t, 480); // note
    t.push_back(0x40 | 0x30), t.push_back(0x00), put_number(t, 0); // bend 0x3000 - 0x2000
    t.push_back(0x22), put_number(t, 0);                       // loop x3
    t.push_back(0x02), put_number(t, 0);                       // loop end
    t.push_back(0x06), t.push_back(0x07), t.push_back(0xA1), t.push_back(0x20), put_number(t, 0); // tempo
    t.push_back(0x1F), put_number(t, 1);                       // undefined: no-op
    t.push_back(0x02), put_number(t, 0);
    t.push_back(0x00);

    SeqBank b = SeqBank::parse(make_seqg(t, 500000));
    CHECK(b.sequences.size() == 1);
    CHECK(b.sequences[0].tempo == 500000);
    CHECK(b.sequences[0].tracks[1] == 0);
    auto ev = decode_seq_track(*b.data, b.sequences[0].tracks[0]);
    CHECK(ev.size() == 12);
    CHECK(ev[0].ev.op == SeqOp::Program && ev[0].ev.value == 5);
    CHECK(ev[3].ev.op == SeqOp::LoopStart && ev[3].ev.value == kSeqLoopEndless);
    CHECK(ev[4].ev.op == SeqOp::Note && ev[4].tick == 200 && ev[4].ev.key == 60 && ev[4].ev.length == 300);
    CHECK(ev[5].ev.op == SeqOp::PitchBend && ev[5].ev.bend == 0x1000 && ev[5].tick == 680);
    CHECK(ev[6].ev.op == SeqOp::LoopStart && ev[6].ev.value == 3);
    CHECK(ev[8].ev.op == SeqOp::Tempo && ev[8].ev.tempo == 500000);
    CHECK(ev[9].ev.op == SeqOp::Nop && ev[9].ev.value == 0x1F);
    CHECK(ev.back().ev.op == SeqOp::End && ev.back().tick == 681);

    SeqSummary sum = summarize_sequence(b, 0);
    CHECK(sum.length_ticks == 681 && sum.loop_start_tick == 0 && sum.notes == 1);

    // Truncated data is an error, not a crash.
    Bytes bad = make_seqg(Bytes{0x00, 0x80 | 60, 100}, 500000);
    SeqBank bb = SeqBank::parse(bad);
    bool threw = false;
    try {
        decode_seq_track(*bb.data, bb.sequences[0].tracks[0]);
    } catch (const FormatError&) {
        threw = true;
    }
    CHECK(threw);
}

TEST(sound_seq_music_index) {
    // Two groups of one record each, strings after the records.
    Bytes b(0x14 + 16 + 40);
    std::memcpy(b.data(), "MSEQ", 4);
    put32(b, 4, 1);
    put32(b, 0x0C, 2);
    put32(b, 0x10, 2);
    put32(b, 0x14, 0x24), put32(b, 0x18, 1);
    put32(b, 0x1C, 0x38), put32(b, 0x20, 1);
    u32 str = u32(b.size());
    for (const char* s : {"gtmenu01", "music.seq", "menu-03b", "Someone", "home01", "home2"}) {
        b.insert(b.end(), s, s + std::strlen(s) + 1);
    }
    put32(b, 0x24, str), put32(b, 0x28, str + 9), put32(b, 0x2C, str + 19), put32(b, 0x30, str + 28), put32(b, 0x34, 7);
    put32(b, 0x38, str + 36), put32(b, 0x3C, str + 9), put32(b, 0x40, str + 43), put32(b, 0x44, str + 28),
        put32(b, 0x48, 8);
    auto idx = parse_music_index(b);
    CHECK(idx.size() == 2);
    CHECK(idx[0].name == "gtmenu01" && idx[0].file == "music.seq" && idx[0].title == "menu-03b" &&
          idx[0].sequence == 7 && idx[0].group == 0);
    CHECK(idx[1].name == "home01" && idx[1].title == "home2" && idx[1].sequence == 8 && idx[1].group == 1);
}

TEST(sound_seq_timing) {
    // Tempo 1,000,000 us per 480 ticks at 60 updates/s: exactly 8 ticks per
    // update. A note 16 ticks in with length 8 sounds from update 2; the
    // game keys it off one update later than its nominal length (update 4).
    Bytes t;
    put_number(t, 16);
    t.push_back(0x05), t.push_back(0x40), put_number(t, 0);
    t.push_back(0x80 | 60), t.push_back(127), put_number(t, 8), put_number(t, 100);
    t.push_back(0x00);
    auto seqs = std::make_shared<const SeqBank>(SeqBank::parse(make_seqg(t, 1000000)));
    auto bank = make_bank(0x000F, 0x0000); // instant attack, full sustain, instant release
    Sequencer seq;
    seq.set_instruments(bank);
    seq.set_sequences(seqs);
    VoiceMixer mixer;
    mixer.set_master_volume(1.0f);
    CHECK(seq.play(mixer, 0));
    auto left = run(seq, mixer, 8);
    int on = first_loud(left), off = first_quiet(left, size_t(std::max(on, 0)));
    // The update that starts the note is the second one (frames 800..1599).
    CHECK(on >= 800 && on < 810);
    CHECK(off >= 2400 && off < 2420);
    CHECK(seq.ticks() == 64);
    // Level: full-scale program/velocity/region volumes squared stay just
    // under 0x3FFF; at the centre pan (64) the left gain is the sine table's
    // entry 127 - 64 = 63 (11519 / 16384).
    int peak = 0;
    for (int i = on; i < off; ++i) peak = std::max(peak, int(left[size_t(i)]));
    CHECK(std::abs(peak - int(4076 * 11519 / 16384.0)) < 8);
}

TEST(sound_seq_loops_and_bend) {
    // Endless loop of 96 ticks containing one note; tempo 500,000 us per
    // beat = 16 ticks per update, so each pass is 6 updates.
    Bytes t;
    put_number(t, 0);
    t.push_back(0x05), t.push_back(0x40), put_number(t, 0);
    t.push_back(0x01), t.push_back(0xFF), put_number(t, 0);
    t.push_back(0x80 | 72), t.push_back(127), put_number(t, 40), put_number(t, 48);
    t.push_back(0x40 | 0x3F), t.push_back(0xFF), put_number(t, 48); // bend fully up
    t.push_back(0x40 | 0x20), t.push_back(0x00), put_number(t, 0);  // bend centre
    t.push_back(0x02), put_number(t, 0);
    t.push_back(0x00);
    auto seqs = std::make_shared<const SeqBank>(SeqBank::parse(make_seqg(t, 500000)));
    Sequencer seq;
    seq.set_instruments(make_bank());
    seq.set_sequences(seqs);
    VoiceMixer mixer;
    CHECK(seq.play(mixer, 0));
    for (int u = 0; u < 60; ++u) seq.update(mixer);
    CHECK(seq.ticks() == 960);
    CHECK(seq.loops_completed() == 10);
    CHECK(seq.running());
    seq.stop(mixer);
    CHECK(!seq.active());
    CHECK(seq.sounding_notes() == 0);
}

TEST(sound_seq_pitch_and_volume_curves) {
    // Pitch bend: +8191 with a 2-semitone range is just under 2 semitones.
    InstRegion r;
    r.center = 60 << 8;
    r.bend_up = 2;
    int note = (72 << 8) + ((8191 * r.bend_up) >> 5);
    CHECK(note == (74 << 8) - 1);
    // (The pitch table has 1/32-semitone steps, so the last 1/256 rounds down.)
    CHECK(std::abs(note_to_pitch(note, r.center) / (0x2000 * std::exp2(2 / 12.0)) - 1) < 0.003);
    // Hardware pitch conversion matches the game's packer constant.
    CHECK(game_to_spu_pitch(0x1000) == 0x0EB3);
    CHECK(std::abs(VoiceMixer::hardware_pitch(0x1000) * 48000.0 / 4096 - 44100) < 20);
}

TEST(sound_spu_envelope) {
    // Linear attack at rate 32 (shift 8, step +7 << 3 per sample): full
    // level after 0x7FFF / 56 = 585 samples.
    Bytes loop;
    for (int f = 0; f < 2; ++f) {
        loop.push_back(0);
        loop.push_back(f ? 0x03 : 0x06);
        for (int i = 0; i < 14; ++i) loop.push_back(0x11);
    }
    VoiceMixer m;
    VoiceParams p;
    p.data = std::make_shared<const Bytes>(loop);
    p.adsr1 = 0x200F;
    p.adsr2 = 0x0000;
    VoiceId v = m.key_on(p);
    std::vector<s16> out(2 * 600);
    m.mix(out.data(), 292);
    CHECK(std::abs(m.envelope(v) - 0.5f) < 0.01f);
    m.mix(out.data(), 300);
    CHECK(m.envelope(v) > 0.999f);
    // Exponential release at rate 0x0A << 2 = 40 (shift 10): the step is
    // -8 << 1 scaled by the level, so the level falls by 1/2048 per sample:
    // to 1/e in ~2048 samples.
    VoiceParams q = p;
    q.adsr1 = 0x000F;
    q.adsr2 = 0x002A;
    VoiceId w = m.key_on(q);
    m.mix(out.data(), 10);
    m.key_off(w);
    std::vector<s16> big(2 * 2048);
    m.mix(big.data(), 2048);
    CHECK(std::abs(m.envelope(w) - std::exp(-1.0f)) < 0.02f);
    // Sustain rate 0x7F never moves; release shift 0x1F never moves either.
    VoiceParams h = p;
    h.adsr1 = 0x0001; // fastest attack and decay, sustain level 1 (0x1000)
    h.adsr2 = 0x1FC0 | 0x1F;
    VoiceId x = m.key_on(h);
    m.mix(big.data(), 2048);
    float level = m.envelope(x);
    CHECK(level > 0.06f && level <= 0x1000 / 32767.0f + 1e-4f);
    m.mix(big.data(), 2048);
    CHECK(m.envelope(x) == level);
    m.key_off(x);
    m.mix(big.data(), 2048);
    CHECK(m.envelope(x) == level && m.releasing(x));
}

TEST(sound_spu_reverb) {
    SpuReverb r;
    r.set_mode(SpuReverb::StudioC);
    r.set_depth(0x7FFF, 0x7FFF);
    // Impulse response: silent for the early delay, then a decaying tail.
    double early = 0, mid = 0, late = 0;
    for (int n = 0; n < 48000 * 3; ++n) {
        s32 in = n < 8 ? 20000 : 0, l, rr;
        r.process(in, in, l, rr);
        double e = double(l) * l + double(rr) * rr;
        if (n < 100) early += e;
        else if (n < 24000) mid += e;
        else if (n > 120000) late += e;
    }
    CHECK(early == 0);
    CHECK(mid > 1e6);
    CHECK(late < mid * 1e-3);
    // Off: no output; depth scales linearly.
    SpuReverb off;
    off.set_mode(SpuReverb::Off);
    s32 l = 1, rr = 1;
    off.process(30000, 30000, l, rr);
    CHECK(l == 0 && rr == 0);
    SpuReverb a, b;
    a.set_mode(SpuReverb::Hall), b.set_mode(SpuReverb::Hall);
    a.set_depth(0x7FFF, 0x7FFF), b.set_depth(0x0FFF, 0x0FFF);
    double ea = 0, eb = 0;
    for (int n = 0; n < 48000; ++n) {
        s32 in = (n % 4800) < 4 ? 20000 : 0, la, ra, lb, rb;
        a.process(in, -in, la, ra);
        b.process(in, -in, lb, rb);
        ea += std::abs(la), eb += std::abs(lb);
    }
    CHECK(std::abs(eb / ea - 0x0FFF / double(0x7FFF)) < 0.01);
}

TEST(sound_seq_on_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Volume& vol = disc->volume();
    auto idx = parse_music_index(vol.read(*vol.find("music/music.inf")));
    CHECK(idx.size() == 18);
    auto seqs = std::make_shared<const SeqBank>(SeqBank::parse(vol.read(*vol.find("music/music.seq"))));
    auto bank = std::make_shared<const InstBank>(InstBank::parse(vol.read(*vol.find("music/music.ins"))));
    CHECK(seqs->sequences.size() == 9);
    for (const MusicEntry& e : idx) CHECK(e.sequence < seqs->sequences.size() && e.file == "music.seq");
    size_t notes = 0, unmapped = 0;
    for (size_t i = 0; i < seqs->sequences.size(); ++i) {
        SeqSummary sum = summarize_sequence(*seqs, i);
        CHECK(sum.loop_start_tick >= 0); // every sequence loops forever
        notes += sum.notes;
        for (u32 off : seqs->sequences[i].tracks) {
            if (!off) continue;
            int program = 0;
            for (const auto& te : decode_seq_track(*seqs->data, off)) {
                if (te.ev.op == SeqOp::Program) program = te.ev.value;
                if (te.ev.op != SeqOp::Note) continue;
                bool hit = false;
                for (u8 r : bank->programs.at(size_t(program)).regions)
                    hit |= te.ev.key >= bank->regions[r].key_lo && te.ev.key <= bank->regions[r].key_hi;
                unmapped += !hit;
            }
        }
    }
    CHECK(notes == 35909);
    // 84 drum notes (sequence 4, key 47) have no region in their kit; the
    // game plays nothing for them either.
    CHECK(unmapped == 84);

    // Sequence 5 ("option"): render one loop. The loop period must match the
    // tempo (to within one 60 Hz update) and nothing may clip.
    SequenceRenderOptions opt;
    opt.loops = 2;
    opt.tail_seconds = 1;
    SequenceRender rr = render_sequence(bank, seqs, 5, opt);
    CHECK(rr.loop_frames.size() == 2);
    if (rr.loop_frames.size() == 2) {
        SeqSummary sum = summarize_sequence(*seqs, 5);
        double period = seq_ticks_to_seconds(sum.length_ticks - u64(sum.loop_start_tick), seqs->sequences[5].tempo);
        double got = double(rr.loop_frames[1] - rr.loop_frames[0]) / VoiceMixer::kRate;
        std::printf("  sequence 5 loop: %.3f s expected, %.3f s rendered, peak %d, %d voices\n", period, got, rr.peak,
                    rr.max_voices);
        CHECK(std::abs(got - period) < 1.0 / 60 + 1e-6);
    }
    CHECK(rr.clipped == 0);
    CHECK(rr.peak > 2000);
    CHECK(rr.max_voices <= 46);
}

TEST(sound_seq_tick_hook_is_sample_exact) {
    // Driving the sequencer from the mixer's tick hook with arbitrary
    // buffer sizes must give the same samples as update() + mix(800).
    Bytes t;
    put_number(t, 3);
    t.push_back(0x05), t.push_back(0x40), put_number(t, 0);
    t.push_back(0x01), t.push_back(0xFF), put_number(t, 0);
    t.push_back(0x80 | 60), t.push_back(100), put_number(t, 5), put_number(t, 7);
    t.push_back(0x80 | 67), t.push_back(90), put_number(t, 9), put_number(t, 13);
    t.push_back(0x02), put_number(t, 0);
    t.push_back(0x00);
    auto seqs = std::make_shared<const SeqBank>(SeqBank::parse(make_seqg(t, 1234567)));
    auto bank = make_bank(0x0A0F, 0x0008);

    Sequencer a;
    a.set_instruments(bank), a.set_sequences(seqs);
    VoiceMixer ma;
    a.play(ma, 0);
    std::vector<s16> ref;
    std::vector<s16> buf(800 * 2);
    for (int u = 0; u < 30; ++u) {
        a.update(ma);
        ma.mix(buf.data(), 800);
        ref.insert(ref.end(), buf.begin(), buf.end());
    }

    Sequencer b;
    b.set_instruments(bank), b.set_sequences(seqs);
    VoiceMixer mb;
    mb.set_tick_hook([&] { b.update(mb); }, b.samples_per_update());
    b.play(mb, 0);
    std::vector<s16> got;
    size_t sizes[] = {1, 317, 800, 1999, 64, 5};
    for (size_t i = 0; got.size() < ref.size(); ++i) {
        size_t n = std::min(sizes[i % 6], (ref.size() - got.size()) / 2);
        std::vector<s16> chunk(n * 2);
        mb.mix(chunk.data(), n);
        got.insert(got.end(), chunk.begin(), chunk.end());
    }
    CHECK(got == ref);
    CHECK(b.loops_completed() == a.loops_completed() && a.loops_completed() > 0);
}
