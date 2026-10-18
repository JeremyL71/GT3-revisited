#pragma once

// Sequenced music (menus): "MSEQ" index (music/music.inf) and "SEQG"
// sequence bank (music/music.seq), played with the instruments of
// music/music.ins. See docs/formats/sound.md.
//
// music.inf ("MSEQ"):
//   0x00 "MSEQ", 0x04 u32 1, 0x08 u32 0 (relocation base), 0x0C u32 group count
//   0x10 u32 group count again
//   0x14 { u32 record_offset; u32 record_count }[groups]
//   records, 20 bytes: u32 name, file, title, artist (string offsets),
//                      u32 sequence index into the SEQG file
//
// music.seq ("SEQG"):
//   0x00 "SEQG", 0x04 u32 0 (relocation base), 0x08 u32 sequence count
//   0x0C sequence[count], 0x48 bytes:
//        u32 volume (Q14, 0x4000 = 1.0)
//        u32 tempo  (microseconds per beat of 480 ticks)
//        u32 track_offset[16] (0 = no track)
//
// A track is a byte stream: an initial delta time, then (event, delta)
// pairs until the end event. Deltas and note lengths are MIDI-style
// variable-length numbers (7 bits per byte, big endian, bit 7 = more).
// Events by first byte b:
//   0x80..0xFF  note: key = b & 0x7F, u8 velocity, length (variable)
//   0x40..0x7F  pitch bend: ((b & 0x3F) << 8 | u8) - 0x2000
//   0x20..0x3F  loop start, count (b & 0x1F) + 1
//   0x00        end of track (no delta follows)
//   0x01 n      loop start, count n (0xFF = endless)
//   0x02        loop end
//   0x03 n      program (index into the INST program table)
//   0x04 n      channel volume 0..127
//   0x05 n      channel pan 0..127 (0x40 centre)
//   0x06 a b c  tempo, 24-bit big endian
//   0x07..0x1F  no operation (ignored by the game)

#include "base/bytes.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace gt3r::sound {

// One playable entry of music.inf. Several names can share a sequence.
struct MusicEntry {
    std::string name;   // id the game asks for, e.g. "gtmenu01"
    std::string file;   // "music.seq"
    std::string title;  // working title, e.g. "main-01"
    std::string artist;
    u32 sequence = 0;   // index into the SEQG bank
    u32 group = 0;      // index of the group the record sits in
};

std::vector<MusicEntry> parse_music_index(ByteSpan music_inf);

constexpr int kSeqTracks = 16;
constexpr int kSeqTicksPerBeat = 480;
constexpr u8 kSeqLoopEndless = 0xFF;

struct Sequence {
    u32 volume = 0x4000;                    // Q14
    u32 tempo = 500000;                     // microseconds per 480 ticks
    std::array<u32, kSeqTracks> tracks{};   // offsets into SeqBank::data, 0 = none
};

struct SeqBank {
    std::shared_ptr<const Bytes> data; // the whole file; track offsets index into it
    std::vector<Sequence> sequences;

    static SeqBank parse(ByteSpan file);
};

enum class SeqOp : u8 { Note, PitchBend, LoopStart, LoopEnd, Program, Volume, Pan, Tempo, Nop, End };

struct SeqEvent {
    SeqOp op = SeqOp::End;
    u8 key = 0, velocity = 0; // Note
    u8 value = 0;             // Program/Volume/Pan, LoopStart count, Nop: the raw byte
    s32 bend = 0;             // PitchBend, -8192..8191
    u32 length = 0;           // Note length in ticks
    u32 tempo = 0;            // Tempo
};

const char* seq_op_name(SeqOp op);

// Reads a variable-length number at `pos` and advances. Throws FormatError
// when it runs off the data or is longer than 4 bytes.
u32 read_seq_number(ByteSpan d, size_t& pos);
// Decodes the event at `pos` and advances past it (not past the delta that
// follows). Throws FormatError on truncated data.
SeqEvent read_seq_event(ByteSpan d, size_t& pos);

// A track flattened once from start to end (loops not expanded).
struct SeqTimedEvent {
    u32 tick = 0;   // ticks from the track start
    u32 offset = 0; // file offset of the event byte
    SeqEvent ev;
};
std::vector<SeqTimedEvent> decode_seq_track(ByteSpan file, u32 offset);

// Timing summary of a sequence, from its first track: total length and the
// endless loop's start tick (-1 if the sequence does not loop).
struct SeqSummary {
    u32 length_ticks = 0;
    s64 loop_start_tick = -1;
    u32 notes = 0;       // over all tracks, one pass
    int tracks_used = 0; // tracks with at least one note
};
SeqSummary summarize_sequence(const SeqBank& bank, size_t index);

inline double seq_ticks_to_seconds(u64 ticks, u32 tempo) { return double(ticks) * tempo / (1e6 * kSeqTicksPerBeat); }

} // namespace gt3r::sound
