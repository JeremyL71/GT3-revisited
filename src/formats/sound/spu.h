#pragma once

// Helpers shared by GT3's sound banks, all of which hold PS2 SPU2 sample
// data (PS-ADPCM) and are driven with SPU2-style register values:
//   pitch  u16, 0x1000 = 44.1 kHz in the game's units. The game converts to
//          SPU2 hardware pitch (0x1000 = 48 kHz) when it sends voice
//          commands: hw = pitch * 0xEB3 >> 12 (game_to_spu_pitch).
//   volume s16, 0x3FFF = full scale
// See docs/formats/sound.md.

#include "formats/ps_adpcm.h"

#include <vector>

namespace gt3r::sound {

constexpr u32 kSpuRate = 48000;    // SPU2 output rate
constexpr u32 kGameRate = 44100;   // sample rate at game pitch 0x1000
constexpr u16 kSpuPitchUnity = 0x1000;

// The game's voice-command packer: game pitch (0x1000 = 44.1 kHz) to SPU2
// pitch (0x1000 = 48 kHz).
constexpr u16 game_to_spu_pitch(u16 pitch) { return u16((u32(pitch) * 0xEB3) >> 12); }
// Sample rate a game pitch value plays at.
inline double game_pitch_rate(u32 pitch) { return double(pitch) * kGameRate / kSpuPitchUnity; }

// PS-ADPCM frame flags (byte 1 of each 16-byte frame).
enum : u8 { kFlagEnd = 1, kFlagRepeat = 2, kFlagLoopStart = 4 };

// The game's note-to-pitch conversion: `note` and `center` are in 1/256
// semitone (note << 8, center_note << 8 | fine). Uses a 1/32-semitone table
// exactly like the original, so results match bit for bit.
u16 note_to_pitch(int note_x256, int center_x256);

// Pan law used by the game: sin curve over 0..127 (0 = left, 64 centre,
// 127 = right), 14-bit gains (16384 = unity).
void pan_gains(int pan, int& left, int& right);

// Equal-power crossfade curve used between engine layers: index 0..1024,
// result 0..4096 (4096 * sin(i / 1024 * pi / 2)).
int crossfade_gain(int index_0_1024);

// One decoded SPU sample.
struct SpuSample {
    std::vector<s16> pcm;
    size_t loop_start = 0; // sample index of the loop-start frame
    bool looped = false;   // the end frame has the repeat flag
    size_t frames = 0;     // 16-byte ADPCM frames consumed
};

// Decodes from `offset` until (and including) the first frame with the end
// flag, or the end of `data`.
SpuSample decode_spu_sample(ByteSpan data, size_t offset);

// Like the hardware: plays the sample and then `loops` more times through
// its loop, with the decoder history running on across each jump.
std::vector<s16> render_spu_loops(ByteSpan data, size_t offset, int loops);

} // namespace gt3r::sound
