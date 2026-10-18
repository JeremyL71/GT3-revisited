#pragma once

// "ENGN" engine sound bank: sound/engine/<id>, sound/normal0..3/<id>,
// sound/turbo0..3/<id> and sound/rn_01.es. A set of looped PS-ADPCM
// recordings of one engine, each at a fixed RPM ("layers").
//
//   0x00 "ENGN"
//   0x04 u32 0           relocation base (written by the game at load)
//   0x08 u32 header size (= data offset)
//   0x0C u32 0           SPU address (written by the game at load)
//   0x10 u32 data size
//   0x14 u32 data offset
//   0x18 u32 layer count, 0x1C u32 layer table offset (0x30)
//   0x20 u32 0, 0x24 u32 0   second (unused) layer table slot
//   0x28 u32 127, 0x2C u32 127   unknown, constant
//   0x30 layers[count], 16 bytes each:
//        u16 base_rpm   RPM the recording was made at
//        s16 lo_rpm     layer plays alone for lo..hi; between the hi of one
//        s16 hi_rpm     layer and the lo of the next the two are crossfaded
//        s16 volume     0..0x3FFF
//        u32 pitch      game pitch that reproduces base_rpm (0x1000 = 44.1 kHz)
//        u32 offset     sample start, relative to the data
// Each layer is one SPU sample: a silent frame, a loop-start frame, ...,
// a loop-end frame. See docs/formats/sound.md.

#include "formats/sound/spu.h"

#include <memory>
#include <vector>

namespace gt3r::sound {

struct EngineLayer {
    u16 base_rpm = 0;
    s16 lo_rpm = 0;
    s16 hi_rpm = 0;
    s16 volume = 0;
    u32 pitch = 0;
    u32 offset = 0; // into EngineBank::adpcm
    u32 size = 0;   // bytes up to the next layer
};

struct EngineBank {
    std::vector<EngineLayer> layers;
    u32 unknown_28 = 0, unknown_2c = 0;
    std::shared_ptr<const Bytes> adpcm; // sample data, shared with mixer voices

    static EngineBank parse(ByteSpan file);
    SpuSample decode_layer(size_t i) const { return decode_spu_sample(*adpcm, layers.at(i).offset); }
    // Effective sample rate of a layer when played at its base RPM.
    double layer_rate(size_t i) const { return game_pitch_rate(layers.at(i).pitch); }
};

} // namespace gt3r::sound
