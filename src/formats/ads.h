#pragma once

#include "formats/ps_adpcm.h"

#include <functional>
#include <vector>

namespace gt3r {

// PS2 "ADS" stream (SShd header + SSbd body), used for GT3's music.
//   "SShd" u32 0x18, u32 codec (0x01 PCM16LE, 0x10 PS-ADPCM), u32 rate,
//   u32 channels, u32 interleave, u32 loop_start, u32 loop_end
//   "SSbd" u32 body size, body (channels interleaved every `interleave` bytes)
struct AdsHeader {
    u32 codec = 0;
    u32 sample_rate = 0;
    u32 channels = 0;
    u32 interleave = 0;
    u32 body_offset = 0;
    u32 body_size = 0;

    static constexpr u32 kHeaderBytes = 0x28;
    static AdsHeader parse(ByteSpan first_bytes);
    u64 total_frames() const; // sample frames per channel
};

// Pull-model decoder producing interleaved s16 PCM. `read` fetches body
// bytes by offset relative to the start of the file, so the stream can be
// fed from a disc file without loading it whole.
class AdsStream {
public:
    using ReadFn = std::function<void(u64 offset, void* out, size_t len)>;
    AdsStream(const AdsHeader& header, ReadFn read, bool loop);

    const AdsHeader& header() const { return hdr_; }
    // Fills up to `frames` sample frames; returns frames produced (0 = end).
    size_t read(s16* out, size_t frames);
    void rewind();
    double position_seconds() const;

private:
    bool refill(); // decodes the next interleave block of every channel
    AdsHeader hdr_;
    ReadFn read_;
    bool loop_;
    u64 block_ = 0;
    std::vector<PsAdpcmDecoder> decoders_;
    std::vector<s16> pcm_; // interleaved decoded samples
    size_t pcm_pos_ = 0;   // in frames
    u64 frames_out_ = 0;
    Bytes scratch_;
};

} // namespace gt3r
