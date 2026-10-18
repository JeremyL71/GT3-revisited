#pragma once

// PSS: the PS2 movie container (stream/*.pss). It is an MPEG-2 program
// stream (ISO/IEC 13818-1): 2048-byte packs, each a pack header followed by
// PES packets.
//   0xE0       MPEG-2 video elementary stream (MP@ML, 640x448 on GT3)
//   0xBD       private stream 1: PS2 audio. Every payload starts with a
//              4-byte sub-header (FF A1 00 00 on GT3); the rest, concatenated
//              over all packets, is a complete .ads file ("SShd" header,
//              "SSbd" body of PS-ADPCM interleaved per channel).
//   0xBB/0xBE  system header / padding (ignored)
// See docs/formats/pss.md.
//
// The demuxer reads sequentially through a ReadFn, so files stream from the
// disc without being loaded whole.

#include "base/bytes.h"

#include <functional>

namespace gt3r {

struct PssPacket {
    u8 stream_id = 0;     // 0xE0..0xEF video, 0xC0..0xDF MPEG audio, 0xBD private 1
    u8 sub_id = 0;        // private stream 1: first payload byte (0xFF on GT3 audio)
    s64 pts = -1, dts = -1; // 90 kHz, -1 when absent
    u64 offset = 0;       // file offset of the packet start code
    const u8* data = nullptr; // payload (private stream 1: after the 4-byte sub-header)
    size_t size = 0;      // valid until the next call to next()
};

class PssDemuxer {
public:
    using ReadFn = std::function<void(u64 offset, void* out, size_t len)>;

    static constexpr u8 kVideoStream = 0xE0;
    static constexpr u8 kPrivateStream1 = 0xBD;
    static constexpr size_t kAudioSubHeader = 4;

    PssDemuxer(ReadFn read, u64 size);

    // Next elementary-stream packet (video, audio, private 1). False at end.
    bool next(PssPacket& out);
    void rewind();

    u64 size() const { return size_; }
    u64 position() const { return pos_; }
    u64 resyncs() const { return resyncs_; } // bytes skipped looking for start codes

    // True if `head` looks like a program stream (starts with a pack header).
    static bool probe(ByteSpan head);

private:
    // Makes [pos, pos+len) available in buf_; false if it extends past EOF.
    bool ensure(u64 pos, size_t len);

    ReadFn read_;
    u64 size_;
    u64 pos_ = 0;
    Bytes buf_;
    u64 buf_pos_ = 0; // file offset of buf_[0]
    size_t buf_len_ = 0;
    u64 resyncs_ = 0;
};

} // namespace gt3r
