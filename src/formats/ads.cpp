#include "formats/ads.h"

#include <algorithm>

namespace gt3r {

AdsHeader AdsHeader::parse(ByteSpan d) {
    if (!has_magic(d, "SShd") || d.size() < kHeaderBytes) throw FormatError("not an ADS stream");
    AdsHeader h;
    h.codec = rd32(d, 0x08);
    h.sample_rate = rd32(d, 0x0C);
    h.channels = rd32(d, 0x10);
    h.interleave = rd32(d, 0x14);
    if (std::memcmp(&d[0x20], "SSbd", 4) != 0) throw FormatError("ADS: missing SSbd");
    h.body_size = rd32(d, 0x24);
    h.body_offset = kHeaderBytes;
    if (h.channels == 0 || h.channels > 8 || h.sample_rate == 0) throw FormatError("ADS: bad format");
    if (h.codec != 0x10 && h.codec != 0x01) throw FormatError("ADS: unsupported codec");
    if (h.interleave == 0 || h.interleave % 16) throw FormatError("ADS: bad interleave");
    return h;
}

u64 AdsHeader::total_frames() const {
    u64 per_channel = body_size / channels;
    return codec == 0x10 ? per_channel / 16 * 28 : per_channel / 2;
}

AdsStream::AdsStream(const AdsHeader& header, ReadFn read, bool loop)
    : hdr_(header), read_(std::move(read)), loop_(loop), decoders_(header.channels) {}

void AdsStream::rewind() {
    block_ = 0;
    pcm_.clear();
    pcm_pos_ = 0;
    frames_out_ = 0;
    for (auto& d : decoders_) d.reset();
}

double AdsStream::position_seconds() const { return double(frames_out_) / hdr_.sample_rate; }

bool AdsStream::refill() {
    const u32 ch = hdr_.channels, il = hdr_.interleave;
    u64 off = u64(block_) * il * ch;
    if (off >= hdr_.body_size) return false;
    size_t len = size_t(std::min<u64>(u64(il) * ch, hdr_.body_size - off));
    scratch_.resize(len);
    read_(hdr_.body_offset + off, scratch_.data(), len);
    ++block_;

    size_t per_ch = len / ch;
    size_t frames = hdr_.codec == 0x10 ? per_ch / 16 * 28 : per_ch / 2;
    pcm_.assign(frames * ch, 0);
    pcm_pos_ = 0;
    for (u32 c = 0; c < ch; ++c) {
        const u8* src = scratch_.data() + size_t(c) * per_ch;
        if (hdr_.codec == 0x10) {
            s16 tmp[28];
            for (size_t f = 0; f < per_ch / 16; ++f) {
                decoders_[c].decode_frame(src + f * 16, tmp);
                for (size_t i = 0; i < 28; ++i) pcm_[(f * 28 + i) * ch + c] = tmp[i];
            }
        } else {
            for (size_t i = 0; i < frames; ++i) {
                s16 s;
                std::memcpy(&s, src + i * 2, 2);
                pcm_[i * ch + c] = s;
            }
        }
    }
    return frames > 0;
}

size_t AdsStream::read(s16* out, size_t frames) {
    const u32 ch = hdr_.channels;
    size_t done = 0;
    while (done < frames) {
        size_t avail = pcm_.size() / ch - pcm_pos_;
        if (avail == 0) {
            if (!refill()) {
                if (!loop_ || frames_out_ == 0) break;
                rewind();
                continue;
            }
            continue;
        }
        size_t n = std::min(avail, frames - done);
        std::memcpy(out + done * ch, pcm_.data() + pcm_pos_ * ch, n * ch * sizeof(s16));
        pcm_pos_ += n;
        done += n;
        frames_out_ += n;
    }
    return done;
}

} // namespace gt3r
