#include "video/movie.h"

#include "vfs/vfs.h"

#include <algorithm>
#include <cstring>

namespace gt3r::video {

namespace {
constexpr size_t kProbePackets = 1024;
constexpr u64 kProbeBytes = 256 * 1024; // audio, if any, starts well within this
constexpr size_t kMaxQueuedVideoBytes = 32u << 20; // safety cap if audio is pulled far ahead
} // namespace

double MovieInfo::estimated_seconds() const {
    if (audio_seconds > 0) return audio_seconds;
    return bit_rate ? double(file_size) * 8.0 / bit_rate : 0.0;
}

Movie::Movie(ReadFn read, u64 size) : demux_(std::move(read), size) {
    info_.file_size = size;
    // Demux until the video sequence header is decoded and the audio header
    // (if the movie has audio) is buffered. Packets are queued, not lost.
    for (size_t n = 0; n < kProbePackets; ++n) {
        if (!demux_one()) break;
        if (info_.start_pts >= 0 && (info_.has_audio || demux_.position() > kProbeBytes)) break;
    }
    if (info_.start_pts < 0 && video_q_.empty()) throw FormatError("PSS: no MPEG video stream found");
    // Parse the sequence header without consuming queued video: scan the
    // first chunk(s) for 00 00 01 B3 and decode those fields with a scratch decoder.
    Mpeg2Decoder probe;
    for (const VideoChunk& c : video_q_) {
        probe.push(c.data.data(), c.data.size(), -1);
        if (probe.sequence().valid) break;
    }
    const SequenceInfo& s = probe.sequence();
    if (!s.valid) throw FormatError("PSS: no MPEG sequence header in the first packets");
    info_.width = s.width;
    info_.height = s.height;
    info_.frame_rate = s.frame_rate;
    info_.bit_rate = s.bit_rate;
}

Movie::~Movie() = default;

bool Movie::demux_one() {
    if (eof_) return false;
    PssPacket pkt;
    if (!demux_.next(pkt)) {
        eof_ = true;
        return false;
    }
    if (pkt.stream_id == PssDemuxer::kVideoStream) {
        if (info_.start_pts < 0 && pkt.pts >= 0) info_.start_pts = pkt.pts;
        if (video_enabled_ || info_.width == 0) video_q_.push_back({Bytes(pkt.data, pkt.data + pkt.size), pkt.pts});
    } else if (pkt.stream_id == PssDemuxer::kPrivateStream1) {
        if (!info_.has_audio && audio_fifo_base_ == 0 && audio_fifo_.empty()) {
            // First audio packet: it must start with the ADS header.
            if (pkt.size >= AdsHeader::kHeaderBytes && has_magic(ByteSpan(pkt.data, pkt.size), "SShd")) {
                AdsHeader h = AdsHeader::parse(ByteSpan(pkt.data, pkt.size));
                info_.has_audio = true;
                info_.audio_rate = h.sample_rate;
                info_.audio_channels = h.channels;
                info_.audio_start_pts = pkt.pts;
                info_.audio_seconds = double(h.total_frames()) / h.sample_rate;
                audio_ = std::make_unique<AdsStream>(
                    h, [this](u64 off, void* out, size_t len) { read_audio_bytes(off, out, len); }, false);
            } else {
                return true; // unknown private stream
            }
        }
        if (audio_enabled_) audio_fifo_.insert(audio_fifo_.end(), pkt.data, pkt.data + pkt.size);
        else audio_fifo_base_ += pkt.size;
    }
    return true;
}

bool Movie::next_frame(Frame& out) {
    for (;;) {
        if (decoder_.pop(out)) return true;
        if (!video_q_.empty()) {
            VideoChunk c = std::move(video_q_.front());
            video_q_.pop_front();
            decoder_.push(c.data.data(), c.data.size(), c.pts);
            continue;
        }
        if (demux_one()) continue;
        if (!flushed_) {
            decoder_.flush();
            flushed_ = true;
            continue;
        }
        video_eof_ = true;
        return false;
    }
}

void Movie::read_audio_bytes(u64 offset, void* out, size_t len) {
    // AdsStream reads monotonically increasing offsets; demux until covered.
    size_t queued_video = 0;
    for (const VideoChunk& c : video_q_) queued_video += c.data.size();
    while (audio_fifo_base_ + audio_fifo_.size() < offset + len && queued_video < kMaxQueuedVideoBytes) {
        size_t before = video_q_.size();
        if (!demux_one()) break;
        if (video_q_.size() > before) queued_video += video_q_.back().data.size();
    }
    u8* dst = static_cast<u8*>(out);
    std::memset(dst, 0, len);
    if (offset < audio_fifo_base_) return; // already discarded (should not happen)
    u64 rel = offset - audio_fifo_base_;
    if (rel < audio_fifo_.size()) {
        size_t n = size_t(std::min<u64>(len, audio_fifo_.size() - rel));
        std::memcpy(dst, audio_fifo_.data() + rel, n);
    }
    // Drop everything before this read.
    size_t drop = size_t(std::min<u64>(rel, audio_fifo_.size()));
    if (drop > 64 * 1024) {
        audio_fifo_.erase(audio_fifo_.begin(), audio_fifo_.begin() + std::ptrdiff_t(drop));
        audio_fifo_base_ += drop;
    }
}

size_t Movie::read_audio(s16* out, size_t frames) {
    if (!audio_ || !audio_enabled_) return 0;
    return audio_->read(out, frames);
}

std::unique_ptr<Movie> open_movie(Vfs& vfs, const std::string& path) {
    auto stream = vfs.open_stream(path);
    if (!stream) throw FormatError("movie not found: " + path);
    return std::make_unique<Movie>(std::move(stream->read), stream->size);
}

} // namespace gt3r::video
