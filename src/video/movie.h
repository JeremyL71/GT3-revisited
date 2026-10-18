#pragma once

// A PSS movie: demuxer + MPEG-2 video decoder + PS-ADPCM audio track, pulled
// independently by the consumer (video frames in display order, PCM audio).
// Data is read sequentially through a ReadFn; nothing is loaded whole.
// Not thread-safe: use from one thread (MoviePlayer runs it on a worker).

#include "formats/ads.h"
#include "formats/pss.h"
#include "video/mpeg2_decoder.h"

#include <deque>
#include <memory>
#include <string>

namespace gt3r {
class Vfs;
}

namespace gt3r::video {

struct MovieInfo {
    int width = 0, height = 0;
    double frame_rate = 0;
    s64 start_pts = -1;         // first video PTS (90 kHz)
    bool has_audio = false;
    u32 audio_rate = 0, audio_channels = 0;
    s64 audio_start_pts = -1;
    double audio_seconds = 0;   // from the ADS header
    u64 file_size = 0;
    u32 bit_rate = 0;           // from the sequence header
    double estimated_seconds() const; // from file size and bit rate (no PTS scan)
};

class Movie {
public:
    using ReadFn = PssDemuxer::ReadFn;

    // Probes the start of the stream (sequence header, audio header).
    // Throws FormatError if no MPEG video is found.
    Movie(ReadFn read, u64 size);
    ~Movie();

    const MovieInfo& info() const { return info_; }

    // Next frame in display order; false at end of stream.
    bool next_frame(Frame& out);
    // Interleaved s16 PCM; returns frames produced (0 = end or no audio).
    size_t read_audio(s16* out, size_t frames);
    // When disabled, demuxed audio is discarded (video-only consumers).
    void set_audio_enabled(bool on) { audio_enabled_ = on; }
    // When disabled, demuxed video is discarded (audio-only consumers).
    void set_video_enabled(bool on) { video_enabled_ = on; }

    const Mpeg2Decoder& decoder() const { return decoder_; }
    const PssDemuxer& demuxer() const { return demux_; }
    bool video_finished() const { return video_eof_ && decoder_.frames_ready() == 0; }

private:
    bool demux_one();             // routes one packet; false at EOF
    void read_audio_bytes(u64 offset, void* out, size_t len);

    PssDemuxer demux_;
    Mpeg2Decoder decoder_;
    MovieInfo info_;
    bool eof_ = false, video_eof_ = false, flushed_ = false;

    struct VideoChunk {
        Bytes data;
        s64 pts;
    };
    std::deque<VideoChunk> video_q_;

    bool audio_enabled_ = true;
    bool video_enabled_ = true;
    Bytes audio_fifo_;
    u64 audio_fifo_base_ = 0; // ADS-file offset of audio_fifo_[0]
    std::unique_ptr<AdsStream> audio_;
};

// Opens a movie through the VFS (Vfs::open_stream): disc files and files
// provided by mods both stream with ranged reads. Throws FormatError.
std::unique_ptr<Movie> open_movie(Vfs& vfs, const std::string& path);

} // namespace gt3r::video
