#include "formats/pss.h"

#include <algorithm>
#include <cstring>

namespace gt3r {

namespace {

constexpr size_t kChunk = 64 * 1024;

s64 read_timestamp(const u8* p) {
    return (s64(p[0] >> 1) & 7) << 30 | s64(p[1]) << 22 | s64(p[2] >> 1) << 15 | s64(p[3]) << 7 | s64(p[4] >> 1);
}

bool is_es_stream(u8 id) { return id == 0xBD || (id >= 0xC0 && id <= 0xEF); }

} // namespace

PssDemuxer::PssDemuxer(ReadFn read, u64 size) : read_(std::move(read)), size_(size) {}

bool PssDemuxer::probe(ByteSpan h) { return h.size() >= 4 && h[0] == 0 && h[1] == 0 && h[2] == 1 && h[3] == 0xBA; }

void PssDemuxer::rewind() {
    pos_ = 0;
    buf_len_ = 0;
    buf_pos_ = 0;
}

bool PssDemuxer::ensure(u64 pos, size_t len) {
    if (pos + len > size_) return false;
    if (pos >= buf_pos_ && pos + len <= buf_pos_ + buf_len_) return true;
    size_t want = size_t(std::min<u64>(std::max(len, kChunk), size_ - pos));
    if (buf_.size() < want) buf_.resize(want);
    read_(pos, buf_.data(), want);
    buf_pos_ = pos;
    buf_len_ = want;
    return true;
}

bool PssDemuxer::next(PssPacket& out) {
    for (;;) {
        if (!ensure(pos_, 4)) return false;
        const u8* p = buf_.data() + (pos_ - buf_pos_);
        if (p[0] != 0 || p[1] != 0 || p[2] != 1) {
            ++pos_;
            ++resyncs_;
            continue;
        }
        u8 id = p[3];
        if (id == 0xBA) { // pack header
            if (!ensure(pos_, 14)) return false;
            p = buf_.data() + (pos_ - buf_pos_);
            if ((p[4] & 0xC0) == 0x40) pos_ += 14 + (p[13] & 7); // MPEG-2
            else pos_ += 12;                                       // MPEG-1
            continue;
        }
        if (id == 0xB9) { // program end
            pos_ += 4;
            continue;
        }
        if (id < 0xBB) { // not a system start code: garbage
            ++pos_;
            ++resyncs_;
            continue;
        }
        if (!ensure(pos_, 6)) return false;
        p = buf_.data() + (pos_ - buf_pos_);
        size_t len = size_t(p[4]) << 8 | p[5];
        u64 start = pos_;
        if (!ensure(pos_, 6 + len)) return false;
        pos_ += 6 + len;
        if (!is_es_stream(id)) continue;

        p = buf_.data() + (start - buf_pos_);
        const u8* body = p + 6;
        const u8* end = body + len;
        PssPacket pkt;
        pkt.stream_id = id;
        pkt.offset = start;
        if (len >= 3 && (body[0] & 0xC0) == 0x80) { // MPEG-2 PES header
            u8 flags = body[1];
            size_t hlen = body[2];
            if (3 + hlen > len) continue;
            if ((flags & 0x80) && hlen >= 5) pkt.pts = read_timestamp(body + 3);
            if ((flags & 0xC0) == 0xC0 && hlen >= 10) pkt.dts = read_timestamp(body + 8);
            body += 3 + hlen;
        } else { // MPEG-1 PES header
            while (body < end && *body == 0xFF) ++body; // stuffing
            if (body < end && (*body & 0xC0) == 0x40) body += 2; // STD buffer
            if (body + 5 <= end && (*body & 0xF0) == 0x20) {
                pkt.pts = read_timestamp(body);
                body += 5;
            } else if (body + 10 <= end && (*body & 0xF0) == 0x30) {
                pkt.pts = read_timestamp(body);
                pkt.dts = read_timestamp(body + 5);
                body += 10;
            } else if (body < end) {
                ++body; // 0000 1111
            }
        }
        if (body > end) continue;
        if (id == kPrivateStream1) {
            if (end - body < std::ptrdiff_t(kAudioSubHeader)) continue;
            pkt.sub_id = body[0];
            body += kAudioSubHeader;
        }
        pkt.data = body;
        pkt.size = size_t(end - body);
        out = pkt;
        return true;
    }
}

} // namespace gt3r
