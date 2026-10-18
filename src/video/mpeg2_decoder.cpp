#include "video/mpeg2_decoder.h"

#include "video/bit_reader.h"
#include "video/idct.h"
#include "video/mpeg2_tables.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

namespace gt3r::video {

namespace {

constexpr size_t kNone = size_t(-1);
constexpr size_t kUnitPad = 16; // zero bytes after each unit so BitReader can over-read
constexpr size_t kCompactThreshold = 256 * 1024;

// Start codes (the byte after 00 00 01).
constexpr u8 kPictureStart = 0x00;
constexpr u8 kSliceFirst = 0x01;
constexpr u8 kSliceLast = 0xAF;
constexpr u8 kSequenceHeader = 0xB3;
constexpr u8 kExtension = 0xB5;
constexpr u8 kSequenceEnd = 0xB7;
constexpr u8 kGroupStart = 0xB8;

enum PictureType { kI = 1, kP = 2, kB = 3 };

constexpr double kFrameRates[16] = {0, 24000.0 / 1001, 24, 25, 30000.0 / 1001, 30, 50, 60000.0 / 1001, 60,
                                    0, 0, 0, 0, 0, 0, 0};

inline u8 clamp255(int v) { return u8(v < 0 ? 0 : v > 255 ? 255 : v); }

// A reference or B picture at the coded size (multiple of 16).
struct Picture {
    int w = 0, h = 0; // luma coded size
    std::vector<u8> y, u, v;
    s64 pts = -1;
    char type = '?';

    void alloc(int width, int height) {
        w = width;
        h = height;
        y.assign(size_t(w) * h, 16);
        u.assign(size_t(w / 2) * (h / 2), 128);
        v.assign(size_t(w / 2) * (h / 2), 128);
    }
};

// Half-pel motion compensation of a w x h block. `src` points at the integer
// sample position in the reference; `avg` averages into dst (bidirectional).
template <int HX, int HY, bool AVG>
void mc_block(const u8* src, u8* dst, int stride, int w, int h) {
    for (int r = 0; r < h; ++r, src += stride, dst += stride) {
        for (int c = 0; c < w; ++c) {
            int p;
            if constexpr (!HX && !HY) p = src[c];
            else if constexpr (HX && !HY) p = (src[c] + src[c + 1] + 1) >> 1;
            else if constexpr (!HX && HY) p = (src[c] + src[c + stride] + 1) >> 1;
            else p = (src[c] + src[c + 1] + src[c + stride] + src[c + stride + 1] + 2) >> 2;
            if constexpr (AVG) dst[c] = u8((dst[c] + p + 1) >> 1);
            else dst[c] = u8(p);
        }
    }
}

using McFn = void (*)(const u8*, u8*, int, int, int);
constexpr McFn kMc[2][4] = {
    {mc_block<0, 0, false>, mc_block<1, 0, false>, mc_block<0, 1, false>, mc_block<1, 1, false>},
    {mc_block<0, 0, true>, mc_block<1, 0, true>, mc_block<0, 1, true>, mc_block<1, 1, true>},
};

// Predicts the block at (x, y) of a plane from `ref` displaced by the vector
// (mvx, mvy) in half-sample units. Vectors that leave the picture (illegal
// in MPEG-2) are clamped instead of reading out of bounds.
void predict_plane(const u8* ref, u8* dst, int stride, int plane_h, int x, int y, int mvx, int mvy, int w, int h,
                   bool avg) {
    int hx = mvx & 1, hy = mvy & 1;
    int sx = std::clamp(x + (mvx >> 1), 0, stride - w - hx);
    int sy = std::clamp(y + (mvy >> 1), 0, plane_h - h - hy);
    kMc[avg][hx | (hy << 1)](ref + size_t(sy) * stride + sx, dst + size_t(y) * stride + x, stride, w, h);
}

} // namespace

// blk is deliberately alignas(16); MSVC warns (C4324) about the padding that implies.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
struct Mpeg2Decoder::Impl {
    explicit Impl(Mpeg2Decoder& owner) : d(owner) {
        std::memcpy(intra_q, kDefaultIntraMatrix, 64);
        std::memset(nonintra_q, 16, 64);
    }

    Mpeg2Decoder& d;

    // --- elementary stream assembly -----------------------------------------
    Bytes es;
    u64 es_base = 0;   // absolute stream offset of es[0]
    size_t scan = 0;   // next position to search for a start code
    size_t unit = kNone; // start of the current (incomplete) unit
    struct PtsMark {
        u64 offset;
        s64 pts;
    };
    std::deque<PtsMark> marks;
    Bytes unit_buf;

    // --- sequence state ------------------------------------------------------
    bool have_seq = false;
    bool mpeg2 = false; // sequence extension seen
    int width = 0, height = 0, mb_w = 0, mb_h = 0;
    u8 intra_q[64];    // raster order
    u8 nonintra_q[64];

    // --- picture state -------------------------------------------------------
    bool in_picture = false, started = false, decode_this = false, have_pic_ext = false;
    int pic_type = 0;
    int f_code[2][2] = {{1, 1}, {1, 1}};
    int dc_precision = 0, pic_structure = 3;
    bool frame_pred_frame_dct = true, concealment = false, q_scale_type = false, intra_vlc = false,
         alternate_scan = false;
    s64 pic_pts = -1;
    bool closed_gop = false, drop_leading_b = false;

    Picture pics[3];
    int fwd = -1, bwd = -1, cur = -1;
    bool bwd_emitted = true;
    s64 last_out_pts = -1;
    u32 out_index = 0;

    // --- slice state ---------------------------------------------------------
    int qscale = 2;
    int dc_pred[3] = {};
    int pmv[2][2] = {};  // [s][t] (frame vectors: one predictor per direction)
    int last_flags = 0;
    int last_mv[2][2] = {};
    alignas(16) s16 blk[64];

    // ======================================================================
    void push(const u8* data, size_t len, s64 pts) {
        if (pts >= 0) marks.push_back({es_base + es.size(), pts});
        es.insert(es.end(), data, data + len);
        process(false);
    }

    void process(bool eof) {
        const size_t n = es.size();
        size_t i = scan;
        while (i + 3 < n) {
            // Find 00 00 01 xx.
            if (es[i + 2] > 1) {
                i += 3;
                continue;
            }
            if (es[i] != 0 || es[i + 1] != 0 || es[i + 2] != 1) {
                ++i;
                continue;
            }
            if (unit != kNone) handle_unit(unit, i);
            unit = i;
            i += 3;
        }
        scan = i;
        if (eof) {
            if (unit != kNone) handle_unit(unit, n);
            unit = kNone;
            scan = n;
        }
        compact();
    }

    void compact() {
        size_t keep_from = unit != kNone ? unit : scan;
        if (keep_from < kCompactThreshold) return;
        es.erase(es.begin(), es.begin() + std::ptrdiff_t(keep_from));
        es_base += keep_from;
        scan -= keep_from;
        if (unit != kNone) unit -= keep_from;
    }

    void handle_unit(size_t begin, size_t end) {
        if (end < begin + 4) return;
        u8 code = es[begin + 3];
        size_t len = end - begin - 4;
        unit_buf.assign(es.begin() + std::ptrdiff_t(begin + 4), es.begin() + std::ptrdiff_t(end));
        unit_buf.resize(len + kUnitPad, 0);
        const u8* p = unit_buf.data();
        try {
            if (code >= kSliceFirst && code <= kSliceLast) {
                if (!in_picture) return;
                if (!started) begin_picture();
                if (!decode_this) return;
                ++d.stats_.slices;
                try {
                    decode_slice(code, p, len);
                } catch (const FormatError&) {
                    ++d.stats_.slice_errors;
                }
                return;
            }
            switch (code) {
            case kPictureStart: {
                finish_picture();
                u64 abs = es_base + begin;
                s64 pts = -1;
                while (!marks.empty() && marks.front().offset <= abs) {
                    pts = marks.front().pts;
                    marks.pop_front();
                }
                picture_header(p, len, pts);
                break;
            }
            case kSequenceHeader:
                finish_picture();
                sequence_header(p, len);
                break;
            case kExtension:
                extension(p, len);
                break;
            case kGroupStart: {
                finish_picture();
                BitReader br(p, len);
                br.skip(25); // time code
                closed_gop = br.get1();
                bool broken_link = br.get1();
                if (broken_link) drop_leading_b = true;
                break;
            }
            case kSequenceEnd:
                finish_picture();
                emit_pending_reference();
                break;
            default: // user data, reserved codes, stray system start codes
                break;
            }
        } catch (const FormatError&) {
            // Corrupt header: drop the current picture and resynchronise on the next header.
            in_picture = false;
            ++d.stats_.skipped_pictures;
        }
    }

    // ======================================================================
    void sequence_header(const u8* p, size_t len) {
        BitReader br(p, len);
        int w = int(br.get(12)), h = int(br.get(12));
        int aspect = int(br.get(4)), frc = int(br.get(4));
        u32 rate = br.get(18);
        br.skip(1 + 10 + 1); // marker, vbv_buffer_size, constrained_parameters_flag
        if (br.get1())
            for (int i = 0; i < 64; ++i) intra_q[kZigzagScan[i]] = u8(br.get(8));
        else
            std::memcpy(intra_q, kDefaultIntraMatrix, 64);
        if (br.get1())
            for (int i = 0; i < 64; ++i) nonintra_q[kZigzagScan[i]] = u8(br.get(8));
        else
            std::memset(nonintra_q, 16, 64);
        if (w == 0 || h == 0) throw FormatError("MPEG-2: zero picture size");
        set_size(w, h);
        SequenceInfo& s = d.seq_;
        s.valid = true;
        s.aspect_code = aspect;
        s.frame_rate_code = frc;
        s.frame_rate = kFrameRates[frc];
        s.bit_rate = rate * 400;
        have_seq = true;
        mpeg2 = false; // until a sequence extension follows
    }

    void set_size(int w, int h) {
        if (w == width && h == height) return;
        width = w;
        height = h;
        mb_w = (w + 15) / 16;
        mb_h = (h + 15) / 16;
        for (Picture& pic : pics) pic.alloc(mb_w * 16, mb_h * 16);
        fwd = bwd = -1;
        bwd_emitted = true;
        d.seq_.width = w;
        d.seq_.height = h;
    }

    void extension(const u8* p, size_t len) {
        BitReader br(p, len);
        int id = int(br.get(4));
        if (id == 1) { // sequence extension
            SequenceInfo& s = d.seq_;
            s.profile_level = int(br.get(8));
            s.progressive = br.get1();
            s.chroma_format = int(br.get(2));
            int hext = int(br.get(2)), vext = int(br.get(2));
            u32 rate_ext = br.get(12);
            br.skip(1 + 8 + 1); // marker, vbv ext, low_delay
            int frn = int(br.get(2)), frd = int(br.get(5));
            if (s.chroma_format != 1) throw FormatError("MPEG-2: only 4:2:0 is supported");
            if (hext || vext) set_size(width + (hext << 12), height + (vext << 12));
            s.bit_rate += (rate_ext << 18) * 400;
            if (s.frame_rate > 0) s.frame_rate = s.frame_rate * (frn + 1) / (frd + 1);
            mpeg2 = true;
        } else if (id == 3) { // quant matrix extension (chroma matrices unused in 4:2:0)
            if (br.get1())
                for (int i = 0; i < 64; ++i) intra_q[kZigzagScan[i]] = u8(br.get(8));
            if (br.get1())
                for (int i = 0; i < 64; ++i) nonintra_q[kZigzagScan[i]] = u8(br.get(8));
        } else if (id == 8 && in_picture) { // picture coding extension
            f_code[0][0] = int(br.get(4));
            f_code[0][1] = int(br.get(4));
            f_code[1][0] = int(br.get(4));
            f_code[1][1] = int(br.get(4));
            dc_precision = int(br.get(2));
            pic_structure = int(br.get(2));
            br.skip(1); // top_field_first
            frame_pred_frame_dct = br.get1();
            concealment = br.get1();
            q_scale_type = br.get1();
            intra_vlc = br.get1();
            alternate_scan = br.get1();
            have_pic_ext = true;
        }
    }

    void picture_header(const u8* p, size_t len, s64 pts) {
        BitReader br(p, len);
        br.skip(10); // temporal_reference
        pic_type = int(br.get(3));
        in_picture = true;
        started = false;
        decode_this = false;
        have_pic_ext = false;
        pic_pts = pts;
    }

    // Called on the first slice of a picture, once all extensions are known.
    void begin_picture() {
        started = true;
        decode_this = false;
        bool ok = have_seq && mpeg2 && have_pic_ext && pic_structure == 3 &&
                  (pic_type == kI || pic_type == kP || pic_type == kB);
        if (ok && pic_type == kP && bwd < 0) ok = false; // no reference yet
        if (ok && pic_type == kB) {
            if (bwd < 0 || drop_leading_b) ok = false;
            else if (fwd < 0 && !closed_gop) ok = false;
        }
        if (!ok) {
            ++d.stats_.skipped_pictures;
            return;
        }
        cur = 0;
        while (cur == fwd || cur == bwd) ++cur;
        Picture& pic = pics[cur];
        pic.pts = pic_pts;
        pic.type = " IPB"[pic_type];
        decode_this = true;
    }

    void finish_picture() {
        if (!in_picture) return;
        in_picture = false;
        if (!started || !decode_this) return;
        ++d.stats_.pictures;
        if (pic_type == kB) {
            emit(cur);
        } else {
            emit_pending_reference();
            fwd = bwd;
            bwd = cur;
            bwd_emitted = false;
            if (pic_type == kP) drop_leading_b = false;
        }
    }

    void emit_pending_reference() {
        if (bwd >= 0 && !bwd_emitted) {
            emit(bwd);
            bwd_emitted = true;
        }
    }

    void emit(int idx) {
        const Picture& pic = pics[idx];
        Frame f;
        if (!d.spare_.empty()) {
            f = std::move(d.spare_.back());
            d.spare_.pop_back();
        }
        f.width = width;
        f.height = height;
        int cw = f.chroma_width(), ch = f.chroma_height();
        f.y.resize(size_t(width) * height);
        f.u.resize(size_t(cw) * ch);
        f.v.resize(size_t(cw) * ch);
        for (int r = 0; r < height; ++r) std::memcpy(&f.y[size_t(r) * width], &pic.y[size_t(r) * pic.w], width);
        for (int r = 0; r < ch; ++r) {
            std::memcpy(&f.u[size_t(r) * cw], &pic.u[size_t(r) * (pic.w / 2)], cw);
            std::memcpy(&f.v[size_t(r) * cw], &pic.v[size_t(r) * (pic.w / 2)], cw);
        }
        s64 period = d.seq_.frame_rate > 0 ? s64(90000.0 / d.seq_.frame_rate + 0.5) : 3003;
        if (pic.pts >= 0) f.pts = pic.pts;
        else f.pts = last_out_pts >= 0 ? last_out_pts + period : 0;
        last_out_pts = f.pts;
        f.type = pic.type;
        f.index = out_index++;
        d.out_.push_back(std::move(f));
        ++d.stats_.output;
    }

    // ======================================================================
    int quant_scale(u32 code) const {
        if (code == 0) throw FormatError("MPEG-2: zero quantiser_scale_code");
        return q_scale_type ? kNonLinearQuantScale[code] : int(code) * 2;
    }

    void reset_dc() { dc_pred[0] = dc_pred[1] = dc_pred[2] = 1 << (dc_precision + 7); }
    void reset_pmv() { std::memset(pmv, 0, sizeof(pmv)); }

    static int read_vlc(BitReader& br, const VlcTable& t) {
        const VlcEntry& e = t.lut[br.peek(t.bits)];
        if (!e.len) throw FormatError("MPEG-2: invalid VLC");
        br.skip(e.len);
        return e.value;
    }

    void decode_slice(int code, const u8* p, size_t len) {
        BitReader br(p, len);
        int mb_y = code - 1;
        if (height > 2800) mb_y += int(br.get(3)) << 7;
        if (mb_y >= mb_h) throw FormatError("MPEG-2: slice below picture");
        qscale = quant_scale(br.get(5));
        if (br.peek(1)) { // intra_slice_flag, intra_slice, reserved, extra information
            br.skip(9);
            while (br.get1()) br.skip(8);
        } else {
            br.skip(1);
        }
        reset_dc();
        reset_pmv();
        last_flags = 0;
        int mb_x = -1;
        bool first = true;
        const VlcTable& mba = vlc_mb_address_increment();
        for (;;) {
            int inc = 0;
            for (;;) {
                int v = read_vlc(br, mba);
                if (v == kMbaEscape) inc += 33;
                else if (v != kMbaStuffing) {
                    inc += v;
                    break;
                }
                if (br.exhausted()) throw FormatError("MPEG-2: slice overrun");
            }
            if (first) {
                mb_x = inc - 1;
                first = false;
            } else {
                for (int k = 1; k < inc; ++k)
                    if (mb_x + k < mb_w) skipped_macroblock(mb_x + k, mb_y);
                mb_x += inc;
            }
            if (mb_x >= mb_w) throw FormatError("MPEG-2: macroblock past row end");
            macroblock(br, mb_x, mb_y);
            if (br.exhausted()) {
                if (br.bits_left() < 0) throw FormatError("MPEG-2: slice overrun");
                break;
            }
            if (br.peek(23) == 0) break; // next start code / stuffing
        }
    }

    // Reference used for forward prediction (P: the newest reference).
    const Picture& forward_ref() const { return pics[pic_type == kP ? bwd : (fwd >= 0 ? fwd : bwd)]; }
    const Picture& backward_ref() const { return pics[bwd]; }

    void skipped_macroblock(int mx, int my) {
        reset_dc();
        int mv[2][2] = {};
        int flags;
        if (pic_type == kP) {
            reset_pmv();
            flags = kMbMotionForward;
        } else if (pic_type == kB) {
            flags = last_flags & (kMbMotionForward | kMbMotionBackward);
            if (!flags) throw FormatError("MPEG-2: skipped B macroblock after intra");
            std::memcpy(mv, last_mv, sizeof(mv));
        } else {
            throw FormatError("MPEG-2: skipped macroblock in I picture");
        }
        predict(mx, my, flags, mv);
    }

    void predict(int mx, int my, int flags, const int mv[2][2]) {
        Picture& dst = pics[cur];
        bool avg = false;
        for (int s = 0; s < 2; ++s) {
            if (!(flags & (s == 0 ? kMbMotionForward : kMbMotionBackward))) continue;
            const Picture& ref = s == 0 ? forward_ref() : backward_ref();
            int vx = mv[s][0], vy = mv[s][1];
            predict_plane(ref.y.data(), dst.y.data(), dst.w, dst.h, mx * 16, my * 16, vx, vy, 16, 16, avg);
            int cvx = vx / 2, cvy = vy / 2; // 4:2:0 chroma vectors, truncated toward zero
            predict_plane(ref.u.data(), dst.u.data(), dst.w / 2, dst.h / 2, mx * 8, my * 8, cvx, cvy, 8, 8, avg);
            predict_plane(ref.v.data(), dst.v.data(), dst.w / 2, dst.h / 2, mx * 8, my * 8, cvx, cvy, 8, 8, avg);
            avg = true;
        }
    }

    // Frame motion vector for direction s; updates the predictors.
    void motion_vector(BitReader& br, int s, int out[2]) {
        for (int t = 0; t < 2; ++t) {
            int code = read_vlc(br, vlc_motion_code());
            int r_size = f_code[s][t] - 1;
            if (r_size < 0 || r_size > 8) throw FormatError("MPEG-2: bad f_code");
            int delta = 0;
            if (code != 0) {
                int residual = r_size ? int(br.get(r_size)) : 0;
                delta = ((std::abs(code) - 1) << r_size) + residual + 1;
                if (code < 0) delta = -delta;
            }
            int range = 32 << r_size, low = -(16 << r_size), high = (16 << r_size) - 1;
            int v = pmv[s][t] + delta;
            if (v < low) v += range;
            else if (v > high) v -= range;
            pmv[s][t] = v;
            out[t] = v;
        }
    }

    // Destination of block b (0-3 luma, 4 Cb, 5 Cr) of macroblock (mx, my) in
    // the current picture, for frame or field DCT.
    void block_dest(int b, int mx, int my, bool field_dct, u8*& ptr, int& stride) {
        Picture& dst = pics[cur];
        if (b < 4) {
            int x = mx * 16 + (b & 1) * 8;
            int y = field_dct ? my * 16 + (b >> 1) : my * 16 + (b >> 1) * 8;
            stride = field_dct ? dst.w * 2 : dst.w;
            ptr = dst.y.data() + size_t(y) * dst.w + x;
        } else {
            stride = dst.w / 2;
            ptr = (b == 4 ? dst.u.data() : dst.v.data()) + size_t(my) * 8 * stride + mx * 8;
        }
    }

    // The six blocks of an intra macroblock, written to the current picture.
    void intra_blocks(BitReader& br, int mx, int my, bool field_dct) {
        for (int b = 0; b < 6; ++b) {
            intra_block(br, b < 4 ? 0 : b - 3);
            idct_8x8(blk);
            u8* ptr;
            int stride;
            block_dest(b, mx, my, field_dct, ptr, stride);
            for (int r = 0; r < 8; ++r, ptr += stride)
                for (int c = 0; c < 8; ++c) ptr[c] = clamp255(blk[r * 8 + c]);
        }
    }

    // PS2 IPU picture (see ipu.h): MPEG-2 intra macroblocks without slices.
    void ipu_picture(const u8* p, size_t len, int w, int h) {
        BitReader br(p, len);
        int flags = int(br.get(8));
        if (flags & 0x80) throw FormatError("IPU: MPEG-1 style blocks are not supported");
        set_size(w, h);
        fwd = bwd = -1;
        cur = 0;
        pics[cur].pts = 0;
        pics[cur].type = 'I';
        dc_precision = flags & 3;
        q_scale_type = flags & 0x40;
        intra_vlc = flags & 0x20;
        alternate_scan = flags & 0x10;
        bool has_dct_type = flags & 4;
        std::memcpy(intra_q, kDefaultIntraMatrix, 64);
        reset_dc();
        qscale = 1;
        for (int my = 0; my < mb_h; ++my) {
            for (int mx = 0; mx < mb_w; ++mx) {
                if ((mx || my) && !br.get1()) throw FormatError("IPU: bad macroblock address increment");
                bool quant = false;
                if (!br.get1()) {
                    if (!br.get1()) throw FormatError("IPU: bad macroblock type");
                    quant = true;
                }
                bool field_dct = has_dct_type && br.get1();
                if (quant) qscale = quant_scale(br.get(5));
                intra_blocks(br, mx, my, field_dct);
                if (br.bits_left() < 0) throw FormatError("IPU: picture data truncated");
            }
        }
        emit(cur);
    }

    void macroblock(BitReader& br, int mx, int my) {
        const VlcTable& types = pic_type == kI ? vlc_mb_type_i() : pic_type == kP ? vlc_mb_type_p() : vlc_mb_type_b();
        int flags = read_vlc(br, types);
        int motion_type = 2; // frame
        if ((flags & (kMbMotionForward | kMbMotionBackward)) && !frame_pred_frame_dct) motion_type = int(br.get(2));
        bool field_dct = false;
        if (!frame_pred_frame_dct && (flags & (kMbIntra | kMbPattern))) field_dct = br.get1();
        if (flags & kMbQuant) qscale = quant_scale(br.get(5));

        if (flags & kMbIntra) {
            if (concealment) {
                int tmp[2];
                motion_vector(br, 0, tmp);
                br.skip(1); // marker
            } else {
                reset_pmv();
            }
            intra_blocks(br, mx, my, field_dct);
            last_flags = kMbIntra;
            return;
        }

        reset_dc();
        if (motion_type != 2) throw FormatError("MPEG-2: field/dual-prime prediction not supported");
        int mv[2][2] = {};
        if (flags & kMbMotionForward) motion_vector(br, 0, mv[0]);
        if (flags & kMbMotionBackward) motion_vector(br, 1, mv[1]);
        if (pic_type == kP && !(flags & kMbMotionForward)) {
            // P macroblock without motion compensation: zero vector, predictors reset.
            flags |= kMbMotionForward;
            reset_pmv();
        }
        predict(mx, my, flags, mv);
        last_flags = flags;
        std::memcpy(last_mv, mv, sizeof(mv));

        if (flags & kMbPattern) {
            int cbp = read_vlc(br, vlc_coded_block_pattern());
            for (int b = 0; b < 6; ++b) {
                if (!(cbp & (32 >> b))) continue;
                nonintra_block(br);
                idct_8x8(blk);
                u8* ptr;
                int stride;
                block_dest(b, mx, my, field_dct, ptr, stride);
                for (int r = 0; r < 8; ++r, ptr += stride)
                    for (int c = 0; c < 8; ++c) ptr[c] = clamp255(ptr[c] + blk[r * 8 + c]);
            }
        }
    }

    static inline int saturate(int v) { return v < -2048 ? -2048 : v > 2047 ? 2047 : v; }

    // Reads one run/level pair. Returns false at end of block.
    static inline bool coefficient(BitReader& br, const DctTable& t, int& run, int& level) {
        const DctEntry& e = t.lookup(br.peek(16));
        if (!e.len) throw FormatError("MPEG-2: invalid DCT code");
        if (e.run == kDctEob) {
            br.skip(e.len);
            return false;
        }
        if (e.run == kDctEscape) {
            br.skip(6);
            run = int(br.get(6));
            level = int(br.get(12));
            if (level >= 2048) level -= 4096;
            if ((level & 2047) == 0) throw FormatError("MPEG-2: invalid escape level");
            return true;
        }
        br.skip(e.len);
        run = e.run;
        level = br.get1() ? -int(e.level) : int(e.level);
        return true;
    }

    void intra_block(BitReader& br, int cc) {
        std::memset(blk, 0, sizeof(blk));
        int size = read_vlc(br, cc == 0 ? vlc_dc_size_luma() : vlc_dc_size_chroma());
        int diff = 0;
        if (size) {
            int bits = int(br.get(size));
            diff = bits < (1 << (size - 1)) ? bits - (1 << size) + 1 : bits;
        }
        dc_pred[cc] += diff;
        int sum = dc_pred[cc] << (3 - dc_precision);
        blk[0] = s16(sum);
        const DctTable& t = intra_vlc ? dct_table_one() : dct_table_zero();
        const u8* scan_order = alternate_scan ? kAlternateScan : kZigzagScan;
        int run, level;
        for (int i = 1;; ++i) {
            if (!coefficient(br, t, run, level)) break;
            i += run;
            if (i > 63) throw FormatError("MPEG-2: coefficient index overflow");
            int pos = scan_order[i];
            int f = saturate(level * 2 * intra_q[pos] * qscale / 32);
            blk[pos] = s16(f);
            sum += f;
        }
        if ((sum & 1) == 0) blk[63] ^= 1; // mismatch control
    }

    void nonintra_block(BitReader& br) {
        std::memset(blk, 0, sizeof(blk));
        const DctTable& t = dct_table_zero();
        const u8* scan_order = alternate_scan ? kAlternateScan : kZigzagScan;
        int sum = 0, run, level;
        for (int i = 0;; ++i) {
            if (i == 0 && br.peek(1)) { // first coefficient: "1s" = run 0, level +-1
                br.skip(1);
                run = 0;
                level = br.get1() ? -1 : 1;
            } else if (!coefficient(br, t, run, level)) {
                break;
            }
            i += run;
            if (i > 63) throw FormatError("MPEG-2: coefficient index overflow");
            int pos = scan_order[i];
            int k = level > 0 ? 1 : -1;
            int f = saturate((2 * level + k) * nonintra_q[pos] * qscale / 32);
            blk[pos] = s16(f);
            sum += f;
        }
        if ((sum & 1) == 0) blk[63] ^= 1;
    }
};
#ifdef _MSC_VER
#pragma warning(pop)
#endif

Mpeg2Decoder::Mpeg2Decoder() : impl_(std::make_unique<Impl>(*this)) {}
Mpeg2Decoder::~Mpeg2Decoder() = default;

void Mpeg2Decoder::push(const u8* data, size_t len, s64 pts) { impl_->push(data, len, pts); }

void Mpeg2Decoder::flush() {
    impl_->process(true);
    impl_->finish_picture();
    impl_->emit_pending_reference();
}

void Mpeg2Decoder::reset() {
    Impl& m = *impl_;
    m.es.clear();
    m.es_base = 0;
    m.scan = 0;
    m.unit = kNone;
    m.marks.clear();
    m.in_picture = false;
    m.fwd = m.bwd = -1;
    m.bwd_emitted = true;
    m.drop_leading_b = false;
    m.last_out_pts = -1;
    m.out_index = 0;
    for (Frame& f : out_) spare_.push_back(std::move(f));
    out_.clear();
}

void Mpeg2Decoder::decode_ipu_picture(const u8* data, size_t len, int width, int height) {
    Bytes buf(data, data + len);
    buf.resize(len + kUnitPad, 0);
    impl_->ipu_picture(buf.data(), len, width, height);
}

bool Mpeg2Decoder::pop(Frame& out) {
    if (out_.empty()) return false;
    if (!out.y.empty() && spare_.size() < 4) spare_.push_back(std::move(out));
    out = std::move(out_.front());
    out_.pop_front();
    return true;
}

} // namespace gt3r::video
