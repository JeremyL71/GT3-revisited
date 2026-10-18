// Movies: MPEG-2 tables, IDCT accuracy, a hand-encoded MPEG-2 stream through the decoder, synthetic PSS
// demux/Movie tests, and (with GT3R_DISC) the real FMVs under stream/.

#include "test_framework.h"

#include "formats/ads.h"
#include "formats/pss.h"
#include "video/field_layout.h"
#include "video/idct.h"
#include "video/ipu.h"
#include "video/movie.h"
#include "video/mpeg2_decoder.h"
#include "video/mpeg2_tables.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <numbers>
#include <random>
#include <set>
#include <string>

using namespace gt3r;
using namespace gt3r::video;

namespace {

// --- bit string helpers --------------------------------------------------------------------------

// "0100 1" -> value left-aligned in `width` bits (spaces ignored, rest zero).
u32 align_bits(const char* s, int width) {
    u32 v = 0;
    int n = 0;
    for (; *s; ++s) {
        if (*s == ' ') continue;
        if (n < width) v |= u32(*s == '1') << (width - 1 - n);
        ++n;
    }
    return v;
}

const VlcEntry& vlc_at(const VlcTable& t, const char* bits) { return t.lut[align_bits(bits, t.bits)]; }
const DctEntry& dct_at(const DctTable& t, const char* bits) { return t.lookup(align_bits(bits, 16)); }

// MSB-first bit writer.
struct BitWriter {
    Bytes b;
    int used = 0; // bits used in the last byte (0 = byte aligned)

    void bit(int v) {
        if (used == 0) b.push_back(0);
        if (v) b.back() |= u8(0x80 >> used);
        used = (used + 1) & 7;
    }
    void put(u32 v, int n) {
        for (int i = n - 1; i >= 0; --i) bit(int((v >> i) & 1));
    }
    void str(const char* s) {
        for (; *s; ++s)
            if (*s != ' ') bit(*s == '1');
    }
    void align() { used = 0; }
    void start_code(u8 code) {
        align();
        b.push_back(0);
        b.push_back(0);
        b.push_back(1);
        b.push_back(code);
    }
};

// --- hand-encoded MPEG-2 stream ------------------------------------------------------------------

constexpr int kW = 48, kH = 32, kMbW = 3, kMbH = 2;

struct Yuv {
    std::vector<u8> y, u, v;
};

int mb_luma(int i) { return 60 + 10 * i; }
int mb_cb(int i) { return 100 + 7 * i; }
int mb_cr(int i) { return 200 - 9 * i; }

Yuv expected_i_frame() {
    Yuv f;
    f.y.resize(kW * kH);
    f.u.resize(kW / 2 * kH / 2);
    f.v.resize(kW / 2 * kH / 2);
    for (int my = 0; my < kMbH; ++my)
        for (int mx = 0; mx < kMbW; ++mx) {
            int i = my * kMbW + mx;
            for (int r = 0; r < 16; ++r)
                for (int c = 0; c < 16; ++c) f.y[size_t(my * 16 + r) * kW + mx * 16 + c] = u8(mb_luma(i));
            for (int r = 0; r < 8; ++r)
                for (int c = 0; c < 8; ++c) {
                    f.u[size_t(my * 8 + r) * (kW / 2) + mx * 8 + c] = u8(mb_cb(i));
                    f.v[size_t(my * 8 + r) * (kW / 2) + mx * 8 + c] = u8(mb_cr(i));
                }
        }
    return f;
}

void put_dc_diff(BitWriter& w, int diff, bool luma) {
    static const char* kLuma[] = {"100", "00", "01", "101", "110", "1110", "11110", "111110", "1111110"};
    static const char* kChroma[] = {"00", "01", "10", "110", "1110", "11110", "111110", "1111110", "11111110"};
    int mag = std::abs(diff), size = 0;
    while ((1 << size) <= mag) ++size;
    w.str(luma ? kLuma[size] : kChroma[size]);
    if (size) w.put(u32(diff > 0 ? diff : diff + (1 << size) - 1), size);
}

// One intra macroblock whose six blocks are DC-only (flat) with the given values.
void put_intra_mb(BitWriter& w, int y, int cb, int cr, int pred[3]) {
    w.str("1"); // macroblock_type: intra
    for (int b = 0; b < 4; ++b) {
        put_dc_diff(w, y - pred[0], true);
        pred[0] = y;
        w.str("10"); // end of block
    }
    put_dc_diff(w, cb - pred[1], false);
    pred[1] = cb;
    w.str("10");
    put_dc_diff(w, cr - pred[2], false);
    pred[2] = cr;
    w.str("10");
}

Bytes sequence_prefix() {
    BitWriter w;
    w.start_code(0xB3);
    w.put(kW, 12);
    w.put(kH, 12);
    w.put(1, 4);       // aspect ratio
    w.put(5, 4);       // 30 fps
    w.put(1000, 18);   // bit rate
    w.put(1, 1);       // marker
    w.put(20, 10);     // vbv buffer size
    w.put(0, 1);       // constrained parameters
    w.put(0, 1);       // load_intra_quantiser_matrix
    w.put(0, 1);       // load_non_intra_quantiser_matrix
    w.start_code(0xB5); // sequence extension
    w.put(1, 4);
    w.put(0x48, 8);    // profile and level: MP@ML
    w.put(1, 1);       // progressive_sequence
    w.put(1, 2);       // chroma_format 4:2:0
    w.put(0, 2);       // horizontal_size_extension
    w.put(0, 2);       // vertical_size_extension
    w.put(0, 12);      // bit_rate_extension
    w.put(1, 1);       // marker
    w.put(0, 8);       // vbv_buffer_size_extension
    w.put(0, 1);       // low_delay
    w.put(0, 2);       // frame_rate_extension_n
    w.put(0, 5);       // frame_rate_extension_d
    w.start_code(0xB8); // group of pictures
    w.put(0, 25);      // time code
    w.put(1, 1);       // closed_gop
    w.put(0, 1);       // broken_link
    w.align();
    return w.b;
}

// type: 1 = I, 2 = P, 3 = B. f_code is used for the directions the picture type has.
void put_picture_header(BitWriter& w, int tref, int type) {
    w.start_code(0x00);
    w.put(u32(tref), 10);
    w.put(u32(type), 3);
    w.put(0xFFFF, 16); // vbv_delay
    if (type >= 2) {
        w.put(0, 1);
        w.put(7, 3); // forward_f_code (MPEG-1 field, 7 in MPEG-2)
    }
    if (type == 3) {
        w.put(0, 1);
        w.put(7, 3);
    }
    w.put(0, 1); // extra_bit_picture
    w.start_code(0xB5);
    w.put(8, 4); // picture coding extension
    const int f = 1, none = 15;
    int fwd = type >= 2 ? f : none, bwd = type == 3 ? f : none;
    w.put(u32(fwd), 4);
    w.put(u32(fwd), 4);
    w.put(u32(bwd), 4);
    w.put(u32(bwd), 4);
    w.put(0, 2); // intra_dc_precision
    w.put(3, 2); // picture_structure: frame
    w.put(0, 1); // top_field_first
    w.put(1, 1); // frame_pred_frame_dct
    w.put(0, 1); // concealment_motion_vectors
    w.put(0, 1); // q_scale_type
    w.put(0, 1); // intra_vlc_format
    w.put(0, 1); // alternate_scan
    w.put(0, 1); // repeat_first_field
    w.put(1, 1); // chroma_420_type
    w.put(1, 1); // progressive_frame
    w.put(0, 1); // composite_display_flag
}

void put_slice_header(BitWriter& w, int row) {
    w.start_code(u8(row + 1));
    w.put(5, 5); // quantiser_scale_code
    w.put(0, 1); // extra_bit_slice
}

Bytes build_i_picture(int tref) {
    BitWriter w;
    put_picture_header(w, tref, 1);
    for (int row = 0; row < kMbH; ++row) {
        put_slice_header(w, row);
        int pred[3] = {128, 128, 128};
        for (int mx = 0; mx < kMbW; ++mx) {
            int i = row * kMbW + mx;
            w.str("1"); // macroblock_address_increment 1
            put_intra_mb(w, mb_luma(i), mb_cb(i), mb_cr(i), pred);
        }
    }
    w.align();
    return w.b;
}

// Zero-vector forward-predicted picture: first and last macroblock of each row coded as "motion forward, no
// residual", the ones between skipped.
Bytes build_p_picture(int tref) {
    BitWriter w;
    put_picture_header(w, tref, 2);
    for (int row = 0; row < kMbH; ++row) {
        put_slice_header(w, row);
        w.str("1");   // address increment 1
        w.str("001"); // motion forward, not coded
        w.str("1 1"); // motion_code 0 horizontal, 0 vertical
        w.str("011"); // address increment 2 (skips the middle macroblock)
        w.str("001");
        w.str("1 1");
    }
    w.align();
    return w.b;
}

// Same, but every macroblock carries an explicit horizontal half-pel vector (+1, +1, then -1 back to 0 so the last
// macroblock does not reference outside the picture).
Bytes build_p_picture_halfpel(int tref) {
    BitWriter w;
    put_picture_header(w, tref, 2);
    for (int row = 0; row < kMbH; ++row) {
        put_slice_header(w, row);
        w.str("1");
        w.str("001");
        w.str("010 1"); // dx = +1 half-pel (motion_code 1, positive), dy = 0
        w.str("1");
        w.str("001");
        w.str("1 1");   // delta 0: stays +1
        w.str("1");
        w.str("001");
        w.str("011 1"); // delta -1: back to 0
    }
    w.align();
    return w.b;
}

// Bidirectional zero-vector picture ("interpolated, not coded"), skipped macroblocks in between.
Bytes build_b_picture(int tref) {
    BitWriter w;
    put_picture_header(w, tref, 3);
    for (int row = 0; row < kMbH; ++row) {
        put_slice_header(w, row);
        w.str("1");
        w.str("10");  // forward + backward, no residual
        w.str("1 1"); // forward vector 0,0
        w.str("1 1"); // backward vector 0,0
        w.str("011"); // skip one
        w.str("10");
        w.str("1 1");
        w.str("1 1");
    }
    w.align();
    return w.b;
}

Bytes sequence_end() {
    BitWriter w;
    w.start_code(0xB7);
    return w.b;
}

struct Piece {
    Bytes data;
    s64 pts;
};

// Feeds the pieces into the decoder, each split into random-size chunks; the timestamp goes with the first chunk.
void feed(Mpeg2Decoder& dec, const std::vector<Piece>& pieces, unsigned seed) {
    std::mt19937 rng(seed);
    for (const Piece& p : pieces) {
        size_t pos = 0;
        bool first = true;
        while (pos < p.data.size()) {
            size_t n = std::min<size_t>(1 + rng() % 37, p.data.size() - pos);
            dec.push(p.data.data() + pos, n, first ? p.pts : -1);
            first = false;
            pos += n;
        }
    }
    dec.flush();
}

bool planes_equal(const Frame& f, const Yuv& e) { return f.y == e.y && f.u == e.u && f.v == e.v; }

constexpr s64 kPtsI = 100000, kPtsB = 103003, kPtsP = 106006;

std::vector<Piece> synthetic_pieces() {
    std::vector<Piece> v;
    v.push_back({sequence_prefix(), -1});
    v.push_back({build_i_picture(0), kPtsI});
    v.push_back({build_p_picture(2), kPtsP});
    v.push_back({build_b_picture(1), kPtsB});
    v.push_back({sequence_end(), -1});
    return v;
}

// --- program stream building ---------------------------------------------------------------------

void put_ts(Bytes& out, int prefix, s64 ts) {
    out.push_back(u8(prefix << 4 | ((ts >> 30) & 7) << 1 | 1));
    out.push_back(u8((ts >> 22) & 0xFF));
    out.push_back(u8(((ts >> 15) & 0x7F) << 1 | 1));
    out.push_back(u8((ts >> 7) & 0xFF));
    out.push_back(u8((ts & 0x7F) << 1 | 1));
}

void append(Bytes& out, const Bytes& more) { out.insert(out.end(), more.begin(), more.end()); }

Bytes pack_header(int stuffing = 0) {
    Bytes b = {0, 0, 1, 0xBA, 0x44, 0x00, 0x04, 0x00, 0x04, 0x01, 0x01, 0x89, 0xC3, u8(0xF8 | stuffing)};
    for (int i = 0; i < stuffing; ++i) b.push_back(0xFF);
    return b;
}

// MPEG-2 PES packet. pts/dts < 0 = absent. `sub` = private stream 1 sub-header FF A1 00 00 before the payload.
Bytes pes_packet(u8 id, const Bytes& payload, s64 pts = -1, s64 dts = -1, bool sub = false) {
    Bytes hdr;
    u8 flags = 0;
    if (pts >= 0 && dts >= 0) {
        flags = 0xC0;
        put_ts(hdr, 3, pts);
        put_ts(hdr, 1, dts);
    } else if (pts >= 0) {
        flags = 0x80;
        put_ts(hdr, 2, pts);
    }
    Bytes body = {0x80, flags, u8(hdr.size())};
    append(body, hdr);
    if (sub) append(body, Bytes{0xFF, 0xA1, 0x00, 0x00});
    append(body, payload);
    Bytes out = {0, 0, 1, id, u8(body.size() >> 8), u8(body.size() & 0xFF)};
    append(out, body);
    return out;
}

Bytes counting_bytes(size_t n, u8 start) {
    Bytes b(n);
    for (size_t i = 0; i < n; ++i) b[i] = u8(start + i);
    return b;
}

PssDemuxer::ReadFn buffer_reader(const Bytes& buf) {
    return [&buf](u64 off, void* out, size_t len) { std::memcpy(out, buf.data() + off, len); };
}

// ADS file: header + body of `blocks` interleave blocks per channel; channel 0 decodes to 4096, channel 1 to 8192.
constexpr u32 kAdsInterleave = 0x40;
constexpr u32 kAdsBlocks = 3;

Bytes build_ads() {
    Bytes b;
    auto put32 = [&](u32 v) {
        for (int i = 0; i < 4; ++i) b.push_back(u8(v >> (8 * i)));
    };
    auto tag = [&](const char* t) { b.insert(b.end(), t, t + 4); };
    tag("SShd");
    put32(0x18);
    put32(0x10);
    put32(44100);
    put32(2);
    put32(kAdsInterleave);
    put32(0xFFFFFFFF);
    put32(0xFFFFFFFF);
    tag("SSbd");
    put32(kAdsInterleave * 2 * kAdsBlocks);
    for (u32 blk = 0; blk < kAdsBlocks; ++blk)
        for (u32 ch = 0; ch < 2; ++ch)
            for (u32 f = 0; f < kAdsInterleave / 16; ++f) {
                b.push_back(0x00); // filter 0, shift 0
                b.push_back(0x00);
                for (int i = 0; i < 14; ++i) b.push_back(u8((ch + 1) | (ch + 1) << 4));
            }
    return b;
}

// Volume access for the disc tests.
std::unique_ptr<Movie> open_disc_movie(Volume& vol, const Volume::Entry& entry) {
    Volume* v = &vol;
    Volume::Entry e = entry;
    return std::make_unique<Movie>([v, e](u64 off, void* out, size_t len) { v->read_range(e, off, out, len); },
                                   e.size);
}

} // namespace

// =================================================================================================
// Tables
// =================================================================================================

TEST(video_vlc_tables) {
    // Building each table throws std::logic_error on overlapping codes.
    const VlcTable* tables[] = {&vlc_mb_address_increment(), &vlc_mb_type_i(),     &vlc_mb_type_p(),
                                &vlc_mb_type_b(),            &vlc_coded_block_pattern(), &vlc_motion_code(),
                                &vlc_dc_size_luma(),         &vlc_dc_size_chroma()};
    for (const VlcTable* t : tables) {
        CHECK(t->bits > 0);
        CHECK(t->lut.size() == (size_t(1) << t->bits));
    }

    const VlcTable& mba = vlc_mb_address_increment();
    CHECK(vlc_at(mba, "1").value == 1 && vlc_at(mba, "1").len == 1);
    CHECK(vlc_at(mba, "011").value == 2 && vlc_at(mba, "011").len == 3);
    CHECK(vlc_at(mba, "010").value == 3);
    CHECK(vlc_at(mba, "0000 0011 000").value == 33);
    CHECK(vlc_at(mba, "0000 0001 000").value == kMbaEscape);
    CHECK(vlc_at(mba, "0000 0000 000").len == 0);

    const VlcTable& mv = vlc_motion_code();
    CHECK(vlc_at(mv, "1").value == 0 && vlc_at(mv, "1").len == 1);
    CHECK(vlc_at(mv, "010").value == 1 && vlc_at(mv, "010").len == 3);
    CHECK(vlc_at(mv, "011").value == -1 && vlc_at(mv, "011").len == 3);
    CHECK(vlc_at(mv, "0000 0011 000").value == 16);
    CHECK(vlc_at(mv, "0000 0011 001").value == -16);

    CHECK(vlc_at(vlc_mb_type_i(), "1").value == kMbIntra);
    CHECK(vlc_at(vlc_mb_type_p(), "001").value == kMbMotionForward);
    CHECK(vlc_at(vlc_mb_type_p(), "1").value == (kMbMotionForward | kMbPattern));
    CHECK(vlc_at(vlc_mb_type_b(), "10").value == (kMbMotionForward | kMbMotionBackward));
    CHECK(vlc_at(vlc_coded_block_pattern(), "111").value == 60);
    CHECK(vlc_at(vlc_coded_block_pattern(), "0000 0000 1").value == 0);
    CHECK(vlc_at(vlc_dc_size_luma(), "100").value == 0);
    CHECK(vlc_at(vlc_dc_size_luma(), "00").value == 1);
    CHECK(vlc_at(vlc_dc_size_luma(), "1111 1111 0").value == 10);
    CHECK(vlc_at(vlc_dc_size_chroma(), "00").value == 0);
    CHECK(vlc_at(vlc_dc_size_chroma(), "1111 1111 10").value == 10);

    // Table zero (B.14).
    const DctTable& t0 = dct_table_zero();
    CHECK(dct_at(t0, "110").run == 0 && dct_at(t0, "110").level == 1 && dct_at(t0, "110").len == 2);
    CHECK(dct_at(t0, "111").run == 0 && dct_at(t0, "111").level == 1 && dct_at(t0, "111").len == 2);
    CHECK(dct_at(t0, "10").run == kDctEob && dct_at(t0, "10").len == 2);
    CHECK(dct_at(t0, "0000 01").run == kDctEscape && dct_at(t0, "0000 01").len == 6);
    CHECK(dct_at(t0, "0100").run == 0 && dct_at(t0, "0100").level == 2 && dct_at(t0, "0100").len == 4);
    CHECK(dct_at(t0, "011").run == 1 && dct_at(t0, "011").level == 1 && dct_at(t0, "011").len == 3);
    CHECK(dct_at(t0, "0010 1").run == 0 && dct_at(t0, "0010 1").level == 3);
    CHECK(dct_at(t0, "0000 0000 0000 0000").len == 0);
    // Long codes: 0/40 is 15 bits, 1/15 is 16 bits.
    CHECK(dct_at(t0, "0000 0000 0010 000").run == 0 && dct_at(t0, "0000 0000 0010 000").level == 40);
    CHECK(dct_at(t0, "0000 0000 0010 000").len == 15);
    CHECK(dct_at(t0, "0000 0000 0001 0011").run == 1 && dct_at(t0, "0000 0000 0001 0011").level == 15);
    CHECK(dct_at(t0, "0000 0000 0001 0011").len == 16);

    // Table one (B.15).
    const DctTable& t1 = dct_table_one();
    CHECK(dct_at(t1, "0110").run == kDctEob && dct_at(t1, "0110").len == 4);
    CHECK(dct_at(t1, "100").run == 0 && dct_at(t1, "100").level == 1 && dct_at(t1, "100").len == 2);
    CHECK(dct_at(t1, "0000 01").run == kDctEscape);

    // Both tables hold all 111 distinct run/level pairs plus one EOB and one escape.
    for (const DctTable* t : {&t0, &t1}) {
        std::set<int> pairs;
        int eob = 0, esc = 0;
        auto scan = [&](const DctEntry* e) {
            for (int i = 0; i < 1024; ++i) {
                if (!e[i].len) continue;
                if (e[i].run == kDctEob) ++eob;
                else if (e[i].run == kDctEscape) ++esc;
                else pairs.insert(e[i].run * 256 + e[i].level);
            }
        };
        scan(t->short_codes);
        scan(t->long_codes);
        CHECK(pairs.size() == 111);
        CHECK(eob > 0 && esc > 0);
        // Every code of at most 16 bits decodes consistently: valid entries have 2..16 bit lengths.
        for (u32 pk = 0; pk < 65536; pk += 7) {
            const DctEntry& e = t->lookup(pk);
            CHECK(e.len == 0 || (e.len >= 2 && e.len <= 16));
        }
    }

    // Scan orders are permutations.
    for (const u8* scan : {kZigzagScan, kAlternateScan}) {
        std::set<int> seen(scan, scan + 64);
        CHECK(seen.size() == 64);
    }
}

// =================================================================================================
// IDCT accuracy (IEEE 1180-1990)
// =================================================================================================

namespace {

struct IdctStats {
    double peak = 0, mse = 0, mean = 0, worst_pixel_mse = 0, worst_pixel_mean = 0;
};

IdctStats run_idct_1180(int low, int high, bool negate, u32 seed, int blocks) {
    // Forward DCT matrix.
    double cs[8][8];
    for (int u = 0; u < 8; ++u)
        for (int x = 0; x < 8; ++x)
            cs[u][x] = (u == 0 ? std::sqrt(0.125) : 0.5) * std::cos((2 * x + 1) * u * std::numbers::pi / 16.0);

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(-low, high);
    double sum[64] = {}, sumsq[64] = {}, peak = 0;
    for (int n = 0; n < blocks; ++n) {
        double px[64], tmp[64];
        for (double& v : px) v = negate ? -dist(rng) : dist(rng);
        // Separable forward DCT in double.
        for (int y = 0; y < 8; ++y)
            for (int u = 0; u < 8; ++u) {
                double s = 0;
                for (int x = 0; x < 8; ++x) s += cs[u][x] * px[y * 8 + x];
                tmp[y * 8 + u] = s;
            }
        s16 in[64];
        for (int v = 0; v < 8; ++v)
            for (int u = 0; u < 8; ++u) {
                double s = 0;
                for (int y = 0; y < 8; ++y) s += cs[v][y] * tmp[y * 8 + u];
                in[v * 8 + u] = s16(std::clamp(int(std::floor(s + 0.5)), -2048, 2047));
            }
        double ref[64];
        idct_8x8_reference(in, ref);
        s16 blk[64];
        std::memcpy(blk, in, sizeof(blk));
        idct_8x8(blk);
        for (int i = 0; i < 64; ++i) {
            int r = std::clamp(int(std::floor(ref[i] + 0.5)), -256, 255);
            int o = std::clamp(int(blk[i]), -256, 255);
            double e = o - r;
            sum[i] += e;
            sumsq[i] += e * e;
            peak = std::max(peak, std::abs(e));
        }
    }
    IdctStats st;
    st.peak = peak;
    double tot = 0, totmean = 0;
    for (int i = 0; i < 64; ++i) {
        double mse = sumsq[i] / blocks, mean = sum[i] / blocks;
        st.worst_pixel_mse = std::max(st.worst_pixel_mse, mse);
        st.worst_pixel_mean = std::max(st.worst_pixel_mean, std::abs(mean));
        tot += mse;
        totmean += mean;
    }
    st.mse = tot / 64;
    st.mean = totmean / 64;
    return st;
}

} // namespace

TEST(video_idct_accuracy) {
    // All-zero input.
    {
        s16 blk[64] = {};
        idct_8x8(blk);
        bool zero = true;
        for (s16 v : blk) zero &= v == 0;
        CHECK(zero);
    }
    const int ranges[3][2] = {{256, 255}, {5, 5}, {300, 300}};
    u32 seed = 1;
    for (const auto& r : ranges)
        for (bool neg : {false, true}) {
            IdctStats s = run_idct_1180(r[0], r[1], neg, seed++, 10000);
            std::printf("  idct L=%d H=%d%s: peak %.0f, pixel MSE max %.5f, overall MSE %.5f, pixel |mean| max %.5f, "
                        "overall mean %+.6f\n",
                        r[0], r[1], neg ? " (negated)" : "", s.peak, s.worst_pixel_mse, s.mse, s.worst_pixel_mean,
                        s.mean);
            CHECK(s.peak <= 1.0);
            CHECK(s.worst_pixel_mse <= 0.06);
            CHECK(s.mse <= 0.02);
            CHECK(s.worst_pixel_mean <= 0.015);
            CHECK(std::abs(s.mean) <= 0.0015);
        }
}

// =================================================================================================
// Decoder on a hand-encoded stream
// =================================================================================================

TEST(video_mpeg2_synthetic) {
    for (unsigned seed : {1u, 2u, 3u}) {
        Mpeg2Decoder dec;
        feed(dec, synthetic_pieces(), seed);
        CHECK(dec.sequence().valid);
        CHECK(dec.sequence().width == kW && dec.sequence().height == kH);
        CHECK(dec.sequence().profile_level == 0x48);
        CHECK(dec.sequence().progressive);
        CHECK(std::abs(dec.sequence().frame_rate - 30.0) < 1e-9);
        CHECK(dec.stats().slice_errors == 0);
        CHECK(dec.stats().skipped_pictures == 0);
        CHECK(dec.stats().pictures == 3);
        CHECK(dec.stats().slices == 6);
        CHECK(dec.frames_ready() == 3);

        const Yuv expect = expected_i_frame();
        const char types[3] = {'I', 'B', 'P'};
        const s64 pts[3] = {kPtsI, kPtsB, kPtsP};
        Frame f;
        for (int i = 0; i < 3; ++i) {
            CHECK(dec.pop(f));
            CHECK(f.type == types[i]);
            CHECK(f.index == u32(i));
            CHECK(f.width == kW && f.height == kH);
            CHECK(f.y.size() == size_t(kW * kH) && f.u.size() == size_t(kW * kH / 4));
            CHECK(f.pts == pts[i]);
            CHECK(planes_equal(f, expect));
        }
        CHECK(!dec.pop(f));
    }
}

TEST(video_mpeg2_halfpel) {
    std::vector<Piece> pieces = {{sequence_prefix(), -1},
                                 {build_i_picture(0), kPtsI},
                                 {build_p_picture_halfpel(1), kPtsP},
                                 {sequence_end(), -1}};
    Mpeg2Decoder dec;
    feed(dec, pieces, 7);
    CHECK(dec.stats().slice_errors == 0);
    CHECK(dec.frames_ready() == 2);
    Frame fi, fp;
    CHECK(dec.pop(fi) && dec.pop(fp));
    CHECK(fi.type == 'I' && fp.type == 'P');

    // MB0 and MB1 use a +1 half-pel horizontal vector (average of a pixel and its right neighbour); MB2 uses zero.
    // Chroma vectors are halved and truncated to 0.
    Yuv e = expected_i_frame();
    Yuv p = e;
    for (int y = 0; y < kH; ++y)
        for (int x = 0; x < 32; ++x) p.y[size_t(y) * kW + x] = u8((e.y[size_t(y) * kW + x] + e.y[size_t(y) * kW + x + 1] + 1) >> 1);
    CHECK(planes_equal(fi, e));
    CHECK(planes_equal(fp, p));
}

// =================================================================================================
// PSS demuxer
// =================================================================================================

TEST(video_pss_demux_synthetic) {
    const s64 kPts1 = 0x123456789LL; // needs all 33 bits
    const s64 kPts2 = 0x1FFFFFFFFLL, kDts2 = 0x0AAAAAAAALL;
    const s64 kPtsA = 90000;
    Bytes v1 = counting_bytes(10, 0x11), v2 = counting_bytes(300, 0x40), a1 = counting_bytes(20, 0x80),
          v3 = counting_bytes(5, 0x01);
    v3[0] = 0x55; // keep clear of start-code lookalikes (payloads are length-skipped anyway)

    Bytes s;
    append(s, pack_header());
    append(s, Bytes{0, 0, 1, 0xBB, 0, 6, 0x80, 0x00, 0x01, 0x04, 0xE1, 0x7F}); // system header
    size_t off_v1 = s.size();
    append(s, pes_packet(0xE0, v1, kPts1));
    append(s, Bytes{0, 0, 1, 0xBE, 0, 4, 0xFF, 0xFF, 0xFF, 0xFF}); // padding
    const Bytes garbage = {0xDE, 0xAD, 0x00, 0x00, 0x02, 0xBE, 0xEF};
    append(s, garbage);
    append(s, pack_header(2));
    size_t off_v2 = s.size();
    append(s, pes_packet(0xE0, v2, kPts2, kDts2));
    size_t off_a = s.size();
    append(s, pes_packet(0xBD, a1, kPtsA, -1, true));
    append(s, pack_header());
    append(s, pes_packet(0xE0, v3));
    append(s, Bytes{0, 0, 1, 0xB9});

    for (int pass = 0; pass < 2; ++pass) {
        PssDemuxer d(buffer_reader(s), s.size());
        if (pass == 0) CHECK(PssDemuxer::probe(ByteSpan(s.data(), s.size())));
        PssPacket p;
        CHECK(d.next(p));
        CHECK(p.stream_id == 0xE0 && p.pts == kPts1 && p.dts == -1 && p.offset == off_v1);
        CHECK(p.size == v1.size() && std::memcmp(p.data, v1.data(), v1.size()) == 0);
        CHECK(d.resyncs() == 0);
        CHECK(d.next(p));
        CHECK(p.stream_id == 0xE0 && p.pts == kPts2 && p.dts == kDts2 && p.offset == off_v2);
        CHECK(p.size == v2.size() && std::memcmp(p.data, v2.data(), v2.size()) == 0);
        CHECK(d.resyncs() == garbage.size());
        CHECK(d.next(p));
        CHECK(p.stream_id == 0xBD && p.sub_id == 0xFF && p.pts == kPtsA && p.offset == off_a);
        CHECK(p.size == a1.size() && std::memcmp(p.data, a1.data(), a1.size()) == 0);
        CHECK(d.next(p));
        CHECK(p.stream_id == 0xE0 && p.pts == -1 && p.dts == -1);
        CHECK(p.size == v3.size() && std::memcmp(p.data, v3.data(), v3.size()) == 0);
        CHECK(!d.next(p));
        CHECK(!d.next(p));
        CHECK(d.resyncs() > 0);
        d.rewind();
    }
    // rewind() inside the loop restarts a fresh demuxer; also check it on one instance.
    PssDemuxer d(buffer_reader(s), s.size());
    PssPacket p;
    int n = 0;
    while (d.next(p)) ++n;
    CHECK(n == 4);
    d.rewind();
    CHECK(d.next(p) && p.pts == kPts1 && p.offset == off_v1);
    n = 1;
    while (d.next(p)) ++n;
    CHECK(n == 4);
}

// =================================================================================================
// Movie (demux + decode + audio) on a synthetic stream
// =================================================================================================

TEST(video_movie_synthetic) {
    std::vector<Piece> pieces = synthetic_pieces();
    // Video PES per picture; the sequence prefix rides with the I picture, the sequence end with the B picture.
    Bytes v0 = pieces[0].data;
    append(v0, pieces[1].data);
    Bytes v2 = pieces[3].data;
    append(v2, pieces[4].data);

    Bytes ads = build_ads();
    const size_t body = kAdsInterleave * 2 * kAdsBlocks;
    CHECK(ads.size() == 0x28 + body);
    // Audio split across three packets; the first holds the whole ADS header.
    Bytes a0(ads.begin(), ads.begin() + 0x28 + 100);
    Bytes a1(ads.begin() + 0x28 + 100, ads.begin() + 0x28 + 250);
    Bytes a2(ads.begin() + 0x28 + 250, ads.end());

    Bytes s;
    append(s, pack_header());
    append(s, pes_packet(0xE0, v0, kPtsI));
    append(s, pes_packet(0xBD, a0, 5000, -1, true));
    append(s, pack_header());
    append(s, pes_packet(0xE0, pieces[2].data, kPtsP));
    append(s, pes_packet(0xBD, a1, -1, -1, true));
    append(s, pack_header());
    append(s, pes_packet(0xE0, v2, kPtsB));
    append(s, pes_packet(0xBD, a2, -1, -1, true));
    append(s, Bytes{0, 0, 1, 0xB9});

    Movie m(buffer_reader(s), s.size());
    const MovieInfo& mi = m.info();
    CHECK(mi.width == kW && mi.height == kH);
    CHECK(std::abs(mi.frame_rate - 30.0) < 1e-9);
    CHECK(mi.start_pts == kPtsI);
    CHECK(mi.has_audio);
    CHECK(mi.audio_rate == 44100 && mi.audio_channels == 2);
    CHECK(mi.audio_start_pts == 5000);
    CHECK(mi.file_size == s.size());
    const u64 total = u64(body) / 2 / 16 * 28;
    CHECK(std::abs(mi.audio_seconds - double(total) / 44100.0) < 1e-9);

    const Yuv expect = expected_i_frame();
    const char types[3] = {'I', 'B', 'P'};
    Frame f;
    for (int i = 0; i < 3; ++i) {
        CHECK(m.next_frame(f));
        CHECK(f.type == types[i] && f.index == u32(i));
        CHECK(planes_equal(f, expect));
    }
    CHECK(!m.next_frame(f));
    CHECK(m.video_finished());
    CHECK(m.decoder().stats().slice_errors == 0);

    std::vector<s16> pcm(2 * 1000);
    size_t got = 0;
    for (;;) {
        size_t n = m.read_audio(pcm.data(), 100);
        if (!n) break;
        for (size_t i = 0; i < n; ++i) {
            CHECK(pcm[i * 2] == 4096);
            CHECK(pcm[i * 2 + 1] == 8192);
        }
        got += n;
    }
    CHECK(got == total);
    CHECK(m.read_audio(pcm.data(), 10) == 0);
}

// =================================================================================================
// Disc
// =================================================================================================

TEST(video_disc_menu_bg2) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Volume& vol = disc->volume();
    const Volume::Entry* e = vol.find("stream/menu_bg2.pss");
    if (!e) SKIP("stream/menu_bg2.pss not on this disc");
    auto m = open_disc_movie(vol, *e);
    CHECK(m->info().width == 320 && m->info().height == 112);
    CHECK(std::abs(m->info().frame_rate - 30.0) < 0.01);
    CHECK(!m->info().has_audio);

    int count = 0;
    s64 last_pts = -1;
    Frame f;
    double var100 = 0;
    while (m->next_frame(f)) {
        CHECK(f.type == 'I' || f.type == 'P' || f.type == 'B');
        CHECK(f.index == u32(count));
        CHECK(f.width == 320 && f.height == 112);
        CHECK(f.pts > last_pts);
        last_pts = f.pts;
        if (count == 100) {
            double sum = 0, sq = 0;
            for (u8 v : f.y) {
                sum += v;
                sq += double(v) * v;
            }
            double n = double(f.y.size());
            var100 = sq / n - (sum / n) * (sum / n);
        }
        ++count;
    }
    CHECK(count == 360);
    CHECK(var100 > 10.0);
    CHECK(m->decoder().stats().slice_errors == 0);
    CHECK(m->decoder().stats().skipped_pictures == 0);
}

TEST(video_disc_opening) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Volume& vol = disc->volume();
    const Volume::Entry* e = vol.find("stream/opening.pss");
    if (!e) SKIP("stream/opening.pss not on this disc");
    auto m = open_disc_movie(vol, *e);
    CHECK(m->info().width == 640 && m->info().height == 448);
    CHECK(m->info().has_audio);
    CHECK(m->info().audio_rate == 44100 && m->info().audio_channels == 2);

    Frame f;
    int count = 0;
    while (count < 60 && m->next_frame(f)) {
        CHECK(f.width == 640 && f.height == 448);
        ++count;
    }
    CHECK(count == 60);
    CHECK(m->decoder().stats().slice_errors == 0);

    std::vector<s16> pcm(44100 * 2);
    size_t n = m->read_audio(pcm.data(), 44100);
    CHECK(n == 44100);
    bool nonzero = false;
    for (size_t i = 0; i < n * 2; ++i) nonzero |= pcm[i] != 0;
    CHECK(nonzero);
}

TEST(video_disc_all_headers) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Volume& vol = disc->volume();
    int movies = 0;
    for (const Volume::Entry& e : vol.entries()) {
        if (e.path.rfind("stream/", 0) != 0 || e.path.size() < 4 || e.path.substr(e.path.size() - 4) != ".pss")
            continue;
        ++movies;
        std::unique_ptr<Movie> m;
        try {
            m = open_disc_movie(vol, e);
        } catch (const std::exception& ex) {
            std::printf("  %s: %s\n", e.path.c_str(), ex.what());
            CHECK(false);
            continue;
        }
        CHECK(m->info().width > 0 && m->info().height > 0);
        CHECK(m->info().frame_rate > 0);
        Frame f;
        int n = 0;
        while (n < 3 && m->next_frame(f)) ++n;
        if (n != 3 || m->decoder().stats().slice_errors != 0)
            std::printf("  %s: %d frames, %llu slice errors\n", e.path.c_str(), n,
                        (unsigned long long)m->decoder().stats().slice_errors);
        CHECK(n == 3);
        CHECK(m->decoder().stats().slice_errors == 0);
    }
    CHECK(movies > 0);
}

// ---------------------------------------------------------------------------
// IPU stills and field layout
// ---------------------------------------------------------------------------

TEST(video_ipua_synthetic) {
    // Archive header + 2 entries; offsets are only parsed, not decoded.
    Bytes d(0x100, 0);
    std::memcpy(d.data(), "ipua", 4);
    d[4] = 2;
    d[8] = 0x80;
    std::memcpy(&d[12], "first", 5);
    d[40] = 0xC0;
    std::memcpy(&d[44], "second", 6);
    auto e = parse_ipua(d);
    CHECK(e.size() == 2);
    CHECK(e[0].name == "first" && e[0].offset == 0x80 && e[0].size == 0x40);
    CHECK(e[1].name == "second" && e[1].offset == 0xC0 && e[1].size == 0x40);
    Bytes bad = d;
    bad[0] = 'x';
    bool threw = false;
    try {
        parse_ipua(bad);
    } catch (const FormatError&) {
        threw = true;
    }
    CHECK(threw);
}

TEST(video_field_layout_synthetic) {
    Frame f;
    f.width = 64;
    f.height = 64;
    f.y.assign(64 * 64, 0);
    f.u.assign(32 * 32, 128);
    f.v.assign(32 * 32, 128);
    // Progressive: vertical gradient, continuous at the seam.
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x) f.y[y * 64 + x] = u8(40 + y * 2 + (x & 3));
    CHECK(seam_ratio(f) < 2.0);
    // Field-split: the same half-height gradient twice -> jump at the seam.
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x) f.y[y * 64 + x] = u8(40 + (y % 32) * 4 + (x & 3));
    CHECK(seam_ratio(f) > FieldLayoutDetector::kSeamThreshold);
    FieldLayoutDetector det;
    for (int i = 0; i < FieldLayoutDetector::kVotes; ++i) det.add(f);
    CHECK(det.decided() && det.field_split());
    Frame w = weave_fields(f);
    CHECK(w.y[0] == f.y[0] && w.y[64] == f.y[32 * 64] && w.y[2 * 64] == f.y[64]);
}

TEST(video_disc_course_ipic) {
    Disc* disc = gt3r::test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Volume& vol = disc->volume();
    const Volume::Entry* e = vol.find("stream/course.ipic");
    CHECK(e != nullptr);
    if (!e) return;
    Bytes data = vol.read(*e);
    auto entries = parse_ipua(data);
    CHECK(entries.size() == 42);
    int ok = 0;
    for (const auto& en : entries) {
        Frame f = decode_ipum(ByteSpan(data).subspan(en.offset, en.size));
        double mean = 0, sq = 0;
        for (u8 v : f.y) {
            mean += v;
            sq += double(v) * v;
        }
        mean /= double(f.y.size());
        double var = sq / double(f.y.size()) - mean * mean;
        if (f.width == 640 && f.height == 448 && var > 10) ++ok;
    }
    CHECK(ok == 42);
}
