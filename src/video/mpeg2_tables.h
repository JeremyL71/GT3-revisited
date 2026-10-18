#pragma once

// Variable-length code tables and constants of ISO/IEC 13818-2 (MPEG-2
// video), Annex B, plus scan orders and default quantiser matrices.
//
// Every VLC table is expanded into a direct lookup table indexed by the next
// `bits` bits of the stream. The DCT coefficient tables (B.14/B.15) have codes
// up to 16 bits (sign excluded); they use two 1024-entry tables: one indexed
// by bits 0..9 for codes of <= 10 bits, and one indexed by the low 10 bits of
// a 16-bit peek whose top 6 bits are zero (every longer code starts with six
// zeros).

#include "base/bytes.h"

#include <vector>

namespace gt3r::video {

struct VlcEntry {
    s16 value = 0;
    u8 len = 0; // 0 = invalid code
};

struct VlcTable {
    int bits = 0;
    std::vector<VlcEntry> lut; // size 1 << bits
};

// Macroblock type flags (Table 6-2 / B.2-B.4).
enum MbFlags : u8 {
    kMbQuant = 1,
    kMbMotionForward = 2,
    kMbMotionBackward = 4,
    kMbPattern = 8,
    kMbIntra = 16,
};

// Special macroblock_address_increment values.
constexpr s16 kMbaEscape = -1; // 0000 0001 000: add 33 and continue
constexpr s16 kMbaStuffing = -2; // 0000 0001 111 (MPEG-1 only)

const VlcTable& vlc_mb_address_increment(); // B.1
const VlcTable& vlc_mb_type_i();             // B.2
const VlcTable& vlc_mb_type_p();             // B.3
const VlcTable& vlc_mb_type_b();             // B.4
const VlcTable& vlc_coded_block_pattern();   // B.9 (4:2:0, 6-bit cbp)
const VlcTable& vlc_motion_code();           // B.10, sign included: -16..16
const VlcTable& vlc_dc_size_luma();          // B.12
const VlcTable& vlc_dc_size_chroma();        // B.13

struct DctEntry {
    u8 run = 0;   // or kDctEob / kDctEscape
    u8 level = 0;
    u8 len = 0;   // code length without the sign bit; 0 = invalid
};
constexpr u8 kDctEob = 64;
constexpr u8 kDctEscape = 65;

struct DctTable {
    DctEntry short_codes[1024]; // indexed by peek(16) >> 6, codes <= 10 bits
    DctEntry long_codes[1024];  // indexed by peek(16) when it is < 1024
    const DctEntry& lookup(u32 peek16) const {
        return peek16 >= 1024 ? short_codes[peek16 >> 6] : long_codes[peek16];
    }
};

// B.14 (table zero). The first coefficient of a non-intra block uses the
// special code "1s" = run 0, level 1, which the decoder handles itself; this
// table maps "10" to end-of-block and "11s" to run 0 level 1.
const DctTable& dct_table_zero();
// B.15 (table one), used for intra blocks when intra_vlc_format = 1.
const DctTable& dct_table_one();

extern const u8 kZigzagScan[64];    // scan index -> raster position
extern const u8 kAlternateScan[64];
extern const u8 kDefaultIntraMatrix[64]; // raster order
extern const u8 kNonLinearQuantScale[32];

} // namespace gt3r::video
