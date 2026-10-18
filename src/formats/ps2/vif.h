#pragma once

// Generic PS2 VIF / GIF packet helpers.
//
// VIF code (32 bits): imm[15:0] num[23:16] cmd[30:24] irq[31]. Commands with
// payloads (sizes in bytes after the code):
//   STMASK 0x20: 4    STROW 0x30 / STCOL 0x31: 16    MPG 0x4A: num*8
//   DIRECT 0x50 / DIRECTHL 0x51: imm*16 (imm 0 = 65536 qwords)
//   UNPACK 0x60..0x7F: cmd = 011 m vn(2) vl(2); imm = addr[9:0] usn[14] flg[15]
//     data = num * (vn+1) * bits(vl) / 8, padded to 4 bytes (num 0 = 256).
//     vl: 0 = 32, 1 = 16, 2 = 8 bit; vn=3/vl=3 is V4-5 (RGB5A1, 2 bytes each).
//
// GIF tag (128 bits): NLOOP[14:0] EOP[15] PRE[46] PRIM[57:47] FLG[59:58]
// NREG[63:60], REGS[127:64] (4 bits per register descriptor).

#include "base/bytes.h"

#include <array>
#include <vector>

namespace gt3r::ps2 {

enum VifCmd : u8 {
    kVifNop = 0x00,
    kVifStcycl = 0x01,
    kVifOffset = 0x02,
    kVifBase = 0x03,
    kVifItop = 0x04,
    kVifStmod = 0x05,
    kVifMskpath3 = 0x06,
    kVifMark = 0x07,
    kVifFlushe = 0x10,
    kVifFlush = 0x11,
    kVifFlusha = 0x13,
    kVifMscal = 0x14,
    kVifMscalf = 0x15,
    kVifMscnt = 0x17,
    kVifStmask = 0x20,
    kVifStrow = 0x30,
    kVifStcol = 0x31,
    kVifMpg = 0x4A,
    kVifDirect = 0x50,
    kVifDirectHl = 0x51,
    kVifUnpack = 0x60, // 0x60..0x7F
};

struct VifUnpack {
    u32 vn = 0;    // components - 1 (0 = S, 3 = V4)
    u32 vl = 0;    // 0 = 32 bit, 1 = 16 bit, 2 = 8 bit, 3 = 5 bit (only V4-5)
    u32 count = 0; // vectors
    u32 addr = 0;  // VU memory address in qwords (imm & 0x3FF)
    bool usn = false; // unsigned (zero-extend) instead of sign-extend
    bool flg = false; // address relative to TOPS (double buffering)
    bool masked = false;
    ByteSpan data;

    u32 components() const { return vn + 1; }
    bool is_v4_5() const { return vn == 3 && vl == 3; }
};

struct VifCommand {
    u32 code = 0;
    ByteSpan payload;

    u8 cmd() const { return u8((code >> 24) & 0x7F); }
    u8 num() const { return u8(code >> 16); }
    u16 imm() const { return u16(code); }
    bool irq() const { return (code >> 31) != 0; }
    bool is_unpack() const { return (cmd() & 0x60) == 0x60; }
    VifUnpack unpack() const; // only valid when is_unpack()
};

// Payload size in bytes of an UNPACK of `count` vectors.
size_t vif_unpack_bytes(u32 vn, u32 vl, u32 count);

// Splits a VIF stream into commands. Throws FormatError on an unknown command
// or a payload that runs past the end.
std::vector<VifCommand> parse_vif(ByteSpan data);

// Expands an UNPACK into 4x32-bit words per vector, as the VIF writes them
// before masking (no STROW/STCOL/STMOD handling). Unused components are 0.
// 32-bit data is returned as raw bits (reinterpret as float when needed);
// 16/8-bit data is sign- or zero-extended according to `usn`.
std::vector<std::array<u32, 4>> unpack_words(const VifUnpack& u);

// Same, converted to floats: 32-bit data is reinterpreted as IEEE floats,
// narrower integer data is converted by value (no fixed-point scale applied).
std::vector<std::array<float, 4>> unpack_floats(const VifUnpack& u);

// GS PRIM register (also embedded in GIF tags).
struct Prim {
    u32 type = 0; // 0 point, 1 line, 2 line strip, 3 tri, 4 tri strip, 5 tri fan, 6 sprite
    bool iip = false, tme = false, fge = false, abe = false, aa1 = false, fst = false, ctxt = false, fix = false;
    static Prim decode(u32 v);
};

struct GifTag {
    u32 nloop = 0;
    bool eop = false;
    bool pre = false;
    u32 prim = 0; // raw 11-bit PRIM value (valid when pre)
    u32 flg = 0;  // 0 PACKED, 1 REGLIST, 2 IMAGE
    u32 nreg = 0; // 0 means 16
    u64 regs = 0;

    static GifTag decode(u64 lo, u64 hi);
    // Decodes a tag from up to 16 bytes (missing high bytes are zero).
    static GifTag decode(ByteSpan bytes);
    u32 reg(u32 i) const { return u32(regs >> (4 * i)) & 0xF; }
    u32 reg_count() const { return nreg ? nreg : 16; }
};

// GIF PACKED register descriptors.
enum GifReg : u32 { kGifPrim = 0, kGifRgbaq = 1, kGifSt = 2, kGifUv = 3, kGifXyzf2 = 4, kGifXyz2 = 5, kGifAd = 0xE };

} // namespace gt3r::ps2
