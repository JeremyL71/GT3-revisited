#include "formats/ps2/vif.h"

#include <cstring>
#include <string>

namespace gt3r::ps2 {

size_t vif_unpack_bytes(u32 vn, u32 vl, u32 count) {
    if (vn == 3 && vl == 3) return (size_t(count) * 2 + 3) & ~size_t(3);
    static const u32 bits[4] = {32, 16, 8, 0};
    if (vl == 3) throw FormatError("invalid VIF UNPACK format");
    size_t bytes = size_t(count) * (vn + 1) * bits[vl] / 8;
    return (bytes + 3) & ~size_t(3);
}

VifUnpack VifCommand::unpack() const {
    VifUnpack u;
    u8 c = cmd();
    u.vn = (c >> 2) & 3;
    u.vl = c & 3;
    u.masked = (c & 0x10) != 0;
    u.count = num() ? num() : 256;
    u.addr = imm() & 0x3FF;
    u.usn = (imm() >> 14) & 1;
    u.flg = (imm() >> 15) & 1;
    u.data = payload;
    return u;
}

std::vector<VifCommand> parse_vif(ByteSpan data) {
    std::vector<VifCommand> out;
    size_t o = 0;
    while (o + 4 <= data.size()) {
        VifCommand c;
        c.code = rd32(data, o);
        o += 4;
        size_t len = 0;
        u8 cmd = c.cmd();
        if ((cmd & 0x60) == 0x60) {
            u32 n = c.num() ? c.num() : 256;
            len = vif_unpack_bytes((cmd >> 2) & 3, cmd & 3, n);
        } else {
            switch (cmd) {
            case kVifNop: case kVifStcycl: case kVifOffset: case kVifBase: case kVifItop: case kVifStmod:
            case kVifMskpath3: case kVifMark: case kVifFlushe: case kVifFlush: case kVifFlusha: case kVifMscal:
            case kVifMscalf: case kVifMscnt:
                break;
            case kVifStmask: len = 4; break;
            case kVifStrow: case kVifStcol: len = 16; break;
            case kVifMpg: len = size_t(c.num() ? c.num() : 256) * 8; break;
            case kVifDirect: case kVifDirectHl: len = size_t(c.imm() ? c.imm() : 65536) * 16; break;
            default: throw FormatError("unknown VIF command 0x" + std::to_string(cmd));
            }
        }
        if (len > data.size() - o) throw FormatError("VIF payload past end of packet");
        c.payload = data.subspan(o, len);
        o += len;
        out.push_back(c);
    }
    return out;
}

std::vector<std::array<u32, 4>> unpack_words(const VifUnpack& u) {
    std::vector<std::array<u32, 4>> out(u.count, std::array<u32, 4>{0, 0, 0, 0});
    if (vif_unpack_bytes(u.vn, u.vl, u.count) > u.data.size()) throw FormatError("VIF UNPACK data truncated");
    if (u.is_v4_5()) {
        for (u32 i = 0; i < u.count; ++i) {
            u32 v = rd16(u.data, 2 * i);
            out[i] = {(v & 0x1F) << 3, ((v >> 5) & 0x1F) << 3, ((v >> 10) & 0x1F) << 3, ((v >> 15) & 1) << 7};
        }
        return out;
    }
    u32 n = u.components();
    for (u32 i = 0; i < u.count; ++i)
        for (u32 k = 0; k < n; ++k) {
            size_t idx = size_t(i) * n + k;
            u32 v = 0;
            switch (u.vl) {
            case 0: v = rd32(u.data, idx * 4); break;
            case 1: v = rd16(u.data, idx * 2); if (!u.usn) v = u32(s32(s16(v))); break;
            case 2: v = u.data[idx]; if (!u.usn) v = u32(s32(std::int8_t(v))); break;
            }
            out[i][k] = v;
        }
    return out;
}

std::vector<std::array<float, 4>> unpack_floats(const VifUnpack& u) {
    std::vector<std::array<u32, 4>> w = unpack_words(u);
    std::vector<std::array<float, 4>> out(w.size());
    for (size_t i = 0; i < w.size(); ++i)
        for (int k = 0; k < 4; ++k) {
            if (u.vl == 0) std::memcpy(&out[i][k], &w[i][k], 4);
            else out[i][k] = (u.usn || u.is_v4_5()) ? float(w[i][k]) : float(s32(w[i][k]));
        }
    return out;
}

Prim Prim::decode(u32 v) {
    Prim p;
    p.type = v & 7;
    p.iip = (v >> 3) & 1;
    p.tme = (v >> 4) & 1;
    p.fge = (v >> 5) & 1;
    p.abe = (v >> 6) & 1;
    p.aa1 = (v >> 7) & 1;
    p.fst = (v >> 8) & 1;
    p.ctxt = (v >> 9) & 1;
    p.fix = (v >> 10) & 1;
    return p;
}

GifTag GifTag::decode(u64 lo, u64 hi) {
    GifTag t;
    t.nloop = u32(lo & 0x7FFF);
    t.eop = (lo >> 15) & 1;
    t.pre = (lo >> 46) & 1;
    t.prim = u32(lo >> 47) & 0x7FF;
    t.flg = u32(lo >> 58) & 3;
    t.nreg = u32(lo >> 60) & 0xF;
    t.regs = hi;
    return t;
}

GifTag GifTag::decode(ByteSpan bytes) {
    u8 buf[16] = {};
    std::memcpy(buf, bytes.data(), bytes.size() < 16 ? bytes.size() : 16);
    u64 lo, hi;
    std::memcpy(&lo, buf, 8);
    std::memcpy(&hi, buf + 8, 8);
    return decode(lo, hi);
}

} // namespace gt3r::ps2
