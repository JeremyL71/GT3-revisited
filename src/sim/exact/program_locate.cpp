// The program's masked code; see program_locate.h.

#include "sim/exact/program_locate.h"

#include <algorithm>
#include <array>

namespace gt3r::sim::exact {

namespace {

constexpr u64 kBase = 1099511628211ull; // the rolling hash's base (odd)

// An upper half that forms an address: anywhere from the program's 1 MB load address up to the end of the EE's 32 MB.
// (A float constant's upper half is far above; an upper half of 1 .. 15 builds a plain number.)
bool address_half(u32 imm) { return imm >= 0x10 && imm < 0x200; }
// A loaded value that is an address in the EE's memory (a pointer: it moves with the build).
bool address_value(u32 v) { return v >= 0x00100000u && v < 0x02000000u; }

int load_size(u32 op) {
    switch (op) {
    case 0x20: case 0x24: return 1;                                // byte loads
    case 0x21: case 0x25: return 2;                                // halfword loads
    case 0x22: case 0x23: case 0x26: case 0x27: case 0x31: return 4; // word loads (also unaligned, unsigned, into the FPU)
    case 0x1A: case 0x1B: case 0x37: return 8;                     // doubleword loads (also unaligned)
    case 0x1E: case 0x36: return 16;                               // quadword loads (also into VU0)
    default: return 0;
    }
}
bool store_op(u32 op) {
    switch (op) {
    case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2C: case 0x2D: case 0x2E: case 0x1F: case 0x3F: case 0x39: case 0x3E:
        return true;
    default: return false;
    }
}
// The I-type ops that write a general register rt (not the branches, stores or loads into a coprocessor register).
bool writes_rt(u32 op) {
    return (op >= 0x08 && op <= 0x0E) || op == 0x18 || op == 0x19 || op == 0x1A || op == 0x1B || op == 0x1E ||
           (op >= 0x20 && op <= 0x27) || op == 0x37;
}
bool caller_saved(int r) { return (r >= 1 && r <= 15) || r == 24 || r == 25 || r == 31; }

} // namespace

CodeIndex::CodeIndex(ByteSpan program, u32 load_address, u32 entry_point, const Options& opt)
    : program_(program), load_(load_address) {
    const u32 n_words = u32(program.size() / 4);
    // The end of the code: after the last routine return that is followed by more than 16 KB without one.
    u32 last = 0;
    for (u32 k = 0; k < n_words; ++k) {
        if (rd32(program, 4 * k) != 0x03E00008) continue;
        if (last && 4 * (k - last) > 0x4000) break;
        last = k;
    }
    text_end_ = std::min<u32>(load_ + u32(program.size()), (load_ + 4 * last + 8 + 15) & ~15u);
    // The global pointer: the start-up code builds its value from two halves and moves it in, or sets it directly.
    if (in_image(entry_point, 4)) {
        u32 regs[32] = {};
        for (u32 pc = entry_point; pc < entry_point + 0x100 && in_image(pc, 4); pc += 4) {
            const u32 w = word(pc), op = w >> 26, rs = (w >> 21) & 31, rt = (w >> 16) & 31;
            bool to_gp = false;
            if (op == 0x0F) regs[rt] = (w & 0xFFFF) << 16;
            else if (op == 0x09 || op == 0x19) regs[rt] = regs[rs] + u32(s32(s16(w & 0xFFFF))), to_gp = rt == 28;
            else if (op == 0x0D) regs[rt] = regs[rs] | (w & 0xFFFF), to_gp = rt == 28;
            else if ((w & 0xFC1F07FF) == 0x0000002D && ((w >> 11) & 31) == 28) regs[28] = regs[rs], to_gp = true; // moved into the global pointer
            if (to_gp && regs[28]) {
                gp_ = regs[28];
                break;
            }
        }
    }

    // The masked words: one pass over the code with what each register holds of an address (the upper half it was given).
    const u32 code = (text_end_ - load_) / 4;
    masked_.resize(code);
    struct Half {
        bool valid = false;
        u32 hi = 0;
    };
    std::array<Half, 32> reg{};
    int reset_at = -1, reset_kind = 0; // after a jump's delay slot: every register (1) or the caller-saved ones (2)
    auto clear = [&](u32 r) {
        if (r) reg[r].valid = false;
    };
    for (u32 i = 0; i < code; ++i) {
        if (int(i) == reset_at) {
            for (int r = 0; r < 32; ++r)
                if (reset_kind == 1 || caller_saved(r)) reg[size_t(r)].valid = false;
            reset_at = -1;
        }
        const u32 w = rd32(program, 4 * i), op = w >> 26, rs = (w >> 21) & 31, rt = (w >> 16) & 31, rd = (w >> 11) & 31;
        const u32 imm = w & 0xFFFF;
        u32 m = w;
        if (op == 0x02 || op == 0x03) { // a jump or a call
            m = op << 26;
            reset_at = int(i) + 2, reset_kind = op == 0x02 ? 1 : 2;
        } else if (op == 0x00) {
            const u32 funct = w & 0x3F;
            if (funct == 0x08) reset_at = int(i) + 2, reset_kind = 1; // a jump to a register
            else if (funct == 0x09) reset_at = int(i) + 2, reset_kind = 2, clear(rd); // a call through a register
            else if (funct != 0x0C && funct != 0x0D && funct != 0x0F) clear(rd); // not a system call, break or sync
        } else if (op == 0x01) { // the branch group: its linking branches call
            if (rt == 0x10 || rt == 0x11 || rt == 0x12 || rt == 0x13) reset_at = int(i) + 2, reset_kind = 2;
        } else if (op == 0x0F) { // an upper half
            if (address_half(imm)) {
                m = w & 0xFFFF0000;
                if (rt) reg[rt] = {true, imm << 16};
            } else {
                clear(rt);
            }
        } else if (op == 0x1C) { // the multimedia group
            clear(rd);
        } else if (op == 0x11 || op == 0x12) { // FPU / VU0 moves into a general register
            if (rs == 0 || rs == 1 || rs == 2) clear(rt);
        } else {
            const int size = load_size(op);
            const bool store = store_op(op), arith = op == 0x09 || op == 0x19 || op == 0x0D;
            if (size || store || arith) {
                bool known = false;
                u32 addr = 0;
                if (rs == 28) {
                    known = gp_ != 0;
                    addr = gp_ + u32(s32(s16(imm)));
                    if (!known) m = w & 0xFFFF0000;
                } else if (reg[rs].valid) {
                    known = true;
                    addr = op == 0x0D ? (reg[rs].hi | imm) : reg[rs].hi + u32(s32(s16(imm)));
                }
                if (!known && opt.mask_offsets && (size || store) && rs != 29) m = w & 0xFFFF0000;
                if (known) {
                    m = w & 0xFFFF0000;
                    if (opt.load_values && size && in_image(addr, u32(size))) {
                        u32 v = 0;
                        for (int k = 0; k < size; k += 4) {
                            u32 part = 0;
                            for (int b = 0; b < std::min(4, size - k); ++b) part |= u32(program[addr - load_ + u32(k + b)]) << (8 * b);
                            v ^= part;
                        }
                        if (address_value(v)) v = 0;
                        m ^= v * 0x9E3779B1u;
                    }
                }
            }
            if (writes_rt(op)) clear(rt);
        }
        if (opt.mask_registers) m &= op == 0 || op == 0x1C ? 0xFC00003Fu : 0xFC00FFFFu;
        masked_[i] = m;
    }
    prefix_.assign(code + 1, 0);
    power_.assign(code + 1, 1);
    for (u32 i = 0; i < code; ++i) {
        prefix_[i + 1] = prefix_[i] * kBase + u64(masked_[i]) + 1;
        power_[i + 1] = power_[i] * kBase;
    }
}

u32 CodeIndex::word(u32 addr) const { return in_image(addr, 4) ? rd32(program_, addr - load_) : 0; }

ByteSpan CodeIndex::bytes(u32 addr, u32 n) const {
    if (!in_image(addr, n)) return {};
    return program_.subspan(addr - load_, n);
}

u64 CodeIndex::hash(size_t i, size_t n) const {
    if (i + n > masked_.size()) return 0;
    return prefix_[i + n] - prefix_[i] * power_[n];
}

std::vector<u32> CodeIndex::find(u64 h, u32 n) const {
    std::vector<u32> out;
    if (n == 0 || n > masked_.size()) return out;
    const u64 pn = power_[n];
    for (size_t i = 0; i + n <= masked_.size(); ++i)
        if (prefix_[i + n] - prefix_[i] * pn == h) out.push_back(load_ + u32(4 * i));
    return out;
}

} // namespace gt3r::sim::exact
