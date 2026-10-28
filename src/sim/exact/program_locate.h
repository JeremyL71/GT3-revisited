#pragma once

// Finding GT3's routines in a program image by their code, whatever the build: ProgramData's locator (program_data.h).
//
// Every instruction of the program's code is reduced to the part that does not depend on where the build put things
// ("masked"): jump targets cleared, the two halves of an address an instruction pair builds (or loads or stores through)
// cleared, offsets from the global pointer cleared, and a load from a fixed address replaced by the value it loads (a
// constant: a float, a count; 0 when the value is itself an address). A routine is then known by a hash of its masked code, wherever it sits, and a table it addresses is
// found by decoding the address from that routine's instructions. Only hashes and instruction positions are kept in the
// sources, no code and no addresses of any program.

#include "base/bytes.h"

#include <vector>

namespace gt3r::sim::exact {

class CodeIndex {
public:
    // The masking (the defaults are ProgramData's; the others are for surveys of how builds differ).
    struct Options {
        bool load_values = true;   // a load from a fixed address hashes the value it loads (else only masked)
        bool mask_offsets = false; // also mask every load / store offset from a register (struct offsets)
        bool mask_registers = false; // also mask every register field (a survey's upper bound only)
    };
    // `program`: the image loaded at `load_address`, `entry_point` its start-up code (where the global pointer is set).
    CodeIndex(ByteSpan program, u32 load_address, u32 entry_point) : CodeIndex(program, load_address, entry_point, Options{}) {}
    CodeIndex(ByteSpan program, u32 load_address, u32 entry_point, const Options& o);

    u32 load() const { return load_; }
    u32 text_end() const { return text_end_; } // the end of the code (after the last return before a long run without one)
    u32 gp() const { return gp_; }             // the global pointer as the start-up code sets it (0: not found)
    size_t words() const { return masked_.size(); }
    u32 masked(size_t i) const { return masked_[i]; }
    // The raw word at a program address (0 outside the image).
    u32 word(u32 addr) const;
    bool in_image(u32 addr, u32 n = 1) const { return addr >= load_ && addr - load_ + u64(n) <= program_.size(); }
    ByteSpan bytes(u32 addr, u32 n) const;

    // The hash of the masked words [i, i + n) (i and n in instructions from the load address).
    u64 hash(size_t i, size_t n) const;
    // Every code address where n masked words hash to h.
    std::vector<u32> find(u64 h, u32 n) const;
    // The hash of the masked code [start, end) (code addresses).
    u64 hash_range(u32 start, u32 end) const { return hash((start - load_) / 4, (end - start) / 4); }

private:
    ByteSpan program_;
    u32 load_ = 0, text_end_ = 0, gp_ = 0;
    std::vector<u32> masked_;
    std::vector<u64> prefix_, power_;
};

} // namespace gt3r::sim::exact
