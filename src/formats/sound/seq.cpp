#include "formats/sound/seq.h"

namespace gt3r::sound {

namespace {

std::string read_cstr(ByteSpan d, u32 off) {
    if (off >= d.size()) throw FormatError("MSEQ: string offset out of range");
    std::string s;
    for (size_t p = off; p < d.size() && d[p]; ++p) s.push_back(char(d[p]));
    return s;
}

u8 byte_at(ByteSpan d, size_t& pos) {
    if (pos >= d.size()) throw FormatError("SEQG: track runs off the end of the file");
    return d[pos++];
}

} // namespace

std::vector<MusicEntry> parse_music_index(ByteSpan d) {
    if (!has_magic(d, "MSEQ") || d.size() < 0x14) throw FormatError("not an MSEQ index");
    u32 groups = rd32(d, 0x0C);
    if (groups > 1024 || 0x14ull + 8ull * groups > d.size()) throw FormatError("MSEQ: bad group count");
    std::vector<MusicEntry> out;
    for (u32 g = 0; g < groups; ++g) {
        u32 rec = rd32(d, 0x14 + 8 * g), count = rd32(d, 0x18 + 8 * g);
        if (count > 1024 || u64(rec) + 20ull * count > d.size()) throw FormatError("MSEQ: bad record range");
        for (u32 i = 0; i < count; ++i) {
            size_t o = rec + 20 * i;
            MusicEntry e;
            e.name = read_cstr(d, rd32(d, o));
            e.file = read_cstr(d, rd32(d, o + 4));
            e.title = read_cstr(d, rd32(d, o + 8));
            e.artist = read_cstr(d, rd32(d, o + 12));
            e.sequence = rd32(d, o + 16);
            e.group = g;
            out.push_back(std::move(e));
        }
    }
    return out;
}

SeqBank SeqBank::parse(ByteSpan d) {
    if (!has_magic(d, "SEQG") || d.size() < 0x0C) throw FormatError("not an SEQG bank");
    u32 n = rd32(d, 0x08);
    if (n > 1024 || 0x0Cull + 0x48ull * n > d.size()) throw FormatError("SEQG: bad sequence count");
    SeqBank b;
    b.data = std::make_shared<const Bytes>(d.begin(), d.end());
    for (u32 i = 0; i < n; ++i) {
        size_t o = 0x0C + 0x48 * i;
        Sequence s;
        s.volume = rd32(d, o);
        s.tempo = rd32(d, o + 4);
        for (int t = 0; t < kSeqTracks; ++t) {
            s.tracks[t] = rd32(d, o + 8 + 4 * t);
            if (s.tracks[t] >= d.size()) throw FormatError("SEQG: track offset out of range");
        }
        if (s.tempo == 0) throw FormatError("SEQG: zero tempo");
        b.sequences.push_back(s);
    }
    return b;
}

const char* seq_op_name(SeqOp op) {
    switch (op) {
    case SeqOp::Note: return "note";
    case SeqOp::PitchBend: return "bend";
    case SeqOp::LoopStart: return "loop";
    case SeqOp::LoopEnd: return "loop-end";
    case SeqOp::Program: return "program";
    case SeqOp::Volume: return "volume";
    case SeqOp::Pan: return "pan";
    case SeqOp::Tempo: return "tempo";
    case SeqOp::Nop: return "nop";
    case SeqOp::End: return "end";
    }
    return "?";
}

u32 read_seq_number(ByteSpan d, size_t& pos) {
    u32 v = 0;
    for (int i = 0; i < 4; ++i) {
        u8 b = byte_at(d, pos);
        v = v << 7 | (b & 0x7F);
        if (!(b & 0x80)) return v;
    }
    throw FormatError("SEQG: variable-length number longer than 4 bytes");
}

SeqEvent read_seq_event(ByteSpan d, size_t& pos) {
    SeqEvent e;
    u8 b = byte_at(d, pos);
    if (b & 0x80) {
        e.op = SeqOp::Note;
        e.key = b & 0x7F;
        e.velocity = byte_at(d, pos);
        e.length = read_seq_number(d, pos);
    } else if (b & 0x40) {
        e.op = SeqOp::PitchBend;
        e.bend = s32(((b & 0x3F) << 8) | byte_at(d, pos)) - 0x2000;
    } else if (b & 0x20) {
        e.op = SeqOp::LoopStart;
        e.value = u8((b & 0x1F) + 1);
    } else {
        switch (b) {
        case 0x00: e.op = SeqOp::End; break;
        case 0x01:
            e.op = SeqOp::LoopStart;
            e.value = byte_at(d, pos);
            break;
        case 0x02: e.op = SeqOp::LoopEnd; break;
        case 0x03:
            e.op = SeqOp::Program;
            e.value = byte_at(d, pos);
            break;
        case 0x04:
            e.op = SeqOp::Volume;
            e.value = byte_at(d, pos);
            break;
        case 0x05:
            e.op = SeqOp::Pan;
            e.value = byte_at(d, pos);
            break;
        case 0x06: {
            u32 a = byte_at(d, pos), m = byte_at(d, pos), l = byte_at(d, pos);
            e.op = SeqOp::Tempo;
            e.tempo = a << 16 | m << 8 | l;
            break;
        }
        default:
            e.op = SeqOp::Nop;
            e.value = b;
            break;
        }
    }
    return e;
}

std::vector<SeqTimedEvent> decode_seq_track(ByteSpan d, u32 offset) {
    std::vector<SeqTimedEvent> out;
    size_t pos = offset;
    u32 tick = read_seq_number(d, pos);
    for (;;) {
        SeqTimedEvent te;
        te.tick = tick;
        te.offset = u32(pos);
        te.ev = read_seq_event(d, pos);
        out.push_back(te);
        if (te.ev.op == SeqOp::End) break;
        tick += read_seq_number(d, pos);
        if (out.size() > (1u << 22)) throw FormatError("SEQG: track has no end");
    }
    return out;
}

SeqSummary summarize_sequence(const SeqBank& bank, size_t index) {
    const Sequence& s = bank.sequences.at(index);
    SeqSummary sum;
    bool first = true;
    for (u32 off : s.tracks) {
        if (!off) continue;
        auto evs = decode_seq_track(*bank.data, off);
        u32 notes = 0;
        for (const auto& e : evs) notes += e.ev.op == SeqOp::Note;
        sum.notes += notes;
        sum.tracks_used += notes > 0;
        if (!first) continue;
        first = false;
        sum.length_ticks = evs.back().tick;
        for (const auto& e : evs)
            if (e.ev.op == SeqOp::LoopStart && e.ev.value == kSeqLoopEndless) {
                sum.loop_start_tick = e.tick;
                break;
            }
    }
    return sum;
}

} // namespace gt3r::sound
