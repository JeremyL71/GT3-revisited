#include "formats/tex1.h"

namespace gt3r {

bool Tex1::is_tex1(ByteSpan data) { return has_magic(data, "Tex1") && data.size() >= 0x20; }

Tex1 Tex1::parse(ByteSpan data) {
    if (!is_tex1(data)) throw FormatError("not a Tex1 texture set");
    Tex1 t;
    u32 file_size = rd32(data, 0x0C);
    if (file_size > data.size()) throw FormatError("Tex1 truncated");
    u16 ntex = rd16(data, 0x14);
    u16 nbuf = rd16(data, 0x16);
    u32 tex_off = rd32(data, 0x18);
    u32 buf_off = rd32(data, 0x1C);
    for (u32 i = 0; i < ntex; ++i) {
        size_t o = tex_off + size_t(i) * 0x28;
        Texture tx;
        tx.tex0 = Tex0::decode(rd64(data, o));
        tx.tex1 = rd64(data, o + 0x08);
        tx.clamp = Clamp::decode(rd64(data, o + 0x20));
        t.textures.push_back(tx);
    }
    for (u32 i = 0; i < nbuf; ++i) {
        size_t o = buf_off + size_t(i) * 12;
        Buffer b;
        b.offset = rd32(data, o);
        b.dbp = rd16(data, o + 4);
        b.dbw = data[o + 6];
        b.dpsm = data[o + 7];
        b.width = rd16(data, o + 8);
        b.height = rd16(data, o + 10);
        t.buffers.push_back(b);
    }
    return t;
}

void Tex1::upload(ByteSpan data, GsMemory& gs) const {
    for (const Buffer& b : buffers) {
        size_t bytes = size_t(b.width) * b.height * psm_bits(b.dpsm) / 8;
        if (b.offset > data.size() || data.size() - b.offset < bytes) throw FormatError("Tex1 buffer out of range");
        gs.upload(b.dbp, b.dbw, b.dpsm, b.width, b.height, data.subspan(b.offset, bytes));
    }
}

Tex1::Image Tex1::decode(const GsMemory& gs, size_t i) const {
    const Texture& t = textures.at(i);
    Image img;
    u32 x0 = 0, y0 = 0;
    img.width = t.tex0.width();
    img.height = t.tex0.height();
    if (t.clamp.wms == 2 && t.clamp.maxu >= t.clamp.minu) {
        x0 = t.clamp.minu;
        img.width = t.clamp.maxu - t.clamp.minu + 1;
    }
    if (t.clamp.wmt == 2 && t.clamp.maxv >= t.clamp.minv) {
        y0 = t.clamp.minv;
        img.height = t.clamp.maxv - t.clamp.minv + 1;
    }
    img.rgba = gs.decode_rgba(t.tex0, img.width, img.height, x0, y0);
    return img;
}

std::vector<size_t> find_tex1_sets(ByteSpan data) {
    std::vector<size_t> out;
    for (size_t o = 0; o + 0x20 <= data.size(); o += 16) { // sets are 16-byte aligned
        if (std::memcmp(&data[o], "Tex1", 4) != 0) continue;
        u32 size = rd32(data, o + 0x0C);
        if (size >= 0x20 && size <= data.size() - o) out.push_back(o);
    }
    return out;
}

} // namespace gt3r
