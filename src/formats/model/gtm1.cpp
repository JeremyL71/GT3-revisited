#include "formats/model/gtm1.h"

#include "formats/gs_memory.h"
#include "formats/tex1.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <map>

namespace gt3r::gtm1 {

namespace {

float rdf(ByteSpan d, size_t o) {
    u32 v = rd32(d, o);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}

constexpr u32 kNoTexset = 0xFFFFFFFFu;
constexpr Matrix kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

// a * b with row vectors (apply a, then b).
Matrix mul(const Matrix& a, const Matrix& b) {
    Matrix r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0;
            for (int k = 0; k < 4; ++k) s += a[i * 4 + k] * b[k * 4 + j];
            r[i * 4 + j] = s;
        }
    return r;
}

Matrix rotation(float ax, float ay, float az, float radians) {
    float len = std::sqrt(ax * ax + ay * ay + az * az);
    if (len == 0) return kIdentity;
    ax /= len, ay /= len, az /= len;
    float c = std::cos(radians), s = std::sin(radians), t = 1 - c;
    // Row-vector form of the standard axis-angle matrix (transpose of the column form).
    return {t * ax * ax + c,      t * ax * ay + s * az, t * ax * az - s * ay, 0,
            t * ax * ay - s * az, t * ay * ay + c,      t * ay * az + s * ax, 0,
            t * ax * az + s * ay, t * ay * az - s * ax, t * az * az + c,      0,
            0, 0, 0, 1};
}

// Operand bytes after each opcode for the fixed-size ops. -1 = variable or
// control flow handled explicitly, -2 = unknown.
int fixed_operand_bytes(u8 op) {
    switch (op) {
    case 0x01: case 0x0B: case 0x0C: case 0x0D: case 0x17: case 0x18: case 0x1A: case 0x1B: case 0x1D: case 0x1E:
    case 0x22: case 0x23: case 0x25: case 0x26: case 0x2D: case 0x2E: case 0x30:
        return 0;
    case 0x02: case 0x04: case 0x0E: case 0x19: case 0x1F: case 0x29: case 0x2A: case 0x2B: case 0x2F:
        return 1;
    case 0x03: case 0x05: case 0x1C: case 0x20: case 0x2C:
        return 2;
    case 0x14: case 0x15: case 0x16: case 0x21: case 0x24: case 0x27: case 0x32: case 0x34: case 0x35: case 0x36:
        return 4;
    case 0x11: case 0x12: case 0x31:
        return 12;
    case 0x13: case 0x28: case 0x33: case 0x37:
        return 16;
    case 0x0F: case 0x10:
        return 64;
    case 0x00: case 0x06: case 0x07: case 0x08: case 0x09: case 0x0A:
        return -1;
    default:
        return -2;
    }
}

u32 expand_color(const std::array<u32, 4>& c) {
    auto ch = [](u32 v) { return std::min<u32>(255, (v * 255 + 64) / 128); };
    return ch(c[0]) | ch(c[1]) << 8 | ch(c[2]) << 16 | ch(c[3]) << 24;
}

} // namespace

bool Gtm1::is_gtm1(ByteSpan data) { return has_magic(data, "GTM1") && data.size() >= 0x40; }

Gtm1 Gtm1::parse(ByteSpan data) {
    if (!is_gtm1(data)) throw FormatError("not a GTM1 model");
    Gtm1 g;
    g.data = data;
    u32 node_count = rd16(data, 0x10), chunk_count = rd16(data, 0x12), texset_count = rd16(data, 0x16);
    u32 tex_lods = rd16(data, 0x18);
    u32 node_tab = rd32(data, 0x20), chunk_tab = rd32(data, 0x24), texset_tab = rd32(data, 0x2C);
    u32 sphere_tab = rd32(data, 0x30), lod_tab = rd32(data, 0x34);

    for (u32 i = 0; i < node_count; ++i) {
        g.nodes.push_back(rd32(data, node_tab + 4 * i));
        std::array<float, 4> s{0, 0, 0, 0};
        if (sphere_tab)
            for (int k = 0; k < 4; ++k) s[k] = rdf(data, sphere_tab + 16 * i + 4 * k);
        g.spheres.push_back(s);
    }

    u32 mat_count = rd16(data, 0x14), mat_tab = rd32(data, 0x28);
    auto read_materials = [&](u32 tab) {
        std::vector<Material> out;
        for (u32 i = 0; tab && i < mat_count; ++i) {
            Material m;
            size_t o = tab + 0x50 * size_t(i);
            if (o + 0x48 > data.size()) throw FormatError("GTM1 material out of range");
            for (int k = 0; k < 16; ++k) m.rows[k / 4][k % 4] = rdf(data, o + 4 * k);
            m.shininess = rdf(data, o + 0x40);
            m.flags = rd32(data, o + 0x44);
            out.push_back(m);
        }
        return out;
    };
    g.materials = read_materials(mat_tab);
    // Per-paint material sets: +0x38 -> u32 offsets[count at +0x1A], each a full array.
    u32 paint_sets = rd16(data, 0x1A), paint_tab = rd32(data, 0x38);
    for (u32 i = 0; paint_tab && i < paint_sets && paint_tab + 4 * i + 4 <= data.size(); ++i)
        g.paint_materials.push_back(read_materials(rd32(data, paint_tab + 4 * i)));
    for (u32 i = 0; i < chunk_count; ++i) {
        Chunk c;
        c.offset = rd32(data, chunk_tab + 4 * i);
        c.flags = rd16(data, c.offset + 8);
        u32 n = rd16(data, c.offset + 10);
        c.vertex_count = rd16(data, c.offset + 12);
        c.triangle_count = rd16(data, c.offset + 14);
        for (u32 k = 0; k < n; ++k) {
            size_t e = c.offset + 16 + 8 * size_t(k);
            Packet p;
            p.offset = c.offset + rd32(data, e);
            p.qwc = rd16(data, e + 4);
            p.mat = rd16(data, e + 6);
            if (p.offset > data.size() || size_t(p.qwc) * 16 > data.size() - p.offset)
                throw FormatError("GTM1 packet out of range");
            c.packets.push_back(p);
        }
        g.chunks.push_back(std::move(c));
    }

    auto set_index = [&](u32 off) -> u32 {
        if (off == 0) return kNoTexset; // unused slot (e.g. wheel/pd0004)
        auto it = std::find(g.texsets.begin(), g.texsets.end(), off);
        if (it != g.texsets.end()) return u32(it - g.texsets.begin());
        if (off >= data.size() || !Tex1::is_tex1(data.subspan(off))) throw FormatError("GTM1 texture set is not Tex1");
        g.texsets.push_back(off);
        return u32(g.texsets.size() - 1);
    };
    std::vector<u32> base;
    for (u32 i = 0; i < texset_count; ++i) base.push_back(set_index(rd32(data, texset_tab + 4 * i)));
    g.texset_tables.push_back(base);
    if (lod_tab)
        for (u32 v = 0; v < tex_lods; ++v) {
            u32 tab = rd32(data, lod_tab + 4 * v);
            std::vector<u32> t;
            for (u32 i = 0; i < texset_count; ++i) t.push_back(set_index(rd32(data, tab + 4 * i)));
            if (v == 0) g.texset_tables[0] = t;
            else g.texset_tables.push_back(t);
        }
    return g;
}

std::vector<Draw> Gtm1::run(u32 node, const RunParams& params) const {
    std::vector<Matrix> stack{kIdentity};
    std::vector<Draw> out;
    run_at(node, params, stack, out, 0);
    return out;
}

void Gtm1::run_at(u32 node, const RunParams& p, std::vector<Matrix>& stack, std::vector<Draw>& out, int depth) const {
    if (node >= nodes.size()) throw FormatError("GTM1 node index out of range");
    if (depth > 16) throw FormatError("GTM1 node call depth exceeded");
    const std::vector<u32>& sets = texset_tables[std::clamp<size_t>(size_t(std::max(p.texture_lod, 0)), 0,
                                                                     texset_tables.size() - 1)];
    size_t pc = nodes[node];
    u32 texset = sets.empty() ? 0 : sets[0];
    bool matrix_used = false;
    u8 alpha_mode = 0x44;
    u32 fb_mask = 0;
    // Entry state: car/wheel programs end by restoring ATE on, (GREATER, 0x20), DATE off, so that is the
    // convention they are entered with.
    bool ate = true, date = false, zwrite = true, ztest = true, cull = true;
    u8 atst = 6, aref = 0x20, afail = 0;
    std::array<float, 4> env_c0{1, 1, 1, 1}, env_c1{0, 0, 0, 0}; // reflection weight rows; see Draw
    auto draw = [&](u32 chunk) {
        if (chunk >= chunks.size()) throw FormatError("GTM1 draw of unknown chunk");
        Draw d;
        d.chunk = chunk;
        d.texset = texset;
        d.matrix = stack.back();
        d.has_matrix = matrix_used;
        d.alpha_mode = alpha_mode;
        d.fb_mask = fb_mask;
        d.alpha_test = ate;
        d.alpha_test_method = atst;
        d.alpha_test_ref = aref;
        d.dest_alpha_test = date;
        d.alpha_fail = afail;
        d.depth_write = zwrite;
        d.depth_test = ztest;
        d.cull = cull;
        d.env_c0 = env_c0;
        d.env_c1 = env_c1;
        out.push_back(d);
    };
    auto s16_at = [&](size_t o) { return s32(s16(rd16(data, o))); };
    for (int steps = 0; steps < 100000; ++steps) {
        if (pc >= data.size()) throw FormatError("GTM1 program ran off the end");
        u8 op = data[pc];
        size_t a = pc + 1; // operand address
        switch (op) {
        case 0x00: return;
        case 0x02: run_at(data[a], p, stack, out, depth + 1); pc = a + 1; continue;
        case 0x03: run_at(rd16(data, a), p, stack, out, depth + 1); pc = a + 2; continue;
        case 0x04: draw(data[a]); pc = a + 1; continue;
        case 0x05: draw(rd16(data, a)); pc = a + 2; continue;
        case 0x06: { // s16 ?, u8 n, s16 rel[n]; rel relative to its own field
            u32 n = data[a + 2];
            size_t tab = a + 3;
            int sel = rd16(data, a) == 0 ? (p.brake ? 1 : 0) : p.selector; // id 0: brake now
            if (sel >= 0 && u32(sel) < n) pc = tab + 2 * sel + s16_at(tab + 2 * sel);
            else pc = tab + 2 * n;
            continue;
        }
        case 0x07: { // 16-byte bounding sphere, u8 n, s16 rel[n]
            u32 n = data[a + 16];
            if (n == 0) { pc = a + 17; continue; }
            u32 k = u32(std::clamp(p.lod, 0, int(n) - 1));
            size_t f = a + 17 + 2 * k;
            pc = f + s16_at(f);
            continue;
        }
        case 0x08: pc = a + s32(std::int8_t(data[a])); continue;
        case 0x09: pc = a + s16_at(a); continue;
        case 0x0A: pc = a + 1 + 12 * size_t(data[a]) + 2; continue; // bbox cull: never culled here
        case 0x0C: stack.push_back(stack.back()); pc = a; continue;
        case 0x0D: if (stack.size() > 1) stack.pop_back(); pc = a; continue;
        case 0x0F:
        case 0x10: {
            Matrix m;
            for (int i = 0; i < 16; ++i) m[i] = rdf(data, a + 4 * i);
            stack.back() = op == 0x0F ? m : mul(m, stack.back());
            matrix_used = true;
            pc = a + 64;
            continue;
        }
        case 0x11: {
            Matrix& m = stack.back();
            float v[3] = {rdf(data, a), rdf(data, a + 4), rdf(data, a + 8)};
            for (int j = 0; j < 4; ++j) m[12 + j] += v[0] * m[j] + v[1] * m[4 + j] + v[2] * m[8 + j];
            matrix_used = true;
            pc = a + 12;
            continue;
        }
        case 0x12: {
            Matrix& m = stack.back();
            for (int r = 0; r < 3; ++r)
                for (int j = 0; j < 4; ++j) m[r * 4 + j] *= rdf(data, a + 4 * r);
            matrix_used = true;
            pc = a + 12;
            continue;
        }
        case 0x13:
        case 0x14:
        case 0x15:
        case 0x16: {
            Matrix r;
            if (op == 0x13) r = rotation(rdf(data, a), rdf(data, a + 4), rdf(data, a + 8), rdf(data, a + 12));
            else {
                float rad = rdf(data, a) * 3.14159265f / 180.0f;
                r = rotation(op == 0x14, op == 0x15, op == 0x16, rad);
            }
            stack.back() = mul(r, stack.back());
            matrix_used = true;
            pc = a + (op == 0x13 ? 16 : 4);
            continue;
        }
        case 0x20: alpha_mode = data[a]; pc = a + 2; continue;
        case 0x24: fb_mask = rd32(data, a); pc = a + 4; continue;
        case 0x1A: ate = true; pc = a; continue;
        case 0x1B: ate = false; pc = a; continue;
        case 0x1C: atst = data[a] & 7; aref = data[a + 1]; pc = a + 2; continue;
        case 0x1D: date = true; pc = a; continue;
        case 0x1E: date = false; pc = a; continue;
        case 0x2F: afail = data[a] & 3; pc = a + 1; continue;
        case 0x25: zwrite = false; pc = a; continue;
        case 0x26: zwrite = true; pc = a; continue;
        case 0x17: ztest = true; pc = a; continue;
        case 0x18: ztest = false; pc = a; continue;
        case 0x2D: cull = true; pc = a; continue;
        case 0x2E: cull = false; pc = a; continue;
        case 0x2B: texset = data[a] < sets.size() ? sets[data[a]] : texset; pc = a + 1; continue;
        case 0x2C: texset = rd16(data, a) < sets.size() ? sets[rd16(data, a)] : texset; pc = a + 2; continue;
        case 0x35: draw(rd16(data, a)); pc = a + 4; continue; // animated chunk range: first frame
        case 0x32: { float f = rdf(data, a); env_c1 = {f, f, f, 0}; pc = a + 4; continue; }
        case 0x33: env_c1 = {rdf(data, a), rdf(data, a + 4), rdf(data, a + 8), rdf(data, a + 12)}; pc = a + 16; continue;
        case 0x36: { float f = rdf(data, a); env_c0 = {f, f, f, 1}; pc = a + 4; continue; }
        case 0x37: env_c0 = {rdf(data, a), rdf(data, a + 4), rdf(data, a + 8), rdf(data, a + 12)}; pc = a + 16; continue;
        default: {
            int n = fixed_operand_bytes(op);
            if (n < 0) throw FormatError("GTM1 program: unknown opcode " + std::to_string(op));
            pc = a + n; // render-state ops: not needed for geometry
            continue;
        }
        }
    }
    throw FormatError("GTM1 program does not terminate");
}

DecodedPacket Gtm1::decode_packet(const Packet& pk) const {
    DecodedPacket out;
    std::vector<ps2::VifCommand> cmds = ps2::parse_vif(data.subspan(pk.offset, size_t(pk.qwc) * 16));
    std::vector<std::array<float, 4>> pos, nrm, uv, envn;
    std::vector<std::array<u32, 4>> col;
    for (const ps2::VifCommand& c : cmds) {
        if (!c.is_unpack()) continue;
        ps2::VifUnpack u = c.unpack();
        if (u.addr == 0xC0 && u.count == 1) {
            out.tag = ps2::GifTag::decode(u.data);
            out.prim = ps2::Prim::decode(out.tag.prim);
        } else if (u.addr == 0x00 && u.vn == 2 && u.vl == 0) {
            pos = ps2::unpack_floats(u);
        } else if (u.addr == 0x40 && u.vn == 0 && u.vl == 2) {
            std::vector<std::array<u32, 4>> w = ps2::unpack_words(u);
            u32 n = w.empty() ? 0 : w[0][0];
            for (u32 i = 1; i <= n && i < w.size(); ++i) out.strips.push_back(w[i][0] / 3 + 2);
        } else if (u.addr == 0x80 && u.vn == 2 && u.vl == 0) {
            nrm = ps2::unpack_floats(u);
        } else if (u.addr == 0x80 && u.vn == 3 && u.vl == 2) {
            col = ps2::unpack_words(u);
        } else if (u.addr == 0x40 && u.vn == 1 && u.vl == 0) {
            uv = ps2::unpack_floats(u);
        } else if (u.addr == 0x40 && u.vn == 2 && u.vl == 0) {
            envn = ps2::unpack_floats(u);
        } else if (u.addr == 0x20 || u.addr == 0x60 || u.addr == 0xA0) {
            out.morph = true; // second key frame
        }
    }
    size_t n = pos.size();
    u32 total = 0;
    for (u32 s : out.strips) total += s;
    if (total != n) throw FormatError("GTM1 packet strip lengths do not match vertex count");
    auto check = [&](size_t sz) { if (sz && sz != n) throw FormatError("GTM1 packet attribute count mismatch"); };
    check(nrm.size()), check(col.size()), check(uv.size()), check(envn.size());
    out.has_normals = !nrm.empty();
    out.has_colors = !col.empty();
    out.has_uv = !uv.empty();
    out.env_normals = !envn.empty();
    out.vertices.resize(n);
    for (size_t i = 0; i < n; ++i) {
        scene::Vertex& v = out.vertices[i];
        for (int k = 0; k < 3; ++k) v.pos[k] = pos[i][k];
        if (!nrm.empty()) for (int k = 0; k < 3; ++k) v.normal[k] = nrm[i][k];
        else if (!envn.empty()) for (int k = 0; k < 3; ++k) v.normal[k] = envn[i][k];
        if (!uv.empty()) v.uv[0] = uv[i][0], v.uv[1] = uv[i][1];
        if (!col.empty()) {
            v.color = expand_color(col[i]);
            out.raw_rgb_max = std::max({out.raw_rgb_max, col[i][0] & 0xFF, col[i][1] & 0xFF, col[i][2] & 0xFF});
        }
    }
    return out;
}

namespace {

// 4/8-bit indexed texture with a 32-bit CSM1 CLUT, honouring TEX0.CSA (the
// CLUT entry offset in units of 16). GsMemory::decode_rgba ignores CSA, and
// car paint variants depend on it.
std::vector<u32> decode_indexed_csa(const GsMemory& gs, const Tex0& t, u32 width, u32 height) {
    bool is4 = t.psm == u32(Psm::T4) || t.psm == u32(Psm::T4HL) || t.psm == u32(Psm::T4HH);
    u32 n = is4 ? 16 : 256;
    u32 palette[256];
    for (u32 i = 0; i < n; ++i) {
        u32 e = (is4 ? t.csa * 16 : 0) + i; // position in the 256-entry CSM1 arrangement
        u32 x = (e & 7) | ((e & 0x10) >> 1);
        u32 y = ((e >> 5) << 1) | ((e >> 3) & 1);
        u32 c = gs.read_texel(t.cbp, 1, u32(Psm::CT32), x, y);
        u32 a = std::min<u32>(255, ((c >> 24) * 255 + 64) / 128);
        palette[i] = (c & 0x00FFFFFFu) | (t.cpsm == u32(Psm::CT24) ? 0xFF000000u : a << 24);
    }
    std::vector<u32> out(size_t(width) * height);
    for (u32 y = 0; y < height; ++y)
        for (u32 x = 0; x < width; ++x) {
            u32 c = palette[gs.read_texel(t.tbp, t.tbw, t.psm, x, y) & (n - 1)];
            out[size_t(y) * width + x] = t.tcc ? c : (c | 0xFF000000u);
        }
    return out;
}

bool indexed_ct32(const Tex0& t) {
    bool idx = t.psm == u32(Psm::T4) || t.psm == u32(Psm::T4HL) || t.psm == u32(Psm::T4HH) ||
               t.psm == u32(Psm::T8) || t.psm == u32(Psm::T8H);
    return idx && (t.cpsm == u32(Psm::CT32) || t.cpsm == u32(Psm::CT24)) && t.csm == 0;
}

} // namespace

std::vector<PaintVariant> paint_variants(ByteSpan tex1) {
    std::vector<PaintVariant> out;
    if (!Tex1::is_tex1(tex1)) return out;
    u32 table = rd32(tex1, 0x20);
    if (table == 0) return out;
    u32 v = rd32(tex1, table);
    for (u32 i = 0; i < v; ++i) {
        u32 off = rd32(tex1, table + 4 + 4 * i);
        u32 n = rd32(tex1, off) & 0xFFFF;
        PaintVariant pv;
        for (u32 k = 0; k < n && off + 8 + 4 * k <= tex1.size(); ++k) {
            u32 e = rd32(tex1, off + 4 + 4 * k);
            pv.clut.push_back({e >> 23, (e >> 5) & 0x3FFF, e & 31, (e >> 19) & 0xF});
        }
        out.push_back(std::move(pv));
    }
    return out;
}

int decode_texture_set(ByteSpan tex1, std::vector<scene::Texture>& out, const std::string& prefix, int paint) {
    Tex1 t = Tex1::parse(tex1);
    GsMemory gs;
    t.upload(tex1, gs);
    std::vector<PaintVariant> variants;
    if (paint > 0) variants = paint_variants(tex1);
    const PaintVariant* pv = paint > 0 && size_t(paint) < variants.size() ? &variants[paint] : nullptr;
    int first = int(out.size());
    for (size_t i = 0; i < t.textures.size(); ++i) {
        const Tex1::Texture& tx = t.textures[i];
        Tex0 t0 = tx.tex0;
        if (pv)
            for (const PaintVariant::Entry& e : pv->clut)
                if (e.texture == i) t0.cbp = e.cbp, t0.csa = e.csa, t0.cpsm = e.cpsm;
        scene::Texture st;
        st.name = prefix + std::to_string(i);
        // Always the full power-of-two size: model UVs address the whole texture.
        st.width = t0.width();
        st.height = t0.height();
        st.rgba = indexed_ct32(t0) ? decode_indexed_csa(gs, t0, st.width, st.height)
                                   : gs.decode_rgba(t0, st.width, st.height);
        st.clamp_u = tx.clamp.wms == 1 || tx.clamp.wms == 2;
        st.clamp_v = tx.clamp.wmt == 1 || tx.clamp.wmt == 2;
        out.push_back(std::move(st));
    }
    return first;
}

namespace {

// True when some texel is less than half opaque (PS2 alpha < 0x40).
// GS alpha test of one texel (alpha expanded to 0..255; GS units are 0x80 = 1.0).
bool gs_alpha_pass(u32 a255, u8 method, u8 ref) {
    u32 a = (a255 * 128 + 127) / 255;
    switch (method & 7) {
    case 0: return false;
    case 1: return true;
    case 2: return a < ref;
    case 3: return a <= ref;
    case 4: return a == ref;
    case 5: return a >= ref;
    case 6: return a > ref;
    default: return a != ref;
    }
}

u32 texel_alpha(const scene::Texture& t, float u, float v) {
    if (t.width == 0 || t.height == 0) return 255;
    s32 x = s32(std::floor(u * float(t.width))) % s32(t.width);
    s32 y = s32(std::floor(v * float(t.height))) % s32(t.height);
    if (x < 0) x += s32(t.width);
    if (y < 0) y += s32(t.height);
    return t.rgba[size_t(y) * t.width + size_t(x)] >> 24;
}

// Frame-buffer alpha a pass left at a vertex position (see build_model).
struct Written {
    u32 weight = 0;   // texel alpha x vertex alpha at this vertex (0..255)
    int texture = -1; // the pass's texture (its alpha is the per-pixel reflectivity)
    float uv[2] = {0, 0};
    u32 vertex_alpha = 255;
};

bool texture_has_holes(const scene::Texture& t) {
    for (u32 c : t.rgba)
        if ((c >> 24) < 128) return true;
    return false;
}

} // namespace

scene::Model build_model(const Gtm1& g, const Options& opt, const std::string& name) {
    scene::Model model;
    model.name = name;
    std::map<u32, int> set_first; // texset -> first model texture index (-1 = failed)
    std::map<u32, u32> set_size;
    auto texture_for = [&](u32 texset, u32 index) -> int {
        if (!opt.textures || texset >= g.texsets.size()) return -1;
        auto it = set_first.find(texset);
        if (it == set_first.end()) {
            ByteSpan sub = g.data.subspan(g.texsets[texset]);
            size_t before = model.textures.size();
            int first = decode_texture_set(sub, model.textures, "set" + std::to_string(texset) + "_", opt.paint);
            it = set_first.emplace(texset, first).first;
            set_size[texset] = u32(model.textures.size() - before);
        }
        return index < set_size[texset] ? it->second + int(index) : -1;
    };

    std::map<int, u32> min_alpha; // texture -> smallest texel alpha (gs_state alpha test)
    // Alpha-tested copies of textures: (texture, method, ref) -> texture with binary alpha.
    std::map<std::array<int, 3>, int> cutouts;
    auto cutout_for = [&](int tex, u8 method, u8 ref) -> int {
        std::array<int, 3> key{tex, method, ref};
        auto it = cutouts.find(key);
        if (it != cutouts.end()) return it->second;
        scene::Texture t = model.textures[size_t(tex)];
        bool all_pass = true;
        for (u32& c : t.rgba) {
            bool pass = gs_alpha_pass(c >> 24, method, ref);
            all_pass &= pass;
            c = (c & 0x00FFFFFFu) | (pass ? 0xFF000000u : 0u);
        }
        int idx = tex; // nothing cut away: keep the original
        if (!all_pass) {
            t.name += "_cut";
            model.textures.push_back(std::move(t));
            idx = int(model.textures.size() - 1);
        }
        cutouts.emplace(key, idx);
        return idx;
    };

    // Reflection pass (ALPHA Cs*Ad + Cd) over vertices that earlier passes wrote: the
    // triangles whose three corners come from the same textured pass become a primitive
    // with that texture and its UVs, so the renderer can weight the reflection per pixel
    // with the texel alpha (holes = 0, no reflection). Other triangles keep a per-vertex
    // weight in the vertex alpha. All are kAdditive | kEnvironmentMap | kUnlit | kNoDepthWrite.
    auto emit_weighted_reflection = [](scene::Mesh& mesh, const scene::Primitive& prim,
                                       const std::map<std::array<float, 3>, Written>& written) {
        constexpr u32 kFlags = scene::kAdditive | scene::kEnvironmentMap | scene::kUnlit | scene::kNoDepthWrite;
        std::vector<u32> tris(mesh.indices.begin() + prim.first_index,
                              mesh.indices.begin() + prim.first_index + prim.index_count);
        mesh.indices.resize(prim.first_index);
        auto info = [&](u32 vi) -> const Written* {
            const float* p = mesh.vertices[vi].pos;
            auto it = written.find({p[0], p[1], p[2]});
            return it == written.end() ? nullptr : &it->second;
        };
        std::map<int, std::vector<u32>> groups; // texture -> indices
        for (size_t t = 0; t + 2 < tris.size(); t += 3) {
            const Written* w[3] = {info(tris[t]), info(tris[t + 1]), info(tris[t + 2])};
            bool same = w[0] && w[1] && w[2] && w[0]->texture >= 0 && w[1]->texture == w[0]->texture &&
                        w[2]->texture == w[0]->texture;
            std::vector<u32>& out = groups[same ? w[0]->texture : -1];
            for (int k = 0; k < 3; ++k) {
                u32 vi = tris[t + k];
                if (!same) { // per-vertex weight on the shared vertex
                    u32 a = w[k] ? std::min<u32>(255, w[k]->weight) : 0u;
                    mesh.vertices[vi].color = (mesh.vertices[vi].color & 0x00FFFFFFu) | (a << 24);
                    out.push_back(vi);
                    continue;
                }
                scene::Vertex v = mesh.vertices[vi]; // own copy: UVs of the base pass
                v.uv[0] = w[k]->uv[0], v.uv[1] = w[k]->uv[1];
                v.color = (v.color & 0x00FFFFFFu) | (std::min<u32>(255, w[k]->vertex_alpha) << 24);
                out.push_back(u32(mesh.vertices.size()));
                mesh.vertices.push_back(v);
            }
        }
        for (auto& [tex, idx] : groups) {
            scene::Primitive p;
            p.first_index = u32(mesh.indices.size());
            p.index_count = u32(idx.size());
            p.texture = tex;
            p.flags = kFlags;
            p.env_rows = prim.env_rows;
            std::copy(std::begin(prim.env_c0), std::end(prim.env_c0), p.env_c0);
            std::copy(std::begin(prim.env_c1), std::end(prim.env_c1), p.env_c1);
            mesh.indices.insert(mesh.indices.end(), idx.begin(), idx.end());
            if (p.index_count) mesh.primitives.push_back(p);
        }
    };

    for (u32 node = 0; node < g.nodes.size(); ++node) {
        if (opt.node >= 0 && u32(opt.node) != node) continue;
        std::vector<Draw> draws = g.run(node, opt.run);
        // What earlier passes left in the frame buffer's alpha, by vertex position: the
        // reflectivity that later Cs*Ad+Cd reflection passes are weighted with.
        std::map<std::array<float, 3>, Written> written_alpha;
        if (draws.empty()) continue;
        scene::Mesh mesh;
        mesh.name = "node" + std::to_string(node);
        mesh.lod = opt.run.lod;
        // Base-pass vertex alpha by position, for the road sheen mask.
        std::map<std::array<float, 3>, u32> base_alpha;
        if (opt.road_sheen)
            for (const Draw& d : draws) {
                if ((g.chunks[d.chunk].flags & 4) || d.fb_mask) continue;
                for (const Packet& pk : g.chunks[d.chunk].packets) {
                    DecodedPacket bp = g.decode_packet(pk);
                    // ALPHA 0x58 only matters with blending on (PRIM.ABE): an opaque packet drawn
                    // while the register still holds 0x58 is an ordinary base pass.
                    if (d.alpha_mode == 0x58 && bp.prim.abe) continue;
                    for (const scene::Vertex& v : bp.vertices) base_alpha[{v.pos[0], v.pos[1], v.pos[2]}] = v.color >> 24;
                }
            }
        // static_env_maps: the last alpha-only textured pass (a destination-alpha mask) as a
        // white texture carrying the mask alpha plus one triangle of its geometry for the
        // planar position -> UV mapping.
        struct AlphaMask {
            int texture = -1;
            float pos[3][3] = {};
            float uv[3][2] = {};
        } mask;
        auto mask_uv = [&](const float* p, float* uv) {
            float e1[3], e2[3], d[3];
            for (int k = 0; k < 3; ++k)
                e1[k] = mask.pos[1][k] - mask.pos[0][k], e2[k] = mask.pos[2][k] - mask.pos[0][k], d[k] = p[k] - mask.pos[0][k];
            auto dot = [](const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
            float a11 = dot(e1, e1), a12 = dot(e1, e2), a22 = dot(e2, e2), b1 = dot(d, e1), b2 = dot(d, e2);
            float det = a11 * a22 - a12 * a12;
            float s = det != 0 ? (b1 * a22 - b2 * a12) / det : 0, t = det != 0 ? (b2 * a11 - b1 * a12) / det : 0;
            for (int k = 0; k < 2; ++k)
                uv[k] = mask.uv[0][k] + s * (mask.uv[1][k] - mask.uv[0][k]) + t * (mask.uv[2][k] - mask.uv[0][k]);
        };
        auto place = [](const Draw& d, float* p) {
            if (!d.has_matrix) return;
            const Matrix& m = d.matrix;
            float q[3];
            for (int j = 0; j < 3; ++j) q[j] = p[0] * m[j] + p[1] * m[4 + j] + p[2] * m[8 + j] + m[12 + j];
            for (int j = 0; j < 3; ++j) p[j] = q[j];
        };
        for (const Draw& d : draws) {
            // Alpha-only frame buffer passes (shadow masks) produce no colour.
            const bool alpha_only = (d.fb_mask & 0x00FFFFFFu) == 0x00FFFFFFu;
            if (alpha_only && !opt.static_env_maps && !opt.frame_alpha) continue;
            if (alpha_only && !opt.frame_alpha) {
                for (const Packet& pk : g.chunks[d.chunk].packets) {
                    u32 tex = pk.mat & 0x1FF;
                    DecodedPacket dp = g.decode_packet(pk);
                    if (!dp.prim.tme || !dp.has_uv || tex == 0 || tex == 0x1FF || dp.vertices.size() < 3) continue;
                    int src = texture_for(d.texset, tex - 1);
                    if (src < 0) continue;
                    scene::Texture t = model.textures[size_t(src)];
                    for (u32& c : t.rgba) c |= 0x00FFFFFFu;
                    t.name += "_mask";
                    model.textures.push_back(std::move(t));
                    mask.texture = int(model.textures.size() - 1);
                    for (int k = 0; k < 3; ++k) {
                        std::memcpy(mask.pos[k], dp.vertices[size_t(k)].pos, 12);
                        place(d, mask.pos[k]);
                        mask.uv[k][0] = dp.vertices[size_t(k)].uv[0], mask.uv[k][1] = dp.vertices[size_t(k)].uv[1];
                    }
                    break;
                }
                continue;
            }
            for (const Packet& pk : g.chunks[d.chunk].packets) {
                u32 tex = pk.mat & 0x1FF;       // texture index + 1, 0x1FF = runtime slot
                u32 mat = pk.mat >> 9;          // material index + 1, 0x7F = runtime slot
                DecodedPacket dp = g.decode_packet(pk);
                // Road sheen: the chunk-flag-4 pass, or a blended packet drawn with ALPHA 0x58
                // (Cs*Ad + Cd). Opaque packets drawn while ALPHA still holds 0x58 (e.g. the
                // Laguna / Rome road base passes) are ordinary opaque geometry.
                bool sheen = opt.road_sheen && ((g.chunks[d.chunk].flags & 4) || (d.alpha_mode == 0x58 && dp.prim.abe));
                // Reflection passes carry a normal in the UV slot; the VU derives the env-map coordinates.
                bool env = dp.env_normals;
                if (env && !opt.reflection) continue;
                bool weighted = opt.weighted_reflection && env && d.alpha_mode == 0x58;
                u32 base = u32(mesh.vertices.size());
                for (scene::Vertex v : dp.vertices) {
                    if (sheen) {
                        auto it = base_alpha.find({v.pos[0], v.pos[1], v.pos[2]});
                        v.color = (v.color & 0x00FFFFFFu) | ((it != base_alpha.end() ? it->second : 0u) << 24);
                    }
                    if (d.has_matrix) {
                        const Matrix& m = d.matrix;
                        float p[3], n[3];
                        for (int j = 0; j < 3; ++j) {
                            p[j] = v.pos[0] * m[j] + v.pos[1] * m[4 + j] + v.pos[2] * m[8 + j] + m[12 + j];
                            n[j] = v.normal[0] * m[j] + v.normal[1] * m[4 + j] + v.normal[2] * m[8 + j];
                        }
                        float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                        for (int j = 0; j < 3; ++j) v.pos[j] = p[j], v.normal[j] = len > 0 ? n[j] / len : 0;
                    }
                    mesh.vertices.push_back(v);
                }
                scene::Primitive prim;
                prim.first_index = u32(mesh.indices.size());
                u32 strip_start = base;
                for (u32 len : dp.strips) {
                    for (u32 i = 0; i + 2 < len; ++i) {
                        u32 a = strip_start + i, b = strip_start + i + 1, c = strip_start + i + 2;
                        if (i & 1) std::swap(a, b);
                        const float* pa = mesh.vertices[a].pos;
                        const float* pb = mesh.vertices[b].pos;
                        const float* pc = mesh.vertices[c].pos;
                        if (std::memcmp(pa, pb, 12) == 0 || std::memcmp(pb, pc, 12) == 0 || std::memcmp(pa, pc, 12) == 0)
                            continue; // degenerate stitching triangle
                        mesh.indices.insert(mesh.indices.end(), {a, b, c});
                    }
                    strip_start += len;
                }
                prim.index_count = u32(mesh.indices.size()) - prim.first_index;
                if (dp.prim.abe) prim.flags |= scene::kAlphaBlend;
                // ALPHA 0x89 = (Cd - 0) * As + 0: darkens what is behind by the fragment alpha.
                if (dp.prim.abe && d.alpha_mode == 0x89) prim.flags = scene::kMultiplyDest | scene::kNoDepthWrite;
                if (env) {
                    prim.flags |= scene::kEnvironmentMap;
                    prim.env_rows = true;
                    for (int k = 0; k < 3; ++k) prim.env_c0[k] = d.env_c0[size_t(k)], prim.env_c1[k] = d.env_c1[size_t(k)];
                }
                if (dp.has_colors && !dp.has_normals && !env) prim.flags |= scene::kUnlit;
                // Lit packets: the VU lights them with the selected paint's material (GL
                // defaults when the packet has none); chunk header byte 9 bit 0 = 255 scale.
                if (dp.has_normals && !dp.has_colors && !env) {
                    const std::vector<Material>& mats = g.materials_for(opt.paint);
                    scene::Material& sm = prim.material;
                    sm.valid = true;
                    if (mat != 0 && mat != 0x7F && mat - 1 < mats.size()) {
                        const Material& m = mats[mat - 1];
                        for (int k = 0; k < 3; ++k) {
                            sm.ambient[k] = m.rows[0][k], sm.diffuse[k] = m.rows[1][k];
                            sm.specular[k] = m.rows[2][k];
                            // Lamp materials: emission x (lamps on / 4), applied per draw.
                            sm.emission[k] = (m.flags & 1) ? 0.0f : m.rows[3][k];
                            sm.lamp_emission[k] = (m.flags & 1) ? m.rows[3][k] : 0.0f;
                        }
                        sm.shininess = m.shininess;
                    }
                    if ((g.chunks[d.chunk].flags >> 8) & 1) sm.scale = 255.0f / 128.0f;
                }
                // 0x1FF is a runtime texture slot (op 0x29; the game fills it, e.g. the wheel rim image);
                // the set's first texture is the closest static stand-in.
                if (dp.prim.tme && dp.has_uv && tex != 0 && !env)
                    prim.texture = texture_for(d.texset, tex == 0x1FF ? 0 : tex - 1);
                if (opt.static_env_maps) {
                    // Env-map pass whose sphere map is in the file (prize scenes): bind it; the
                    // renderer derives the UVs from the view-space normal.
                    if (env && dp.prim.tme && tex != 0 && tex != 0x1FF) prim.texture = texture_for(d.texset, tex - 1);
                    // ALPHA 0x48 (Cs*As + Cd) and 0x58 (Cs*Ad + Cd) add to the frame buffer.
                    if (dp.prim.abe && (d.alpha_mode == 0x48 || d.alpha_mode == 0x58))
                        prim.flags = (prim.flags & ~u32(scene::kAlphaBlend)) | scene::kAdditive | scene::kNoDepthWrite;
                    // ALPHA 0x54 ((Cs - Cd) * Ad + Cd) over a destination-alpha mask (the license
                    // card emblem): drawn as the mask texture (white, mask alpha) on the pass's
                    // geometry, coloured with the pass texture's mean colour (its env-map shine
                    // is constant over a flat face).
                    if (dp.prim.abe && d.alpha_mode == 0x54 && mask.texture >= 0) {
                        u32 tint = 0xFFFFFFFFu;
                        if (prim.texture >= 0) {
                            const scene::Texture& t = model.textures[size_t(prim.texture)];
                            u64 sum[3] = {0, 0, 0};
                            for (u32 c : t.rgba)
                                for (int k = 0; k < 3; ++k) sum[k] += (c >> (8 * k)) & 0xFF;
                            size_t n = std::max<size_t>(1, t.rgba.size());
                            tint = 0xFF000000u | u32(sum[0] / n) | u32(sum[1] / n) << 8 | u32(sum[2] / n) << 16;
                        }
                        for (u32 vi = base; vi < mesh.vertices.size(); ++vi) {
                            scene::Vertex& v = mesh.vertices[vi];
                            mask_uv(v.pos, v.uv);
                            u32 out = 0xFF000000u;
                            for (int k = 0; k < 3; ++k)
                                out |= (((v.color >> (8 * k)) & 0xFF) * ((tint >> (8 * k)) & 0xFF) / 255) << (8 * k);
                            v.color = out;
                        }
                        prim.texture = mask.texture;
                        prim.flags = scene::kAlphaBlend | scene::kUnlit;
                    }
                }
                // Frame-buffer alpha this pass leaves behind (texel alpha x vertex alpha): the weight
                // for later reflection passes over the same vertices. Passes with the alpha channel
                // write-masked (car glass) leave whatever lies behind them, which a per-vertex model
                // cannot know; their own alpha is used as the estimate unless a real write exists.
                if (opt.weighted_reflection && !env) {
                    bool masked = (d.fb_mask & 0xFF000000u) != 0;
                    for (u32 vi = base; vi < mesh.vertices.size(); ++vi) {
                        const scene::Vertex& v = mesh.vertices[vi];
                        Written w;
                        w.vertex_alpha = v.color >> 24;
                        w.weight = w.vertex_alpha;
                        w.texture = prim.texture;
                        w.uv[0] = v.uv[0], w.uv[1] = v.uv[1];
                        if (prim.texture >= 0)
                            w.weight = w.weight * texel_alpha(model.textures[size_t(prim.texture)], v.uv[0], v.uv[1]) / 255;
                        std::array<float, 3> key{v.pos[0], v.pos[1], v.pos[2]};
                        if (masked) written_alpha.emplace(key, w);
                        else written_alpha[key] = w;
                    }
                }
                if (opt.program_alpha_test && d.alpha_test && prim.texture >= 0 && !dp.prim.abe) {
                    int cut = cutout_for(prim.texture, d.alpha_test_method, d.alpha_test_ref);
                    if (cut != prim.texture) prim.texture = cut, prim.flags |= scene::kAlphaTest;
                }
                bool gs_tested = false;
                if (opt.gs_state) {
                    if (!d.depth_write) prim.flags |= scene::kNoDepthWrite;
                    if (!d.cull) prim.flags |= scene::kDoubleSided;
                    // GS alpha test of the fragment alpha (texel x vertex alpha, 0x80 = 1.0):
                    // GREATER AREF / GEQUAL AREF as a "discard alpha <= ref" threshold.
                    u8 m = d.alpha_test_method;
                    if (d.alpha_test && prim.texture >= 0 && (m == 5 || m == 6)) {
                        float ref = (float(d.alpha_test_ref) + (m == 6 ? 0.5f : -0.5f)) / 128.0f;
                        if (ref < 0) {
                            gs_tested = true; // GEQUAL 0: every fragment passes
                        } else if (dp.prim.abe && d.alpha_fail == 3) {
                            prim.flags |= scene::kAlphaTest | scene::kAlphaFailBlend;
                            prim.alpha_ref = ref;
                            gs_tested = true;
                        } else if (d.alpha_fail == 0) {
                            // Only where something can fail (keeps early depth for solid packets).
                            u32 cut = u32(ref * 255.0f);
                            auto mt = min_alpha.find(prim.texture);
                            if (mt == min_alpha.end()) {
                                u32 lo = 255;
                                for (u32 c : model.textures[size_t(prim.texture)].rgba) lo = std::min(lo, c >> 24);
                                mt = min_alpha.emplace(prim.texture, lo).first;
                            }
                            bool fails = mt->second <= cut;
                            for (u32 vi = base; vi < mesh.vertices.size() && !fails; ++vi)
                                fails = (mesh.vertices[vi].color >> 24) <= cut;
                            if (fails) prim.flags |= scene::kAlphaTest, prim.alpha_ref = ref;
                            gs_tested = true;
                        }
                    }
                }
                // Textures with transparent texels: the GS alpha test keeps those pixels out of the
                // colour and depth buffers (foliage cut-outs, fences, decal edges).
                if (!gs_tested && opt.alpha_test && prim.texture >= 0 &&
                    texture_has_holes(model.textures[size_t(prim.texture)]))
                    prim.flags |= scene::kAlphaTest;
                if (mat != 0 && mat != 0x7F && mat - 1 < g.materials.size() && (g.materials[mat - 1].flags & 1)) {
                    prim.flags |= kTintable;
                    if (opt.tint)
                        for (u32 vi = base; vi < mesh.vertices.size(); ++vi) {
                            u32& c = mesh.vertices[vi].color;
                            u32 out = c & 0xFF000000u;
                            for (int k = 0; k < 3; ++k) {
                                float ch = float((c >> (8 * k)) & 0xFF) * std::clamp((*opt.tint)[k], 0.f, 1.f);
                                out |= u32(ch + 0.5f) << (8 * k);
                            }
                            c = out;
                        }
                }
                if (sheen) prim.flags = (prim.flags & ~u32(scene::kAlphaBlend | scene::kUnlit)) | scene::kAdditive |
                                        scene::kEnvironmentMap | scene::kNoDepthWrite;
                if (opt.frame_alpha) {
                    if (alpha_only) { // writes only the frame alpha (texel x vertex alpha)
                        // (an alpha test only when the node program enables one: not the generic
                        // cut-out guess, which would drop the masks' low-alpha texels)
                        u32 keep = scene::kDoubleSided | (prim.alpha_ref >= 0 ? scene::kAlphaTest : 0u);
                        prim.flags = (prim.flags & keep) | scene::kAlphaOnly | scene::kUnlit | scene::kNoDepthWrite;
                    }
                    else if ((d.fb_mask & 0xFF000000u) == 0xFF000000u)
                        prim.flags |= scene::kKeepAlpha;
                }
                if (weighted) {
                    emit_weighted_reflection(mesh, prim, written_alpha);
                    continue;
                }
                if (prim.index_count) mesh.primitives.push_back(prim);
            }
        }
        if (!mesh.primitives.empty()) model.meshes.push_back(std::move(mesh));
    }
    model.compute_bounds();
    return model;
}

scene::Model decode_gtm1(ByteSpan gtm1, const Options& opt, const std::string& name) {
    return build_model(Gtm1::parse(gtm1), opt, name);
}

} // namespace gt3r::gtm1
