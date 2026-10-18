#pragma once

// "GTM1" model container: the PS2 geometry format shared by car bodies
// (cars/<tod>/<id>), wheels (wheel/<id>, "GTTW" wrapper) and course meshes.
// See docs/formats/car_model.md for the full layout. All offsets below are
// relative to the "GTM1" magic.
//
//   0x10 u16 node count          0x20 u32 node program offsets[node count]
//   0x12 u16 chunk count         0x24 u32 chunk offsets[chunk count]
//   0x14 u16 material count      0x28 u32 material block (0x50 bytes each)
//   0x16 u16 texture set count   0x2C u32 Tex1 set offsets[texture set count]
//   0x18 u16 texture LOD count   0x30 u32 bounding spheres (16 bytes per node)
//                                0x34 u32 per-texture-LOD set tables (or 0)
//                                0x3C u32 texture animation list (or 0)
//
// Node programs are a small bytecode (draw chunk, LOD switch, bbox culling,
// matrix stack, render state, texture set select). Chunks are lists of VIF
// packets that feed the VU1 renderer: float positions, strip lengths,
// normals or colours, float ST coordinates, plus a GIF tag.

#include "formats/ps2/vif.h"
#include "scene/model.h"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace gt3r::gtm1 {

using Matrix = std::array<float, 16>; // row-vector convention: v' = v * M (rows r0..r3)

struct Packet {
    u32 offset = 0; // relative to the GTM1 start
    u32 qwc = 0;
    u16 mat = 0;    // bits 0-8: texture index + 1 in the current set (0 = none, 0x1FF = runtime slot);
                    // bits 9-15: material index + 1 (0 = none, 0x7F = runtime slot)
};

// 0x50-byte material record: rows ambient, diffuse, specular, emission (vec4 each),
// +0x40 shininess, +0x44 flags (bit 0 = lamp: the car scales its rows by a global
// tint, emission x lamps on / 4).
struct Material {
    float rows[4][4] = {};
    float shininess = 0;
    u32 flags = 0;
};

// Primitive flag set by build_model on packets whose material has flag bit 0
// (tinted at run time by the car colour).
constexpr u32 kTintable = scene::kTintable;

struct Chunk {
    u32 offset = 0;
    u16 flags = 0;  // low byte: vertex layout / pass type, high byte: flags (see docs)
    u16 vertex_count = 0;
    u16 triangle_count = 0;
    std::vector<Packet> packets;
};

// One draw issued by a node program.
struct Draw {
    u32 chunk = 0;
    u32 texset = 0;      // index into Gtm1::texsets
    Matrix matrix{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    bool has_matrix = false;
    // Render state at the time of the draw (node program ops):
    //   op 0x20 u8 mode, u8 fix: GS ALPHA register (A|B<<2|C<<4|D<<6, FIX).
    //     0x44 = (Cs-Cd)*As+Cd (normal), 0x58 = Cs*Ad+Cd (additive, weighted by
    //     the frame buffer's alpha; used by the course road sheen passes).
    //   op 0x24 u32: GS FBMSK (frame buffer write mask, bit set = not written).
    //     0x00FFFFFF = alpha-only pass (course shadow masks that only feed the
    //     sheen weighting), 0xFF000000 = colour-only (decals keep the alpha).
    u8 alpha_mode = 0x44;
    u32 fb_mask = 0;
    //   ops 0x36 f / 0x37 vec4: reflection row C0 (f -> (f, f, f, 1)); 0x32 f / 0x33 vec4:
    //   row C1 (f -> (f, f, f, 0)). Reflection (env-normal) packets are weighted per vertex
    //   by C0 + C1 min(V.N, 0) (V the unit view vector). C0 = 1, C1 = 0 on node entry here.
    std::array<float, 4> env_c0{1, 1, 1, 1}, env_c1{0, 0, 0, 0};
    //   ops 0x1A / 0x1B: alpha test on / off (GS TEST.ATE; render flag 4).
    //   op 0x1C u8 method, u8 ref: TEST.ATST / AREF (ref in GS units, 0x80 = 1.0).
    //   ops 0x1D / 0x1E: destination alpha test on / off (TEST.DATE; render flag 3).
    // State on entry to a node: ATE on, (ATST 6 = GREATER, AREF 0x20), DATE off (the
    // car and wheel programs restore exactly this before they end).
    bool alpha_test = true;
    u8 alpha_test_method = 6;
    u8 alpha_test_ref = 0x20;
    bool dest_alpha_test = false;
    //   op 0x2F u8: TEST.AFAIL, what a pixel failing the alpha test still writes:
    //     0 KEEP (nothing), 1 FB_ONLY (RGBA), 2 ZB_ONLY (depth), 3 RGB_ONLY (colour,
    //     no alpha, no depth). Course foliage: ABE + GREATER 0x7F + RGB_ONLY, i.e. the
    //     opaque texels write depth, the translucent edges blend without it.
    u8 alpha_fail = 0;
    //   ops 0x25 / 0x26: depth write off / on (ZBUF.ZMSK).
    bool depth_write = true;
    //   ops 0x17 / 0x18: depth test on / off (render state 5; 0x17 only when the global
    //     depth-test flag is set).
    bool depth_test = true;
    //   ops 0x2D / 0x2E: back-face culling on / off (render state 0xF; on at stream entry).
    //     Course flags and banners turn it off around their animated chunks.
    bool cull = true;
};

struct RunParams {
    int lod = 0;          // op 0x07 LOD switch branch (clamped to the branch count)
    int selector = 0;     // op 0x06 switch value (out-of-range = take no branch)
    // Op 0x06 switches with selector id 0 (the u16 after the opcode) read the car's
    // "brake pressed now" state instead: branch 1 = brake lamps lit (GT3's lens swap).
    bool brake = false;
    int texture_lod = 0;  // per-texture-LOD set table (0x34), clamped
};

// A decoded VIF packet, vertices in strip order.
struct DecodedPacket {
    std::vector<scene::Vertex> vertices;
    std::vector<u32> strips;       // vertex count per triangle strip
    ps2::GifTag tag;
    ps2::Prim prim;
    bool has_normals = false;      // V3-32 at 0x80 (lit)
    bool has_colors = false;       // V4-8 at 0x80 (pre-lit)
    bool has_uv = false;           // V2-32 at 0x40
    bool env_normals = false;      // V3-32 at 0x40 (reflection pass: normal used for the env map)
    bool morph = false;            // a second key frame is present (ignored; key 0 decoded)
    u32 raw_rgb_max = 0;           // largest raw vertex colour channel (0x80 = 1.0; up to 0xFF = 2.0)
};

class Gtm1 {
public:
    static bool is_gtm1(ByteSpan data);
    // `data` must start at the "GTM1" magic and cover the whole block.
    static Gtm1 parse(ByteSpan data);

    ByteSpan data;
    std::vector<u32> nodes;                 // program offsets
    std::vector<std::array<float, 4>> spheres; // per node: centre xyz, radius
    std::vector<Chunk> chunks;
    std::vector<Material> materials;        // base set (+0x28): used when no paint set applies
    std::vector<std::vector<Material>> paint_materials; // +0x38[colour] (count +0x1A): the set for each paint
    // The material set for paint colour `paint` (the base set when the model has none for it).
    const std::vector<Material>& materials_for(int paint) const {
        return paint >= 0 && size_t(paint) < paint_materials.size() ? paint_materials[size_t(paint)] : materials;
    }
    std::vector<u32> texsets;               // Tex1 offsets (distinct, in table order); 0xFFFFFFFF in tables = empty slot
    std::vector<std::vector<u32>> texset_tables; // [texture LOD][set index] -> index into texsets

    // Executes node `node`'s program and returns its draws (calls 0x02/0x03
    // are followed recursively).
    std::vector<Draw> run(u32 node, const RunParams& params = {}) const;

    DecodedPacket decode_packet(const Packet& p) const;

private:
    void run_at(u32 node, const RunParams& p, std::vector<Matrix>& stack, std::vector<Draw>& out, int depth) const;
};

struct Options {
    RunParams run;
    int node = -1;            // -1 = every node
    bool reflection = true;   // include environment-map packets (normal in the UV slot)
    bool textures = true;     // decode texture sets into Model::textures
    int paint = 0;            // car colour index (Tex1 paint variant), 0 = default
    std::optional<std::array<float, 3>> tint; // multiply vertex colours of tintable packets (RGB 0..1)
    // Course road "sheen": chunks with flag 4 are a second, lit pass over the
    // same vertices as the node's base pass. The game masks it with the base
    // pass's vertex alpha (written to the frame buffer's alpha; the second pass
    // blends with destination alpha). With this set, such packets get the base
    // vertex's alpha copied in (matched by position) and are flagged
    // kAdditive | kEnvironmentMap, which the renderer draws as an additive,
    // view-dependent specular layer (see docs/dev/RENDERING.md). Without it,
    // they are drawn like any other packet (an opaque-looking noisy layer).
    bool road_sheen = false;
    // Flag kAlphaTest on textured packets whose texture has transparent texels
    // (alpha < 0x40 in PS2 units). Courses: tree/fence cut-outs. Not for cars,
    // whose glass uses low-alpha textures.
    bool alpha_test = false;
    // Apply the node program's GS alpha test (ops 0x1A/0x1B/0x1C) to opaque
    // textured packets: texels that fail it become transparent cut-outs
    // (kAlphaTest, on a derived copy of the texture with binary alpha). Car
    // bodies rely on this for wheel arches and grilles (texel alpha 0 = hole,
    // otherwise >= 0x21 = reflectivity).
    bool program_alpha_test = false;
    // Car reflection passes (environment-map packets drawn with ALPHA 0x58 =
    // Cs*Ad + Cd): the game weights the reflection by the frame buffer alpha
    // that the underlying pass wrote, i.e. that pass's texel alpha. With this
    // set, such packets become kAdditive | kEnvironmentMap | kUnlit |
    // kNoDepthWrite primitives: triangles over a single textured base pass get
    // that pass's texture and UVs (texel alpha = per-pixel reflectivity, vertex
    // alpha = the base vertex alpha); the rest are untextured with the weight
    // in the vertex alpha. The env map itself is a runtime slot, so the
    // renderer supplies the reflection colour.
    bool weighted_reflection = false;
    // Prize scenes (.prm): environment-map packets reference a sphere map stored
    // in the file's own texture set. With this set they keep that texture
    // (Primitive::texture, flag kEnvironmentMap, UVs to be derived from the
    // view-space normal), and passes drawn with GS ALPHA 0x48 / 0x58 become
    // kAdditive | kNoDepthWrite. Alpha-only passes (FBMSK 0x00FFFFFF) are kept
    // as a destination-alpha mask: a later ALPHA 0x54 pass becomes an
    // alpha-blended primitive textured with a white copy of the mask texture
    // (name suffix "_mask", UVs from the mask quad's plane), coloured with the
    // mean colour of its own env map (see docs/formats/prm.md).
    bool static_env_maps = false;
    // Courses: map the node program's GS state per packet instead of the generic
    // `alpha_test` guess. Culling off (op 0x2E) -> kDoubleSided; ZMSK (ops 0x25/0x26) -> kNoDepthWrite; an enabled alpha test
    // (TEST.ATE, ATST GREATER/GEQUAL, AREF) on a textured packet -> kAlphaTest with
    // Primitive::alpha_ref = the GS reference; on blended packets with AFAIL RGB_ONLY
    // also kAlphaFailBlend (the failing fragments still blend, without depth write).
    // Packets with the test off keep `alpha_test`'s behaviour.
    bool gs_state = false;
    // Courses drawn with the PS2's frame-buffer alpha (render::DrawItem::frame_alpha): alpha-only
    // passes (FBMSK 0x00FFFFFF, the shadow masks that switch the road sheen off) are kept as
    // scene::kAlphaOnly primitives, colour-only ones (FBMSK 0xFF000000) get scene::kKeepAlpha.
    bool frame_alpha = false;
};

// Turns a parsed GTM1 block into a scene::Model: one Mesh per node that draws
// something (named "node<N>"), matrices baked into the vertices, triangle
// lists with counter-clockwise front faces, UV origin at the texture's top
// left. Textures are decoded per (set, index) actually used.
scene::Model build_model(const Gtm1& g, const Options& opt, const std::string& name = {});

// Convenience: parse + build.
scene::Model decode_gtm1(ByteSpan gtm1, const Options& opt = {}, const std::string& name = {});

// Car paint colours: Tex1 header +0x20 is an offset (from the Tex1 start, 0 =
// none) to { u32 V; u32 variant_offsets[V] }, each variant { u32 n (low 16
// bits); n x u32 e }. Entry e patches three TEX0 fields of one texture record:
// bits 0-4 CSA, bits 5-18 CBP (relative to the set's base, u16 at Tex1 +0x10,
// 0 on disc), bits 19-22 CPSM, bits 23-31 the texture index (not the entry
// position: some variants skip textures). Variant 0 = the TEX0 values. V
// matches the car's colour count in database/carcolor.db. All CLUTs are already
// in the uploaded data.
struct PaintVariant {
    struct Entry {
        u32 texture = 0, cbp = 0, csa = 0, cpsm = 0;
    };
    std::vector<Entry> clut;
};
std::vector<PaintVariant> paint_variants(ByteSpan tex1);

// Appends the textures of one Tex1 set (all of them) to `out`; returns the
// index of the first one. `paint` > 0 selects a colour variant (when the set
// has one). Indexed textures honour TEX0.CSA. Shared by the car/wheel/course loaders.
int decode_texture_set(ByteSpan tex1, std::vector<scene::Texture>& out, const std::string& prefix, int paint = 0);

} // namespace gt3r::gtm1
