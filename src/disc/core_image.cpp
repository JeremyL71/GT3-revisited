#include "disc/core_image.h"

#include "base/gzip.h"

#include <algorithm>
#include <vector>

namespace gt3r {

CoreImage CoreImage::decode(ByteSpan core_gt3) {
    if (core_gt3.size() < 6 || rd16(core_gt3, 0) != 0x0101) throw FormatError("CORE.GT3: bad header");
    u32 unpacked_size = rd32(core_gt3, 2);
    Bytes raw = inflate_raw(core_gt3.subspan(6), unpacked_size);
    if (raw.size() < 0x114) throw FormatError("CORE.GT3: too small");

    // Header words: [0x104] segment count, [0x108] entry point, then each segment: load address, size, its bytes (one
    // segment in GT3's releases: the image right after its header at 0x114). Several segments are laid out as one image
    // from the lowest load address, the gaps between them zero.
    const u32 count = rd32(raw, 0x104);
    if (count == 0 || count > 64) throw FormatError("CORE.GT3: bad segment count");
    struct Segment {
        u32 load, size, at;
    };
    std::vector<Segment> segs;
    size_t at = 0x10C;
    for (u32 k = 0; k < count; ++k) {
        if (at + 8 > raw.size()) throw FormatError("CORE.GT3: segment table past the end");
        const u32 load = rd32(raw, at), size = rd32(raw, at + 4);
        if (at + 8 + size > raw.size()) throw FormatError("CORE.GT3: segment past the end");
        segs.push_back({load, size, u32(at + 8)});
        at += 8 + size;
    }
    u32 lo = ~0u, hi = 0;
    for (const Segment& g : segs) lo = std::min(lo, g.load), hi = std::max(hi, g.load + g.size);
    if (hi <= lo || hi - lo > 0x02000000) throw FormatError("CORE.GT3: segments out of range");
    CoreImage img;
    img.entry_point = rd32(raw, 0x108);
    img.load_address = lo;
    img.image_size = hi - lo;
    img.program.assign(hi - lo, 0);
    for (const Segment& g : segs) std::copy_n(raw.begin() + g.at, g.size, img.program.begin() + (g.load - lo));
    return img;
}

} // namespace gt3r
