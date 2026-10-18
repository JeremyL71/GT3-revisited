#include "disc/core_image.h"

#include "base/gzip.h"

namespace gt3r {

CoreImage CoreImage::decode(ByteSpan core_gt3) {
    if (core_gt3.size() < 6 || rd16(core_gt3, 0) != 0x0101) throw FormatError("CORE.GT3: bad header");
    u32 unpacked_size = rd32(core_gt3, 2);
    Bytes raw = inflate_raw(core_gt3.subspan(6), unpacked_size);
    if (raw.size() < 0x114) throw FormatError("CORE.GT3: too small");

    // Header words: [0x104] segment count (1), [0x108] entry point,
    // [0x10C] load address, [0x110] image size (= unpacked size - 0x114).
    CoreImage img;
    img.entry_point = rd32(raw, 0x108);
    img.load_address = rd32(raw, 0x10C);
    img.image_size = rd32(raw, 0x110);
    img.program.assign(raw.begin() + 0x114, raw.end());
    return img;
}

} // namespace gt3r
