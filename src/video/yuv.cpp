#include "video/yuv.h"

namespace gt3r::video {

std::vector<u32> frame_to_rgba(const Frame& f) {
    std::vector<u32> out(size_t(f.width) * f.height);
    const int cw = f.chroma_width();
    auto clamp = [](int v) { return u32(v < 0 ? 0 : v > 255 ? 255 : v); };
    for (int y = 0; y < f.height; ++y) {
        for (int x = 0; x < f.width; ++x) {
            // 16.16 fixed point BT.601 limited-range coefficients.
            int c = (int(f.y[size_t(y) * f.width + x]) - 16) * 76309;
            int u = int(f.u[size_t(y / 2) * cw + x / 2]) - 128;
            int v = int(f.v[size_t(y / 2) * cw + x / 2]) - 128;
            u32 r = clamp((c + 104597 * v + 32768) >> 16);
            u32 g = clamp((c - 25675 * u - 53279 * v + 32768) >> 16);
            u32 b = clamp((c + 132201 * u + 32768) >> 16);
            out[size_t(y) * f.width + x] = r | g << 8 | b << 16 | 0xFF000000u;
        }
    }
    return out;
}

} // namespace gt3r::video
