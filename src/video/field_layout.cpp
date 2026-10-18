#include "video/field_layout.h"

#include <algorithm>
#include <cmath>

namespace gt3r::video {

double seam_ratio(const Frame& f) {
    const int h = f.height / 2;
    if (h < 4 || f.y.empty()) return 0.0;
    auto row_diff = [&](int a, int b) {
        const u8* p = &f.y[size_t(a) * f.width];
        const u8* q = &f.y[size_t(b) * f.width];
        double s = 0;
        for (int x = 0; x < f.width; ++x) s += std::abs(int(p[x]) - int(q[x]));
        return s / f.width;
    };
    double seam = row_diff(h - 1, h);
    double around = (row_diff(h - 3, h - 2) + row_diff(h - 2, h - 1) + row_diff(h, h + 1) + row_diff(h + 1, h + 2)) / 4;
    if (seam < 1.0 && around < 1.0) return 0.0;
    return seam / std::max(around, 0.5);
}

void FieldLayoutDetector::add(const Frame& f) {
    if (decided()) return;
    double r = seam_ratio(f);
    if (r == 0.0) return; // flat
    ++total_;
    if (r > kSeamThreshold) ++split_;
}

} // namespace gt3r::video
