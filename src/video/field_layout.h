#pragma once

#include "video/mpeg2_decoder.h"

namespace gt3r::video {

// Most GT3 movies are "field-split": each 640x448 MPEG frame holds the two
// fields of an interlaced 60 Hz picture as two 640x224 halves (top half =
// first field, bottom half = second field, 1/60 s later). The PS2 displays
// them as interlaced fields; on a progressive display we show each half
// line-doubled ("bob"), giving 60 pictures per second. Other movies (the
// ending credits) are ordinary progressive frames. There is no flag in the
// stream, so the layout is detected from the picture content. Survey of all
// 51 files (gt3tool video-survey): 48 field-split (>= 94% of frames have a
// seam ratio > 3; the rest are busy or flat frames), 3 progressive
// (ending_long_full, ending_short_full, menu_bg2; <= 1.6% above 3).

// Discontinuity at the half-height seam: mean |row h/2-1 - row h/2| divided
// by the mean difference of the neighbouring row pairs. Field-split frames
// join the bottom of one picture to the top of the next (large ratio);
// progressive frames are continuous there (~1). Returns 0 for flat rows.
double seam_ratio(const Frame& f);

// Decides once per movie: majority of the first kVotes non-flat frames, by
// seam_ratio() > kSeamThreshold. Until then, the majority so far.
class FieldLayoutDetector {
public:
    static constexpr int kVotes = 16;
    static constexpr double kSeamThreshold = 3.0;

    void add(const Frame& f);
    bool field_split() const { return split_ * 2 > total_; }
    bool decided() const { return total_ >= kVotes; }

private:
    int split_ = 0, total_ = 0;
};

} // namespace gt3r::video
