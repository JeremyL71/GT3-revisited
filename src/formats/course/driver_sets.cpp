#include "formats/course/driver_sets.h"

#include "formats/course/course_file.h"

#include <algorithm>

namespace gt3r::course {

DriverSets DriverSets::parse(ByteSpan data, const CourseFile& file) {
    DriverSets out;
    for (int c = 0; c < kClasses; ++c) {
        if (!file.find(kFirstSlot + c)) continue;
        ByteSpan s = file.section(data, kFirstSlot + c);
        Set set{};
        std::copy_n(s.begin(), std::min(s.size(), set.size()), set.begin());
        out.by_class[size_t(c)] = set;
    }
    return out;
}

const DriverSets::Set* DriverSets::for_class(int cls) const {
    if (cls < 0 || cls >= kClasses) cls = 0;
    const std::optional<Set>& s = by_class[size_t(cls)];
    return s ? &*s : nullptr;
}

bool DriverSets::empty() const {
    return std::none_of(by_class.begin(), by_class.end(), [](const std::optional<Set>& s) { return s.has_value(); });
}

} // namespace gt3r::course
