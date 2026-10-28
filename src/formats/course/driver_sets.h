#pragma once

// AI driver parameter sets of a course (course file slots 37-40, no magic; docs/formats/course.md
// "Driver parameter sets"). One section per driveline class: slot 37 + class (0 FR / MR / RR,
// 1 FF and 4WD system 0, 2 other 4WD, 3 4WD system 3). Each section is 0x40 bytes; the first
// 32 are the set (bytes 0-24 the driver's; 25 and 26 a time trial's or free run's flying start, read from the class-0
// set; 27-31 0 or unused), the other 32 are zero on the disc.
// A slot whose offset is 0 has no set: the race then uses the built-in set of the course
// category. The byte meanings: the table there and ai §9.1 (the race start reads them, sim/exact/start.h).

#include "base/bytes.h"

#include <array>
#include <optional>

namespace gt3r::course {

class CourseFile;

struct DriverSets {
    using Set = std::array<u8, 32>;
    static constexpr int kFirstSlot = 37, kClasses = 4;

    std::array<std::optional<Set>, kClasses> by_class;

    // The sets of a parsed course file (`data` the whole file). A section shorter than 32 bytes
    // is padded with zeros.
    static DriverSets parse(ByteSpan data, const CourseFile& file);
    // The set the race uses for a car of driveline class `cls`: classes outside 0..3 read the
    // class 0 slot. nullptr when that slot is empty.
    const Set* for_class(int cls) const;
    bool empty() const;
};

} // namespace gt3r::course
