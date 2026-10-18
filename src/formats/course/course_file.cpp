#include "formats/course/course_file.h"

#include <cctype>

namespace gt3r::course {

bool CourseFile::looks_like_course(ByteSpan data) {
    if (data.size() < kSlotCount * 4 + 4) return false;
    // Slot 0 is always empty and slot 1 (main scenery) starts right after the table.
    return rd32(data, 0) == 0 && rd32(data, 4) == kSlotCount * 4 && has_magic(data.subspan(kSlotCount * 4), "GTM1");
}

CourseFile CourseFile::parse(ByteSpan data) {
    if (data.size() < kSlotCount * 4) throw FormatError("course: file too small");
    CourseFile f;
    u32 prev_end = kSlotCount * 4;
    for (int i = 0; i < kSlotCount; ++i) {
        u32 off = rd32(data, size_t(i) * 4);
        if (!off) continue;
        if (off < prev_end || off > data.size()) throw FormatError("course: bad section offset");
        CourseSection s;
        s.slot = i;
        s.offset = off;
        prev_end = off;
        if (off + 4 <= data.size()) {
            bool printable = true;
            for (int k = 0; k < 4; ++k) printable &= std::isprint(data[off + k]) != 0;
            if (printable) s.tag.assign(reinterpret_cast<const char*>(data.data() + off), 4);
        }
        f.sections_.push_back(s);
    }
    for (size_t i = 0; i < f.sections_.size(); ++i) {
        u32 end = i + 1 < f.sections_.size() ? f.sections_[i + 1].offset : u32(data.size());
        f.sections_[i].size = end - f.sections_[i].offset;
    }
    return f;
}

const CourseSection* CourseFile::find(int slot) const {
    for (const CourseSection& s : sections_)
        if (s.slot == slot) return &s;
    return nullptr;
}

ByteSpan CourseFile::section(ByteSpan data, int slot) const {
    const CourseSection* s = find(slot);
    return s ? data.subspan(s->offset, s->size) : ByteSpan{};
}

} // namespace gt3r::course
