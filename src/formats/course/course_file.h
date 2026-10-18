#pragma once

// Course files: courses/<REGION>/<name> (gzip in GT3.VOL, 4-14 MB unpacked).
//
// The file starts with a table of 48 u32 section offsets (0 = absent); a
// section ends where the next present one starts (or at end of file). Every
// course uses the same slot assignment:
//
//   slot  tag    contents
//   1     GTM1   main scenery model (nodes x chunks, 1-24 texture sets)
//   2-4   Vls0   visibility lists for slot 1
//   5     GTM1   secondary scenery model (road reflections / far detail)
//   6-7   Vls0   visibility lists for slot 5
//   9,13  GTM1   optional extra models (+ Vls0 in 10, 14)
//   21    GTM1   sky dome (centred on the camera)
//   22    GTM1   distant backdrop
//   23    GTM1   small unit object (sun / flare anchor?)
//   24    GTM1   low-resolution sky dome (secondary view pass only, not the main view)
//   25    -      optional small table (28 of 320 files)
//   26    Tex1   extra texture set
//   27    GTM1   optional
//   28    GTRW   road surface, walls, checkpoints, start grid (see road.h)
//   29    GTFX   effects parameters
//   30    BLBd   blob/shadow texture block
//   31    -      AI driving lines (see ai_lines.h)
//   32    Env0   environment (fog/light colours)
//   33    GTCM   course map for the HUD (with Tex1)
//   34    GTRC   replay camera track
//   35    CRSS   ?
//   36    -      ?
//   37-40 "sdd"  per-surface (sound/grip?) parameter blocks, 0x40 bytes each
//
// Region folders (US, JP, UK, EU) hold the same set of 80 courses; most files
// are byte-identical across regions.

#include "base/bytes.h"

#include <string>
#include <vector>

namespace gt3r::course {

enum Slot : int {
    kSlotScenery = 1,
    kSlotScenery2 = 5,
    kSlotSky = 21,
    kSlotBackdrop = 22,
    kSlotSkyObject = 23,
    kSlotSky2 = 24,
    kSlotRoad = 28,
    kSlotAiLines = 31,
    kSlotEnv = 32,
    kSlotCourseMap = 33,
    kSlotReplayCameras = 34,
    kSlotCount = 48,
};

struct CourseSection {
    int slot = 0;
    u32 offset = 0;
    u32 size = 0;
    std::string tag; // printable 4-char magic, or "" when the section has none
};

class CourseFile {
public:
    // Throws FormatError if the offset table is inconsistent.
    static CourseFile parse(ByteSpan data);
    // Cheap check used to recognise course files.
    static bool looks_like_course(ByteSpan data);

    const std::vector<CourseSection>& sections() const { return sections_; }
    const CourseSection* find(int slot) const;
    // The bytes of a slot, or an empty span when absent.
    ByteSpan section(ByteSpan data, int slot) const;

private:
    std::vector<CourseSection> sections_;
};

} // namespace gt3r::course
