#pragma once

// Course visibility lists (Vls0; slots 2-4 for the main scenery, 6-7 for slot 5;
// docs/formats/course.md "Vls0"). Offsets are relative to the block start:
//   +0x10 u16 grid width nx, +0x12 u16 grid depth nz, +0x14 offset of nx * nz cell offsets
//   (row-major, z rows), +0x18 f32 x0, +0x1C f32 z0, +0x20 / +0x24 f32 extent in x / z,
//   +0x28 / +0x2C f32 1 / extent.
// A cell: zero-terminated u32 offsets of view points {u32 offset of a node list, f32 x, y, z};
// a node list: u16 n, then n u16 indices of the scenery nodes visible from that point.
// The game takes the cell under the camera (clamped to the grid) and that cell's point
// nearest the camera (squared 3D distance, the first one on ties), and uses its node list for
// the scenery and for the GTFX lamp lists (one list per slot 1 node).

#include "base/bytes.h"

#include <vector>

namespace gt3r::course {

class Visibility {
public:
    // Empty when the block is absent or not a Vls0.
    static Visibility parse(ByteSpan vls0);
    bool empty() const { return cells_.empty(); }
    // The node list for a camera at (x, y, z); nullptr without data.
    const std::vector<u16>* visible(float x, float y, float z) const;

private:
    struct Point {
        float x = 0, y = 0, z = 0;
        u32 list = 0; // index into lists_
    };
    u32 nx_ = 0, nz_ = 0;
    float x0_ = 0, z0_ = 0, sx_ = 0, sz_ = 0;
    std::vector<std::vector<u32>> cells_; // point indices per cell
    std::vector<Point> points_;
    std::vector<std::vector<u16>> lists_;
};

} // namespace gt3r::course
