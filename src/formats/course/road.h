#pragma once

// GTRW: the drivable road surface of a course (course file slot 28).
// All positions are floats in the same space as the scenery (GTM1), metres,
// Y up. Offsets are relative to the "GTRW" magic.
//
//   0x00 "GTRW", 8 bytes 0, u32 section size
//   0x10 u32 ? (9)   u32 course id (shared with GTRC)   f32 track length (m)   f32 ?
//   0x20 u16 ?, u16 marker_count   u32 markers_off     -> {u32 flags, f32 distance}
//   0x28 u32 grid_count            u32 grid_off        -> {f32 x, y, z, yaw (rad)}; heading = yaw + pi
//   0x30 u32 segment_count         u32 segments_off    -> 0x40-byte segment records
//   0x38 u32 wall_point_count      u32 wall_points_off -> {f32 x, y, z; u32 tag}
//   0x40 u32 checkpoint_count      u32 checkpoints_off -> 0x20-byte records
//   0x48 u32 grid_a_off            u32 grid_b_off      -> lookup grids (triangles: RoadGrid; wall points)
//   0x50 u32 extra_count           u32 extra_off       -> 0x20-byte records
//   0x58 u32 0                     u32 end offset
//
// Segment record (0x40 bytes):
//   u16 vertex_count, u16 triangle_count, u32 vertices_off, u32 triangles_off,
//   u32 0, u32 wall_refs_off, u16 ?, u16 neighbour_count, u16 neighbours[] (first two:
//   prev and next segment), 0...
//   vertices: {f32 x, y, z; u16 attrA, u16 attrB} (attrs /65535, usually 0xFFFF)
//   triangles: {u16 a, b, c, 0; u16 material, 0; u16 flags, 0} (see docs/formats/course.md)
//
// Wall points form polylines: the first point of each polyline has the
// point count in the low 16 bits of `tag`, the last has -count; bit 16 marks
// special points (openings?). Typically: left wall, right wall, pit wall.
//
// Checkpoint record: {f32 x0, z0, x1, z1; f32 distance; f32 dir_x, dir_z; f32 ?}:
// a line across the road at `distance` metres from the start line, (dir_x,
// dir_z) pointing against the direction of travel; the records are ~15 m
// apart and form a ring around the track.

#include "base/bytes.h"

#include <array>
#include <optional>
#include <vector>

namespace gt3r::course {

struct Vec3 {
    float x = 0, y = 0, z = 0;
};

struct RoadTriangle {
    u16 a = 0, b = 0, c = 0; // indices into RoadSegment::vertices
    // Raw words at +8 and +0xC (kept for compatibility). The game reads them
    // as u16 values: `material` (= low half of edge_flags) and `flags`
    // (= low half of `surface`); see docs/formats/course.md.
    u32 edge_flags = 0;
    u32 surface = 0;
    u16 material = 0; // +0x08: surface material index (0 tarmac, 1, 2, ...)
    u16 flags = 0;    // +0x0C: bit 0 rumble, bit 1 alternate material, bits 3-4 ...
};

struct RoadSegment {
    std::vector<Vec3> vertices;
    std::vector<RoadTriangle> triangles;
    u16 prev = 0, next = 0;
    u16 unknown_a = 0, unknown_b = 0;
    std::vector<u16> wall_refs; // raw u16 rows (look like Road::wall_points indices)
    std::vector<u16> checkpoint_ids; // +0x10 / +0x14 (u16 count, unknown_a): the checkpoints the race progress query tries
    std::vector<u32> vertex_attrs; // 4th word of each vertex: two u16 scalars (0xFFFF = 1.0)
    std::vector<u16> neighbours;   // adjacent segments (+0x16 count, +0x18 ids); prev/next
                                   // above are the first two entries of this list
    Vec3 bounds_min, bounds_max;
};

// The game's spatial lookup grids (GTRW +0x48 over the road triangles, +0x4C
// over the wall points): sorted X and Z breakpoints (uniform 8 m in every
// course) and, per cell in stored order, (segment, triangle or wall point)
// references; every wall point is listed once, in the cell that holds it.
struct RoadGridRef {
    u16 segment = 0, triangle = 0;
};

struct RoadGridGroup {
    u16 segment = 0;
    u32 begin = 0, end = 0; // the group's references in RoadGrid::refs
};

struct RoadGrid {
    std::vector<float> xs, zs;       // breakpoints; cells = (xs-1) x (zs-1)
    std::vector<u32> cell_begin;     // (cells + 1) offsets into refs (CSR)
    std::vector<RoadGridRef> refs;
    // The cells' groups as stored ({u16 n, u16 segment, u16 id[n]}, in order): (cells + 1) offsets into groups (CSR).
    std::vector<u32> cell_group_begin;
    std::vector<RoadGridGroup> groups;
    bool empty() const { return xs.size() < 2 || zs.size() < 2; }
};

struct RoadMarker {
    u32 flags = 0;
    float distance = 0;
};

struct GridSlot {
    Vec3 pos;
    float yaw = 0; // radians
};

struct Checkpoint {
    float x0 = 0, z0 = 0, x1 = 0, z1 = 0;
    float distance = 0;
    float dir_x = 0, dir_z = 0;
    float unknown = 0;
};

struct WallPoint {
    Vec3 pos;
    u32 tag = 0;
};

struct Road {
    u32 course_id = 0;
    float length = 0;
    std::vector<RoadMarker> markers;
    std::vector<GridSlot> grid;
    std::vector<RoadSegment> segments;
    std::vector<WallPoint> wall_points;
    std::vector<std::vector<Vec3>> walls; // wall points split into polylines
    std::vector<Checkpoint> checkpoints;  // sorted as stored (a ring)
    float start_distance = 0;             // header +0x1C (0 or length in base files)
    u16 header_flag = 0;                  // header +0x20
    RoadGrid grid_index;                  // decoded +0x48 lookup grid (may be empty)
    RoadGrid wall_grid;                   // decoded +0x4C grid over the wall points: refs (segment, wall point index)
    std::vector<u16> before_segments;     // the words the program scans as segment -1's neighbour list (physics §5.1)
    // The start grid as the race start reads it: slot i at the grid table + 16 i for any i below 8, past the table's end too
    // (what follows it in the block: padding, the segment table). The block's words there (0 past the block's end), which of
    // them the course loader turns into pointers (bit k for word k: in memory such a word holds itself plus the block's
    // address less `base`), and `base`, the block's +0x04 as the file holds it.
    std::array<u32, 32> grid_words{};
    u32 grid_pointers = 0;
    u32 base = 0;

    // `data` must start with "GTRW".
    static Road parse(ByteSpan data);

    size_t triangle_count() const;
    // Height of the road surface under (x, z), taking the highest triangle
    // below `y_max`. Empty when (x, z) is off the road.
    std::optional<float> height_at(float x, float z, float y_max = 1e30f, u32* surface = nullptr) const;
    // Centre line: midpoints of the checkpoint lines in distance order,
    // with y from the road surface where available.
    std::vector<Vec3> centre_line() const;
};

} // namespace gt3r::course
