#pragma once

// Course billboard sprites (BLBd, course file slot 30; docs/formats/course.md): trees,
// spectators and the like, drawn with the scenery as quads that turn about the vertical
// axis to face the camera. Header: +0x10 u16 list count, +0x14 u16 type count, +0x18
// rel-ptr to the list pointers (one list per visibility cell), +0x1C rel-ptr to an
// embedded Tex1, +0x24 rel-ptr to the sprite types {f32 width, f32 height, rel-ptr to
// 4 UVs, rel-ptr to 4 RGBA colours}. A list: u32 count, then 16-byte instances
// {u16 type, u16 texture + 1, f32 x, y, z} (bottom centre).

#include "base/bytes.h"
#include "render/gpu_model.h"
#include "render/view3d.h"
#include "scene/model.h"

#include <memory>
#include <vector>

namespace gt3r::race {

class Billboards {
public:
    // False when the block is absent or empty.
    bool load(render::GpuContext& ctx, ByteSpan blbd, const std::string& name);
    size_t count() const { return sprites_.size(); }
    // Turns every sprite to the camera (its right axis, flattened) and appends the item.
    void add_items(std::vector<render::DrawItem>& items, const render::Vec3& camera_right);

private:
    struct Sprite {
        float x, y, z, w, h;
    };
    std::vector<Sprite> sprites_; // in vertex order (4 vertices each)
    std::vector<scene::Vertex> verts_;
    std::unique_ptr<render::GpuModel> gpu_;
    render::Vec3 last_right_{0, 0, 0};
};

} // namespace gt3r::race
