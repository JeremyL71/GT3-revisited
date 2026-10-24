#pragma once

// GT3's menu showrooms: the camera and lights of
// menu/showroom_<kind>.fbi, shared by the GT-mode / arcade menu car renders
// and the online lobby's car picker.

#include "base/bytes.h"
#include "render/view3d.h"

namespace gt3r::render {

// menu/showroom_<kind>.fbi (menus §9.5): one 0x140-byte record, key k = f32 at
// +0x10 + 4k. 0..3 global ambient; lights i = 0..2 at 4 + 16 i: ambient, diffuse,
// specular, direction (eye space); 0x34 look-at start (x, y, z), 0x38 look-at target,
// 0x3B / 0x3C its spring k / damping; 0x3D elevation (negative = above), 0x3E yaw start,
// 0x3F distance start; 0x41 / 0x42 elevation / yaw acceleration, 0x43 angle damping;
// 0x44 distance target, 0x45 / 0x46 its k / damping; 0x47 / 0x48 view-space shift,
// 0x49 / 0x4A view-space pitch / yaw; 0x4B horizontal field of view.
struct ShowroomCamera {
    float k[76] = {};
    u32 light_mask = 7; // record +0x04: bit i enables light i (buycar: 1, the key light only)
    bool valid = false;
    float key(int i) const { return k[i]; }
};

// Invalid (valid = false) when the record is too short.
ShowroomCamera parse_showroom_camera(ByteSpan fbi);

// Player 2's copy of the split camera (mirrored: look-at x, yaw, yaw acceleration,
// view-space shift x and yaw negated).
ShowroomCamera mirrored(ShowroomCamera c);

// The showroom lights: the record's global ambient and three directional lights in eye
// space (fixed to the camera; the car turns under them), with GT3's material lighting.
// Reflections: the showroom's sphere map `env` (eye space) at full strength times
// `reflect` (1 - dirt), else an almost black gradient. `mirror` negates the lights' y (the
// floor reflection pass).
DrawOptions showroom_options(const ShowroomCamera& cam, const Mat4& view, bool mirror = false,
                             SDL_GPUTexture* env = nullptr, float reflect = 1);

} // namespace gt3r::render
