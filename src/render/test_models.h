#pragma once

// Procedural models for exercising the renderer without game data.

#include "scene/model.h"

namespace gt3r::render {

// Test scene, one per variant (wraps around):
//   0: checker-textured cube (clamped texture), lit vertex-coloured sphere,
//      alpha-blended / additive / alpha-tested quads, a double-sided plane.
//   1: environment-mapped sphere plus an unlit vertex-colour pyramid.
//   2: LOD test: the same sphere at three LODs (lod 0..2), side by side.
scene::Model make_test_model(int variant = 0);
inline constexpr int kTestModelVariants = 3;
const char* test_model_name(int variant);

// Translucent ground grid sized for the given bounds (sits at bounds_min.y).
scene::Model make_ground_grid(const float bmin[3], const float bmax[3]);

// Helpers shared by tests: a UV sphere and an axis-aligned box.
void append_sphere(scene::Mesh& mesh, float cx, float cy, float cz, float r, int slices, int stacks, u32 color);
void append_box(scene::Mesh& mesh, float cx, float cy, float cz, float hx, float hy, float hz, u32 color);

} // namespace gt3r::render
