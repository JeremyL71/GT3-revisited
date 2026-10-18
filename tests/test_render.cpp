// Renderer workstream tests (CPU only: math, camera, procedural models).

#include "render/camera.h"
#include "render/gpu_model.h"
#include "render/math3d.h"
#include "render/test_models.h"
#include "test_framework.h"

#include <cmath>

using namespace gt3r;
using namespace gt3r::render;

namespace {
bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }
}

TEST(render_mat4_inverse) {
    Mat4 m = translate({1, 2, 3}) * rotate_y(0.7f) * scale({2, 3, 4});
    Mat4 i = m * inverse(m);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) CHECK(near(i.at(r, c), r == c ? 1.0f : 0.0f));
    Vec3 p = translate({1, 2, 3}).transform_point({1, 1, 1});
    CHECK(near(p.x, 2) && near(p.y, 3) && near(p.z, 4));
}

TEST(render_perspective_depth_range) {
    // Points on the near/far planes map to depth 0/1 (SDL_GPU convention).
    Mat4 p = perspective(1.0f, 16.0f / 9.0f, 0.5f, 100.0f);
    auto depth = [&](float z) {
        float zc = p.at(2, 2) * z + p.at(2, 3), wc = p.at(3, 2) * z;
        return zc / wc;
    };
    CHECK(near(depth(-0.5f), 0.0f));
    CHECK(near(depth(-100.0f), 1.0f));
}

TEST(render_look_at_maps_target_forward) {
    Mat4 v = look_at({0, 0, 5}, {0, 0, 0}, {0, 1, 0});
    Vec3 t = v.transform_point({0, 0, 0});
    CHECK(near(t.x, 0) && near(t.y, 0) && near(t.z, -5)); // camera looks down -Z
}

TEST(render_orbit_camera_frames_bounds) {
    OrbitCamera cam;
    float bmin[3] = {-1, 0, -2}, bmax[3] = {1, 2, 2};
    cam.frame(bmin, bmax);
    Vec3 e = cam.eye();
    float d = length(e - Vec3{0, 1, 0});
    CHECK(near(d, cam.distance, 1e-3f));
    CHECK(cam.distance > 2.0f); // outside the box radius
    float yaw0 = cam.yaw;
    PadState pad;
    pad.buttons = u16(1u << unsigned(PadButton::Right));
    cam.update(pad, 0.1);
    CHECK(cam.yaw > yaw0);
}

TEST(render_test_models_are_valid) {
    for (int v = 0; v < kTestModelVariants; ++v) {
        scene::Model m = make_test_model(v);
        CHECK(!m.meshes.empty());
        for (const scene::Mesh& mesh : m.meshes) {
            CHECK(mesh.indices.size() % 3 == 0);
            for (u32 i : mesh.indices) CHECK(i < mesh.vertices.size());
            u32 covered = 0;
            for (const scene::Primitive& p : mesh.primitives) {
                CHECK(p.first_index + p.index_count <= mesh.indices.size());
                CHECK(p.texture < int(m.textures.size()));
                covered += p.index_count;
            }
            CHECK(covered == mesh.indices.size());
        }
        for (const scene::Texture& t : m.textures) CHECK(t.rgba.size() == size_t(t.width) * t.height);
        CHECK(m.bounds_max[1] > m.bounds_min[1]);
    }
}

TEST(render_model_stats_by_lod) {
    scene::Model m = make_test_model(2); // three LODs
    ModelStats all = model_stats(m, -1), l0 = model_stats(m, 0), l2 = model_stats(m, 2);
    CHECK(all.max_lod == 2);
    CHECK(all.meshes == 3 && l0.meshes == 1 && l2.meshes == 1);
    CHECK(l0.triangles > l2.triangles);
    CHECK(all.triangles == l0.triangles + model_stats(m, 1).triangles + l2.triangles);
}

TEST(render_ground_grid) {
    float bmin[3] = {-2, -0.5f, -1}, bmax[3] = {2, 1, 1};
    scene::Model g = make_ground_grid(bmin, bmax);
    CHECK(g.meshes.size() == 1 && g.textures.size() == 1);
    CHECK(near(g.bounds_min[1], -0.5f) && near(g.bounds_max[1], -0.5f));
    CHECK(g.bounds_min[0] < -2 && g.bounds_max[0] > 2);
}
