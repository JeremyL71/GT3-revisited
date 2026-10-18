#include "render/showroom.h"

#include <algorithm>
#include <cstring>

namespace gt3r::render {

ShowroomCamera parse_showroom_camera(ByteSpan f) {
    ShowroomCamera c;
    if (f.size() < 0x140) return c;
    for (int i = 0; i < 76; ++i) {
        u32 v = rd32(f, size_t(0x10 + 4 * i));
        std::memcpy(&c.k[i], &v, 4);
    }
    c.light_mask = rd32(f, 4);
    c.valid = true;
    return c;
}

ShowroomCamera mirrored(ShowroomCamera c) {
    c.k[0x34] = -c.k[0x34], c.k[0x38] = -c.k[0x38];
    c.k[0x3E] = 360.0f - c.k[0x3E];
    c.k[0x42] = -c.k[0x42];
    c.k[0x47] = -c.k[0x47], c.k[0x4A] = -c.k[0x4A];
    return c;
}

DrawOptions showroom_options(const ShowroomCamera& cam, const Mat4& view, bool mirror, SDL_GPUTexture* env,
                             float reflect) {
    DrawOptions o;
    auto world = [&](const float d[3], float out[3]) {
        float y = mirror ? -d[1] : d[1];
        for (int i = 0; i < 3; ++i) out[i] = d[0] * view.at(0, i) + y * view.at(1, i) + d[2] * view.at(2, i);
    };
    o.gt3_lighting = true;
    for (int i = 0; i < 3; ++i) {
        o.light_ambient[i] = cam.k[i];
        o.light_diffuse[i] = cam.k[8 + i];
        o.light_specular[i] = cam.k[12 + i];
    }
    float d0[3];
    world(&cam.k[16], d0);
    o.light_dir = {d0[0], d0[1], d0[2]};
    if (!(cam.light_mask & 1))
        for (int i = 0; i < 3; ++i) o.light_diffuse[i] = o.light_specular[i] = 0;
    // Each enabled light's own ambient term adds to the global ambient (GL lighting); only
    // buycar has one (0.4 on light 0).
    for (int l = 0; l < 3; ++l)
        if (cam.light_mask & (1u << l))
            for (int i = 0; i < 3; ++i) o.light_ambient[i] += cam.k[4 + 16 * l + i];
    o.fill_count = 0;
    for (int l = 0; l < 2; ++l) {
        if (!(cam.light_mask & (2u << l))) continue; // disabled fill light
        const float* L = &cam.k[4 + 16 * (l + 1)];
        const int f = o.fill_count++;
        world(L + 12, o.fill_dir[f]);
        for (int i = 0; i < 3; ++i) o.fill_diffuse[f][i] = L[4 + i], o.fill_specular[f][i] = L[8 + i];
    }
    const float sky[3] = {0.10f, 0.08f, 0.07f}, horizon[3] = {0.05f, 0.04f, 0.04f}, ground[3] = {0.01f, 0.01f, 0.01f};
    for (int i = 0; i < 3; ++i) o.env_sky[i] = sky[i], o.env_horizon[i] = horizon[i], o.env_ground[i] = ground[i];
    if (env) {
        o.env_sphere = env;
        o.reflection_strength = std::clamp(reflect, 0.0f, 1.0f);
    }
    return o;
}

} // namespace gt3r::render
