#include "race/race_scene.h"

#include "base/log.h"
#include "formats/course/course_model.h"
#include "race/interp.h"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <cstdio>

namespace gt3r::race {

namespace {

const char* const kWheelNames[4] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};

// Copies the meshes accepted by `keep` (and only the textures they use) into a new model.
template <class Pred>
scene::Model extract(const scene::Model& src, Pred keep) {
    scene::Model out;
    out.name = src.name;
    std::vector<int> remap(src.textures.size(), -1);
    for (const scene::Mesh& m : src.meshes) {
        if (!keep(m)) continue;
        scene::Mesh copy = m;
        for (scene::Primitive& p : copy.primitives) {
            if (p.texture < 0 || size_t(p.texture) >= src.textures.size()) continue;
            int& r = remap[size_t(p.texture)];
            if (r < 0) {
                r = int(out.textures.size());
                out.textures.push_back(src.textures[size_t(p.texture)]);
            }
            p.texture = r;
        }
        out.meshes.push_back(std::move(copy));
    }
    out.compute_bounds();
    return out;
}

void log_bounds(const char* what, const scene::Model& m) {
    LOG_INFO("Race: %s bounds (%.0f %.0f %.0f) .. (%.0f %.0f %.0f), %zu meshes", what, m.bounds_min[0], m.bounds_min[1],
             m.bounds_min[2], m.bounds_max[0], m.bounds_max[1], m.bounds_max[2], m.meshes.size());
}

} // namespace

// The car model's origin (y = 0, the body's lowest point) sits at the static body height, which
// is the ride height q above the ground (GT3 places the body at ground + static CoG height, equal
// to q); the sim's car frame has its origin on the ground.
static float body_lift(const sim::VehicleSetup& setup) {
    float q = setup.wheels[0].ride_height;
    return q > 0 ? q : 0.13f; // placeholder suspension: a typical road car
}

bool car_camera_points(ByteSpan car_file, render::Vec3& p0, render::Vec3& p1) {
    auto rdf = [](ByteSpan d, size_t o) {
        u32 v = rd32(d, o);
        float f;
        std::memcpy(&f, &v, 4);
        return f;
    };
    try {
        CarFile f = CarFile::parse(car_file);
        ByteSpan g = f.info;
        if (g.size() < 0x70) return false;
        u32 n = rd32(g, 0x68), off = rd32(g, 0x6C);
        if (n < 2 || off + 0x80 > g.size()) return false;
        auto pt = [&](u32 i) { return render::Vec3{rdf(g, off + 0x40 * i), rdf(g, off + 0x40 * i + 4), rdf(g, off + 0x40 * i + 8)}; };
        p0 = pt(0), p1 = pt(1);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

render::Mat4 car_matrix(const sim::VehicleState& s) {
    const sim::Quat& q = s.orientation;
    return render::translate({s.position.x, s.position.y, s.position.z}) * render::rotate_quat(q.w, q.x, q.y, q.z);
}

CourseScene::CourseScene(render::GpuContext& ctx, ByteSpan data, const std::string& name, bool sheen)
    : frame_alpha_(sheen) {
    course::CourseModelOptions o;
    o.reflections = sheen; // road sheen (masked second pass)
    o.frame_alpha = frame_alpha_; // its mask: the frame alpha the passes leave (shadow masks included)
    o.backdrop = false;
    o.sky = false;
    scene::Model scenery = course::build_course_model(data, o, name);
    log_bounds("scenery", scenery);
    for (const scene::Mesh& m : scenery.meshes)
        for (const scene::Primitive& p : m.primitives) stats_.triangles += p.index_count / 3;
    stats_.meshes = scenery.meshes.size();
    stats_.textures = scenery.textures.size();
    scenery_ = std::make_unique<render::GpuModel>(ctx, scenery);
    {
        // What a car's sides reflect: the scenery's mean colour (texel x pre-lit vertex colour).
        double acc[3] = {}, n = 0;
        for (const scene::Mesh& m : scenery.meshes)
            for (const scene::Primitive& p : m.primitives) {
                if (p.texture < 0 || p.index_count == 0) continue;
                const scene::Texture& t = scenery.textures[size_t(p.texture)];
                if (t.rgba.empty()) continue;
                const scene::Vertex& v = m.vertices[m.indices[p.first_index]];
                u32 tc = t.rgba[t.rgba.size() / 2];
                for (int k = 0; k < 3; ++k)
                    acc[k] += double(p.index_count) * (double((tc >> (8 * k)) & 0xFF) / 255.0) *
                              (double((v.color >> (8 * k)) & 0xFF) / 255.0);
                n += double(p.index_count);
            }
        if (n > 0)
            for (int k = 0; k < 3; ++k) ground_[k] = float(acc[k] / n);
    }
    occluder_.add(scenery);

    course::CourseModelOptions bo;
    bo.scenery = bo.extras = bo.sky = false;
    bo.backdrop = true;
    scene::Model backdrop = course::build_course_model(data, bo, name + "/backdrop");
    occluder_.add(backdrop);
    if (!backdrop.meshes.empty()) {
        log_bounds("backdrop", backdrop);
        backdrop_ = std::make_unique<render::GpuModel>(ctx, backdrop);
    }

    course::CourseModelOptions so;
    so.scenery = so.extras = so.backdrop = false;
    so.sky = true;
    scene::Model sky = course::build_course_model(data, so, name + "/sky");
    if (!sky.meshes.empty()) {
        log_bounds("sky", sky);
        // Horizon colour: average of the sky textures' lower halves (where the horizon usually is).
        auto mean_rows = [&](float out[3], bool lower) {
            double acc[3] = {}, n = 0;
            for (const scene::Texture& t : sky.textures)
                for (u32 y = lower ? t.height / 2 : 0; y < (lower ? t.height : t.height / 2); ++y)
                    for (u32 x = 0; x < t.width; ++x) {
                        u32 c = t.rgba[size_t(y) * t.width + x];
                        for (int k = 0; k < 3; ++k) acc[k] += double((c >> (8 * k)) & 0xFF) / 255.0;
                        n += 1;
                    }
            if (n > 0)
                for (int k = 0; k < 3; ++k) out[k] = float(acc[k] / n);
        };
        mean_rows(horizon_, true);
        mean_rows(sky_rgb_, false);
        sky_ = std::make_unique<render::GpuModel>(ctx, sky);
    }
    occluder_.build();

    // The reflection map's scene: slot 5 scenery and the slot 23 sky dome.
    course::CourseModelOptions eo;
    eo.scenery = eo.extras = eo.backdrop = eo.sky = false;
    eo.scenery2 = true;
    scene::Model env_scenery = course::build_course_model(data, eo, name + "/env");
    if (!env_scenery.meshes.empty()) env_scenery_ = std::make_unique<render::GpuModel>(ctx, env_scenery);
    eo.scenery2 = false;
    eo.sky_object = true;
    scene::Model env_sky = course::build_course_model(data, eo, name + "/env_sky");
    if (!env_sky.meshes.empty()) env_sky_ = std::make_unique<render::GpuModel>(ctx, env_sky);
}

bool CourseScene::add_env_items(std::vector<render::DrawItem>& items, const render::Vec3& eye,
                                const render::Mat4& sky_rotation) const {
    if (!env_scenery_ && !env_sky_) return false;
    if (env_sky_) {
        render::DrawItem it{env_sky_.get(), render::translate(eye) * sky_rotation, 0};
        it.background = true;
        items.push_back(it);
    }
    if (env_scenery_) items.push_back({env_scenery_.get(), {}, 0});
    return true;
}

void CourseScene::add_items(std::vector<render::DrawItem>& items, const render::Vec3& eye) const {
    if (sky_) {
        render::DrawItem it{sky_.get(), render::translate(eye), 0};
        it.background = true;
        items.push_back(it);
    }
    if (backdrop_) {
        render::DrawItem it{backdrop_.get(), {}, 0};
        it.draw_distance = 0; // distant scenery: never distance-culled
        items.push_back(it);
    }
    if (scenery_) {
        render::DrawItem it{scenery_.get(), {}, 0};
        it.in_order = true; // file order, as the PS2 draws it
        it.frame_alpha = frame_alpha_;
        items.push_back(it);
    }
}

CarVisual::CarVisual(render::GpuContext& ctx, ByteSpan car_file, const std::string& model, const WheelSpec& ws,
                     int paint, ByteSpan wheel_file, const CarBodyMod& body_mod)
    : ctx_(&ctx) {
    CarModelOptions opt;
    opt.body_mod = body_mod;
    opt.wheel_override = wheel_file;
    opt.lod = -1;
    opt.wheels = true;
    opt.effects = true; // shadow nodes
    opt.wheel_spec = ws;
    opt.paint = paint;
    scene::Model car = load_car_model(car_file, opt, model);
    for (const scene::Mesh& m : car.meshes) lods_ = std::max(lods_, m.lod + 1);
    scene::Model body = extract(car, [](const scene::Mesh& m) {
        return m.name.rfind("wheel_", 0) != 0 && m.name.rfind("node", 0) != 0;
    });
    for (int k = 0; k < 3; ++k) {
        std::string n = k == 0 ? "node1" : k == 1 ? "node6" : "node5";
        shadow_src_[size_t(k)] = extract(car, [&](const scene::Mesh& m) { return m.name == n; });
        for (scene::Mesh& m : shadow_src_[size_t(k)].meshes)
            for (scene::Primitive& p : m.primitives) {
                p.material = {};
                if (k < 2) {
                    p.flags = scene::kAlphaBlend | scene::kUnlit | scene::kNoDepthWrite | scene::kDoubleSided, p.texture = -1;
                } else { // headlamp pool: textured, Cs As + Cd
                    p.flags = scene::kAdditive | scene::kUnlit | scene::kNoDepthWrite | scene::kDoubleSided;
                }
            }
    }
    body_ = std::make_unique<render::GpuModel>(ctx, body);
    try {
        lamps_ = CarLampSet::parse(CarFile::parse(car_file).info);
        LOG_INFO("Car %s: lamps %zu head, %zu tail, %zu brake", model.c_str(), lamps_.head.size(), lamps_.tail.size(), lamps_.brake.size());
    } catch (const std::exception&) {
    }
    {
        // The brake-lamp variant: the node programs' brake switch (selector id 0) takes its
        // "pressed" branch (lit lens chunks with lamp materials).
        CarModelOptions bo = opt;
        bo.brake = true;
        bo.wheels = false;
        bo.effects = false;
        scene::Model lit = load_car_model(car_file, bo, model);
        scene::Model lit_body = extract(lit, [](const scene::Mesh& m) { return m.name.rfind("node", 0) != 0; });
        if (!lit_body.meshes.empty()) body_brake_ = std::make_unique<render::GpuModel>(ctx, lit_body);
    }
    for (int i = 0; i < 4; ++i) {
        std::string n = kWheelNames[i];
        scene::Model w = extract(car, [&](const scene::Mesh& m) { return m.name == n || m.name == n + "_tyre"; });
        int axle = i >> 1;
        render::Vec3 c{(i & 1 ? 0.5f : -0.5f) * ws.track[axle], ws.radius[axle], (axle ? 0.5f : -0.5f) * ws.wheelbase};
        wheel_center_[size_t(i)] = c;
        for (scene::Mesh& m : w.meshes)
            for (scene::Vertex& v : m.vertices) v.pos[0] -= c.x, v.pos[1] -= c.y, v.pos[2] -= c.z;
        if (!w.meshes.empty()) wheels_[size_t(i)] = std::make_unique<render::GpuModel>(ctx, w);
    }
}

render::Mat4 CarVisual::body_matrix(const sim::VehicleState& s, const sim::VehicleSetup& setup) const {
    return car_matrix(s) * render::translate({0, body_lift(setup), 0});
}

void CarVisual::set_shadow(const CarShadowParams& p) {
    shadow_params_ = p;
    shadow_set_ = true;
    for (int k = 0; k < 3; ++k) {
        shadow_[size_t(k)].reset();
        if (!p.enabled || shadow_src_[size_t(k)].meshes.empty()) continue;
        scene::Model m = shadow_src_[size_t(k)];
        if (k == 2) { // headlamp pool: vertex colour kept, alpha x Env0 +0x58
            if (p.headlamp <= 0) continue;
            for (scene::Mesh& mesh : m.meshes)
                for (scene::Vertex& v : mesh.vertices) {
                    float a = std::min(1.0f, float(v.color >> 24) / 255.0f * p.headlamp);
                    v.color = (v.color & 0x00FFFFFFu) | u32(a * 255 + 0.5f) << 24;
                }
            shadow_[2] = std::make_unique<render::GpuModel>(*ctx_, m);
            continue;
        }
        float f = p.alpha_scale; // node 1's road-shade factor F is applied per frame (CarLook::shadow)
        for (scene::Mesh& mesh : m.meshes)
            for (scene::Vertex& v : mesh.vertices) {
                float a = std::min(1.0f, float(v.color >> 24) / 255.0f * f);
                u32 c = u32(std::clamp(p.rgb[0], 0.0f, 1.0f) * 255) | u32(std::clamp(p.rgb[1], 0.0f, 1.0f) * 255) << 8 |
                        u32(std::clamp(p.rgb[2], 0.0f, 1.0f) * 255) << 16 | u32(a * 255 + 0.5f) << 24;
                v.color = c;
            }
        shadow_[size_t(k)] = std::make_unique<render::GpuModel>(*ctx_, m);
    }
}

void CarVisual::add_items(std::vector<render::DrawItem>& items, const sim::VehicleState& s,
                          const sim::VehicleSetup& setup, const render::Vec3& eye, const CarLook& look) const {
    float dist = render::length(render::Vec3{s.position.x, s.position.y, s.position.z} - eye);
    int lod = dist < 30 ? 0 : dist < 80 ? 1 : 2;
    lod = std::min(lod, lods_ - 1);
    if (shadow_params_.enabled && (shadow_[0] || shadow_[1])) {
        // Ground frame: the plane through the four wheel contacts, the car's heading.
        sim::Vec3 c{0, 0, 0};
        for (const sim::WheelState& w : s.wheels) c = c + w.contact * 0.25;
        sim::Vec3 fwd = (s.wheels[0].contact + s.wheels[1].contact) - (s.wheels[2].contact + s.wheels[3].contact);
        sim::Vec3 rgt = (s.wheels[1].contact + s.wheels[3].contact) - (s.wheels[0].contact + s.wheels[2].contact);
        render::Vec3 f = render::normalize(render::Vec3{-float(fwd.x), -float(fwd.y), -float(fwd.z)}); // car +Z = back
        render::Vec3 rr = render::normalize(render::Vec3{float(rgt.x), float(rgt.y), float(rgt.z)});
        render::Vec3 up = render::normalize(render::cross(f, rr));
        rr = render::normalize(render::cross(up, f));
        if (render::length(f) > 0.5f && up.y > 0.3f) {
            render::Mat4 g; // columns: right (x), up (y), back (z), origin 2 cm above the plane
            render::Vec3 o = render::Vec3{float(c.x), float(c.y), float(c.z)} + up * 0.02f;
            for (int i = 0; i < 3; ++i) {
                g.at(i, 0) = (&rr.x)[i], g.at(i, 1) = (&up.x)[i], g.at(i, 2) = (&f.x)[i], g.at(i, 3) = (&o.x)[i];
            }
            // Light shear in the ground frame: x' = x + dx y, y' = 0, z' = z + dz y.
            auto shear = [&](const render::Vec3& d) {
                render::Mat4 sh;
                sh.at(0, 1) = render::dot(d, rr);
                sh.at(1, 1) = 1e-3f; // flattened (kept invertible for the normal matrix)
                sh.at(2, 1) = render::dot(d, f);
                return sh;
            };
            render::Mat4 sh = shear(shadow_params_.shear);
            render::Mat4 flat;
            flat.at(1, 1) = 1e-3f;
            // Depth bias (GT3 nudges the shadow's near/far planes): pull it 2 % towards the eye
            // along the view rays, which leaves its screen position unchanged.
            render::Mat4 bias = render::translate(eye) * render::scale({0.98f, 0.98f, 0.98f}) *
                                render::translate(render::Vec3{0, 0, 0} - eye);
            for (int k = 0; shadow_[0] && k < look.lamp_shadows && k < 4; ++k) {
                if (look.lamp_alpha[k] <= 0.01f) continue;
                render::DrawItem it{shadow_[0].get(), bias * g * shear(look.lamp_shear[k]), lod};
                it.alpha = look.lamp_alpha[k];
                items.push_back(it);
            }
            if (shadow_[0] && look.lamp_shadows == 0 && look.shadow * shadow_params_.brightness > 0.01f) {
                render::DrawItem it{shadow_[0].get(), bias * g * sh, lod};
                it.alpha = look.shadow * shadow_params_.brightness; // > 1 allowed: the blend clamps
                items.push_back(it);
            }
            if (shadow_[1]) items.push_back({shadow_[1].get(), bias * g * flat, lod});
            if (shadow_[2]) items.push_back({shadow_[2].get(), bias * g * flat, lod});
        }
    }
    add_body_items(items, s, setup, look, {}, lod);
}

bool CarVisual::road_mirror(const sim::VehicleState& s, render::Mat4& out) {
    sim::Vec3 c{0, 0, 0};
    for (const sim::WheelState& w : s.wheels) c = c + w.contact * 0.25;
    sim::Vec3 fwd = (s.wheels[0].contact + s.wheels[1].contact) - (s.wheels[2].contact + s.wheels[3].contact);
    sim::Vec3 rgt = (s.wheels[1].contact + s.wheels[3].contact) - (s.wheels[0].contact + s.wheels[2].contact);
    // Up = back x right (as the shadows' ground frame).
    render::Vec3 n = render::normalize(render::cross(render::Vec3{-float(fwd.x), -float(fwd.y), -float(fwd.z)},
                                                     render::Vec3{float(rgt.x), float(rgt.y), float(rgt.z)}));
    if (!(n.y > 0.3f)) return false;
    render::Vec3 p{float(c.x), float(c.y), float(c.z)};
    float pn = render::dot(p, n);
    out = render::Mat4{};
    const float N[3] = {n.x, n.y, n.z};
    for (int r = 0; r < 3; ++r) {
        for (int k = 0; k < 3; ++k) out.at(r, k) = (r == k ? 1.0f : 0.0f) - 2.0f * N[r] * N[k];
        out.at(r, 3) = 2.0f * pn * N[r];
    }
    return true;
}

bool CarVisual::add_mirror_items(std::vector<render::DrawItem>& items, const sim::VehicleState& s,
                                 const sim::VehicleSetup& setup, const render::Vec3& eye, const CarLook& look) const {
    render::Mat4 m;
    if (!road_mirror(s, m)) return false;
    float dist = render::length(render::Vec3{s.position.x, s.position.y, s.position.z} - eye);
    int lod = std::min(dist < 30 ? 0 : dist < 80 ? 1 : 2, lods_ - 1);
    add_body_items(items, s, setup, look, m, lod);
    return true;
}

void CarVisual::add_body_items(std::vector<render::DrawItem>& items, const sim::VehicleState& s,
                               const sim::VehicleSetup& setup, const CarLook& look, const render::Mat4& pre,
                               int lod) const {
    render::Mat4 world = pre * car_matrix(s);
    auto styled = [&](render::DrawItem it) {
        for (int k = 0; k < 3; ++k) it.light_scale[k] = look.light_scale[k];
        it.reflection_scale = look.reflection;
        it.lamp = look.lamp;
        it.lights = look.lights;
        return it;
    };
    const render::GpuModel* body = look.brake_now && body_brake_ ? body_brake_.get() : body_.get();
    items.push_back(styled({body, world * render::translate({0, body_lift(setup), 0}), lod}));
    for (int i = 0; i < 4; ++i) {
        if (!wheels_[size_t(i)]) continue;
        const sim::WheelState& w = s.wheels[size_t(i)];
        const sim::VehicleSetup::Wheel& sw = setup.wheels[size_t(i)];
        // Hub in the car frame: the sim's rest hub (y = tyre radius over the ground-level origin)
        // plus the suspension travel from the static pose (+ = compressed: up into the arch;
        // airborne = max droop). The body mesh is lifted by the ride height (see body_lift).
        render::Vec3 c = wheel_center_[size_t(i)];
        c.y = sw.rest_center.y + (w.suspension - sw.static_suspension);
        render::Mat4 m = world * render::translate(c) * render::rotate_y(-w.steer_angle) * render::rotate_x(-w.spin_angle);
        items.push_back(styled({wheels_[size_t(i)].get(), m, lod}));
    }
}

} // namespace gt3r::race
