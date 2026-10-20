#include "race/lamp_glows.h"

#include "base/log.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::race {

namespace {

float rdf(ByteSpan d, size_t o) {
    u32 v = rd32(d, o);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}

// The glow sprite: a radial disc (white centre, alpha to 0 at the rim) with thin vertical and
// diagonal streaks, as race/flare.bin; and a plain soft disc for the halo.
SDL_Texture* make_sprite(SDL_Renderer* r, bool streaks) {
    constexpr int N = 128;
    std::vector<u32> px(size_t(N) * N);
    for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
            float u = (float(x) + 0.5f) / N * 2 - 1, v = (float(y) + 0.5f) / N * 2 - 1;
            // Disc of radius 0.425 inside a sprite spanning +-1.17 (the streak length).
            float k = streaks ? 1.17f : 0.425f;
            float d = std::sqrt(u * u + v * v) * k / 0.425f;
            float a = std::max(0.0f, 1.0f - d);
            a = a * a;
            if (streaks) {
                float sv = std::max(0.0f, 1.0f - std::fabs(u) * 40.0f) * std::max(0.0f, 1.0f - std::fabs(v)); // vertical
                float dd = std::fabs(u - v) * 0.7071f;
                float sd = std::max(0.0f, 1.0f - dd * 40.0f) * std::max(0.0f, 1.0f - std::sqrt(u * u + v * v) / 1.2f);
                a = std::max(a, (84.0f / 255.0f) * std::max(sv, sd));
            }
            u32 c = u32(std::clamp(a, 0.0f, 1.0f) * 255.0f);
            px[size_t(y) * N + size_t(x)] = 0x00FFFFFFu | (c << 24);
        }
    SDL_Texture* t = SDL_CreateTexture(r, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STATIC, N, N);
    if (!t) return nullptr;
    SDL_UpdateTexture(t, nullptr, px.data(), N * 4);
    SDL_SetTextureBlendMode(t, SDL_BLENDMODE_ADD);
    SDL_SetTextureScaleMode(t, SDL_SCALEMODE_LINEAR);
    return t;
}

} // namespace

LampGlows::~LampGlows() {
    if (glow_) SDL_DestroyTexture(glow_);
    if (disc_) SDL_DestroyTexture(disc_);
}

void LampGlows::load(SDL_Renderer* r, ByteSpan g, ByteSpan env) {
    lamps_.clear();
    if (env.size() < 0xE60 || std::memcmp(env.data(), "Env0", 4) != 0 || !(rd32(env, 0x10) & 0x200000)) return;
    for (int i = 0; i < 8; ++i) {
        size_t o = 0xC60 + 0x40 * size_t(i);
        LampType& t = types_[size_t(i)];
        t.rgb = {rdf(env, o), rdf(env, o + 4), rdf(env, o + 8)};
        t.intensity = rdf(env, o + 0xC);
        t.glow = rdf(env, o + 0x10) * 10.0f;
        t.halo = rdf(env, o + 0x14);
        t.halo_gain = rdf(env, o + 0x18);
        t.exponent = rdf(env, o + 0x1C);
        t.att_c = rdf(env, o + 0x20), t.att_l = rdf(env, o + 0x24), t.att_q = rdf(env, o + 0x28);
        t.max_dist = rdf(env, o + 0x2C);
        t.box = rdf(env, o + 0x30) * 128.0f;
        t.blink = rdf(env, o + 0x34);
    }
    if (g.size() < 0x18 || std::memcmp(g.data(), "GTFX", 4) != 0 || !rd32(g, 0x10)) return;
    u32 table = rd32(g, 0x14);
    if (table + 4 > g.size()) return;
    u32 lists = rd32(g, table);
    for (u32 i = 0; i < lists && table + 8 + 4ull * i <= g.size(); ++i) {
        u32 lp = rd32(g, table + 4 + 4 * i);
        if (!lp || lp + 4 > g.size()) continue;
        u32 n = rd32(g, lp);
        for (u32 k = 0; k < n && lp + 4 + 32ull * (k + 1) <= g.size(); ++k) {
            size_t o = lp + 4 + 32ull * k;
            Lamp l;
            l.type = int(rd32(g, o) & 7);
            l.pos = {rdf(g, o + 4), rdf(g, o + 8), rdf(g, o + 12)};
            l.dir = {rdf(g, o + 16), rdf(g, o + 20), rdf(g, o + 24)};
            l.phase = rdf(g, o + 28);
            lamps_.push_back(l);
        }
    }
    if (!lamps_.empty()) {
        if (!glow_) glow_ = make_sprite(r, true);
        if (!disc_) disc_ = make_sprite(r, false);
        LOG_INFO("Lamp glows: %zu lamps", lamps_.size());
    }
}

void LampGlows::draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, const render::Vec3& eye,
                     const render::Frustum& fr,
                     const std::function<bool(const render::Vec3&, const render::Vec3&)>& blocked) const {
    if (lamps_.empty() || !glow_) return;
    int drawn = 0;
    for (const Lamp& l : lamps_) {
        if (drawn >= 100) break;
        const LampType& ty = types_[size_t(l.type)];
        // View space (camera looks down -z).
        float vx = view.at(0, 0) * l.pos.x + view.at(0, 1) * l.pos.y + view.at(0, 2) * l.pos.z + view.at(0, 3);
        float vy = view.at(1, 0) * l.pos.x + view.at(1, 1) * l.pos.y + view.at(1, 2) * l.pos.z + view.at(1, 3);
        float vz = view.at(2, 0) * l.pos.x + view.at(2, 1) * l.pos.y + view.at(2, 2) * l.pos.z + view.at(2, 3);
        float z = -vz;
        if (z <= 0.5f || (ty.max_dist > 0 && z > ty.max_dist)) continue;
        // Directional factor (lamp facing vs the direction to the camera).
        float D = 1;
        int n = int(std::lround(ty.exponent));
        if (n > 0) {
            // dot(facing, camera->lamp), as the original (dir_view . normalize(p_view)): a lamp
            // stored facing up lights the road below it as seen from the cars.
            render::Vec3 cl = normalize(l.pos - eye);
            D = std::pow(std::max(0.0f, dot(normalize(l.dir), cl)), float(n));
            if (D <= 0) continue;
        }
        float I = ty.intensity / std::max(1e-3f, ty.att_c + ty.att_l * z + ty.att_q * z * z);
        if (ty.blink > 0) I *= 0.5f + 0.5f * float(std::sin(2 * 3.14159265 * (time_ / ty.blink + l.phase)));
        float zz = z - 0.5f; // pulled 0.5 m towards the camera
        float px = fr.screen_x(vx, zz, rect.x, rect.w), py = fr.screen_y(vy, zz, rect.y, rect.h);
        float per_m = fr.per_metre(zz, rect.h); // pixels per metre at the lamp
        float glow_r = ty.glow * 1.17f * per_m, halo_r = ty.halo * per_m;
        float rad = std::max(glow_r, halo_r);
        if (rad < 0.5f || px + rad < rect.x || px - rad > rect.x + rect.w || py + rad < rect.y || py - rad > rect.y + rect.h)
            continue;
        ++drawn;
        // Visibility: occlusion between the camera and a point 0.2 m in front of the lamp.
        // (The original tests 0.2 m in front in depth; our ray test stops 1 m short so the lamp
        // fixture the light sits in does not hide it.)
        float V = blocked(eye, l.pos + normalize(eye - l.pos) * 1.0f) ? 0.0f : 1.0f;
        auto c8 = [](float v) { return u8(std::clamp(v, 0.0f, 1.0f) * 255.0f); };
        if (glow_r > 0.5f && V > 0) {
            float k = I * D * V;
            SDL_SetTextureColorMod(glow_, c8(ty.rgb[0] * k), c8(ty.rgb[1] * k), c8(ty.rgb[2] * k));
            SDL_FRect d{px - glow_r, py - glow_r, 2 * glow_r, 2 * glow_r};
            SDL_RenderTexture(r, glow_, nullptr, &d);
        }
        if (halo_r > 0.5f && ty.halo_gain > 0 && z > 1 && V > 0) { // the original's halo is depth-tested
            float k = I * ty.halo_gain * 0.5f;
            SDL_SetTextureColorMod(disc_, c8(ty.rgb[0] * k), c8(ty.rgb[1] * k), c8(ty.rgb[2] * k));
            SDL_FRect d{px - halo_r, py - halo_r, 2 * halo_r, 2 * halo_r};
            SDL_RenderTexture(r, disc_, nullptr, &d);
        }
    }
}

// --- Car lamps --------------------------------------------------------------------

CarLampSet CarLampSet::parse(ByteSpan g) {
    CarLampSet out;
    if (g.size() < 0x58) return out;
    auto vec = [&](size_t o) { return render::Vec3{rdf(g, o), rdf(g, o + 4), rdf(g, o + 8)}; };
    auto list = [&](size_t at, std::vector<CarLampRecord>& dst) {
        u32 n = rd32(g, at), off = rd32(g, at + 4);
        for (u32 i = 0; i < n && i < 32 && off + 0x80ull * (i + 1) <= g.size(); ++i) {
            size_t o = off + 0x80ull * i;
            CarLampRecord l;
            l.xaxis = vec(o), l.view_axis = vec(o + 0x20), l.pos = vec(o + 0x30), l.test = vec(o + 0x4C);
            l.a = rdf(g, o + 0x40), l.b = rdf(g, o + 0x44);
            l.rgb = {rdf(g, o + 0x58), rdf(g, o + 0x5C), rdf(g, o + 0x60)};
            l.exponent = int(rd32(g, o + 0x64));
            l.fade = (rd32(g, o + 0x68) & 1) != 0;
            dst.push_back(l);
        }
    };
    list(0x40, out.head);
    list(0x48, out.tail);
    list(0x50, out.brake);
    return out;
}

CarLampGlows::~CarLampGlows() {
    if (glow_) SDL_DestroyTexture(glow_);
    if (disc_) SDL_DestroyTexture(disc_);
}

void CarLampGlows::load(SDL_Renderer* r, ByteSpan env) {
    active_ = false;
    if (env.size() < 0xA1C || std::memcmp(env.data(), "Env0", 4) != 0 || !(rd32(env, 0x10) & 0x8000)) return;
    LampType& t = type_;
    t.rgb = {rdf(env, 0x9F0), rdf(env, 0x9F4), rdf(env, 0x9F8)};
    t.intensity = rdf(env, 0x9FC);
    t.glow = rdf(env, 0xA00) * 8.0f;
    t.halo = rdf(env, 0xA04);
    t.halo_gain = rdf(env, 0xA08);
    t.att_c = rdf(env, 0xA0C), t.att_l = rdf(env, 0xA10), t.att_q = rdf(env, 0xA14);
    t.max_dist = rdf(env, 0xA18);
    if (!glow_) glow_ = make_sprite(r, true);
    if (!disc_) disc_ = make_sprite(r, false);
    active_ = glow_ && disc_;
}

void CarLampGlows::draw(SDL_Renderer* r, const SDL_FRect& rect, const render::Mat4& view, const render::Vec3& eye,
                        const render::Frustum& fr, const std::vector<Car>& cars,
                        const std::function<bool(const render::Vec3&, const render::Vec3&)>& blocked) const {
    if (!active_) return;
    const LampType& ty = type_;
    auto c8 = [](float v) { return u8(std::clamp(v, 0.0f, 1.0f) * 255.0f); };
    auto xform = [](const render::Mat4& m, const render::Vec3& p) { return m.transform_point(p); };
    auto xdir = [](const render::Mat4& m, const render::Vec3& d) {
        return render::Vec3{m.at(0, 0) * d.x + m.at(0, 1) * d.y + m.at(0, 2) * d.z,
                            m.at(1, 0) * d.x + m.at(1, 1) * d.y + m.at(1, 2) * d.z,
                            m.at(2, 0) * d.x + m.at(2, 1) * d.y + m.at(2, 2) * d.z};
    };
    int drawn = 0;
    // kind 0 headlamp, 1 brake lamp, 2 tail lamp
    auto lamp = [&](const Car& car, const CarLampRecord& l, int kind, float I) {
        if (drawn >= 100) return;
        render::Vec3 tp = xform(car.model, l.test), gp = xform(car.model, l.pos);
        render::Vec3 axis = normalize(xdir(car.model, l.view_axis));
        float vx = view.at(0, 0) * gp.x + view.at(0, 1) * gp.y + view.at(0, 2) * gp.z + view.at(0, 3);
        float vy = view.at(1, 0) * gp.x + view.at(1, 1) * gp.y + view.at(1, 2) * gp.z + view.at(1, 3);
        float vz = view.at(2, 0) * gp.x + view.at(2, 1) * gp.y + view.at(2, 2) * gp.z + view.at(2, 3);
        float z = -vz;
        if (z <= 0.5f || (ty.max_dist > 0 && z > ty.max_dist)) return;
        // Directional factor at the test point: dot(viewing axis, camera -> lamp)^n.
        float D = 1;
        if (kind != 2 && l.exponent > 0) {
            D = std::pow(std::max(0.0f, dot(axis, normalize(tp - eye))), float(l.exponent));
        }
        float att = 1.0f / std::max(1e-3f, ty.att_c + ty.att_l * z + ty.att_q * z * z);
        // Pulled towards the camera by -0.5 x the viewing axis' view-space z (0.5 m head-on).
        float az = view.at(2, 0) * axis.x + view.at(2, 1) * axis.y + view.at(2, 2) * axis.z;
        float zz = std::max(0.3f, z + 0.5f * az);
        float px = fr.screen_x(vx, zz, rect.x, rect.w), py = fr.screen_y(vy, zz, rect.y, rect.h);
        float per_m = fr.per_metre(zz, rect.h);
        bool bar = std::fabs(l.a - l.b) > 1e-4f;
        float glow_r = bar || kind == 2 ? 0.0f : ty.glow * l.a * 1.17f * per_m;
        float hx = ty.halo * l.a * per_m, hy = hx;
        if (bar) { // stretched lamps: an elliptical halo along the lamp's projected x axis
            render::Vec3 xa = normalize(xdir(car.model, l.xaxis));
            float pxv = view.at(0, 0) * xa.x + view.at(0, 1) * xa.y + view.at(0, 2) * xa.z;
            hx = ty.halo * ((l.a - l.b) * std::fabs(pxv) + l.b) * per_m;
            hy = ty.halo * l.b * per_m;
        }
        float rad = std::max(glow_r, std::max(hx, hy));
        if (rad < 0.5f || px + rad < rect.x || px - rad > rect.x + rect.w || py + rad < rect.y || py - rad > rect.y + rect.h)
            return;
        if (blocked(eye, tp + normalize(eye - tp) * 0.2f)) return;
        ++drawn;
        float base[3] = {ty.rgb[0] * ty.intensity * l.rgb[0], ty.rgb[1] * ty.intensity * l.rgb[1],
                         ty.rgb[2] * ty.intensity * l.rgb[2]};
        if (glow_r > 0.5f && D > 0) {
            float k = att * I * D;
            SDL_SetTextureColorMod(glow_, c8(base[0] * k), c8(base[1] * k), c8(base[2] * k));
            SDL_FRect d{px - glow_r, py - glow_r, 2 * glow_r, 2 * glow_r};
            SDL_RenderTexture(r, glow_, nullptr, &d);
        }
        if (ty.halo_gain > 0 && z > 1 && (hx > 0.5f || hy > 0.5f)) { // not scaled by I or D
            float k = att * ty.halo_gain * 0.5f;
            SDL_SetTextureColorMod(disc_, c8(base[0] * k), c8(base[1] * k), c8(base[2] * k));
            SDL_FRect d{px - hx, py - hy, 2 * hx, 2 * hy};
            SDL_RenderTexture(r, disc_, nullptr, &d);
        }
    };
    for (const Car& car : cars) {
        if (!car.lamps) continue;
        for (const CarLampRecord& l : car.lamps->head) lamp(car, l, 0, 1.0f);
        if (car.k > 0)
            for (const CarLampRecord& l : car.lamps->brake) {
                float I = l.fade ? car.k : (car.brake_now ? 1.0f : 0.0f);
                if (I > 0) lamp(car, l, 1, I);
            }
        for (const CarLampRecord& l : car.lamps->tail) lamp(car, l, 2, 1.0f);
    }
}

} // namespace gt3r::race
