// Mod cars (game/data/mod_cars.h), electric drive and the synthesised motor
// sound. Synthetic tests run everywhere; the end-to-end test needs $GT3R_DISC.

#include "test_framework.h"

#include "audio/motor_sound.h"
#include "formats/model/car_model.h"
#include "formats/ps_adpcm.h"
#include "formats/sound/spu.h"
#include "game/data/car_database.h"
#include "game/data/mod_cars.h"
#include "sim/car_spec.h"
#include "sim/vehicle.h"

#include <cmath>
#include <vector>

using namespace gt3r;
using namespace gt3r::test;

namespace {

struct Flat : sim::TrackQuery {
    sim::SurfaceHit probe(float, float, float y_max) const override {
        sim::SurfaceHit h;
        h.hit = y_max >= 0;
        return h;
    }
};

const char* kMinimal = R"(
; a comment
[car]
label = test_ev
name = Test EV          ; trailing comment
donor = legacy_gtb_96
year = 2026

[powertrain]
type = electric
power_kw = 300
torque_nm = 600
max_rpm = 16000

[drivetrain]
drive = AWD
front_torque_pct = 35
gears = 9.04

[tires]
front = 255/35R21
rear = 275/35R21

[raw.chassis]
0x1D = 120
)";

} // namespace

TEST(modcar_tire_size) {
    auto t = data::parse_tire_size("255/35R21");
    CHECK(t && t->width_mm_5 == 51 && t->aspect_5 == 7 && t->rim_in == 21);
    auto z = data::parse_tire_size("225/60 ZR18");
    CHECK(z && z->width_mm_5 == 45 && z->aspect_5 == 12 && z->rim_in == 18);
    CHECK(!data::parse_tire_size("253/35R21")); // not a multiple of 5: the disc stores width / 5
    CHECK(!data::parse_tire_size("255-35-21"));
}

TEST(modcar_electric_curve) {
    auto c = data::electric_curve(600, 300, 16000);
    CHECK(c.size() >= 10 && c.size() <= 16);
    CHECK(c.front().first == 0 && std::fabs(c.front().second - 600) < 1e-3f);
    for (size_t i = 1; i < c.size(); ++i) {
        CHECK(c[i].first > c[i - 1].first);
        CHECK(c[i].second <= c[i - 1].second + 1e-3f); // never rises
        CHECK(std::fmod(c[i].first, 100.0f) == 0);    // the axis is stored in 100 rpm steps
    }
    CHECK(c.back().first == 16000);
    // Constant power above the base speed (300 kW / 600 N*m = 4775 rpm), 85 % at the top.
    float kw_top = c.back().second * c.back().first * 0.10471976f / 1000;
    CHECK(std::fabs(kw_top - 255) < 3);
}

TEST(modcar_parse) {
    std::string err;
    std::vector<std::string> warns;
    auto d = data::parse_mod_car(kMinimal, &err, &warns);
    CHECK(d.has_value());
    CHECK(warns.empty());
    CHECK(d->label == "test_ev" && d->name == "Test EV" && d->donor == "legacy_gtb_96");
    CHECK(d->electric && d->max_rpm && *d->max_rpm == 16000);
    CHECK(d->drive && *d->drive == 2 && d->front_torque_pct && *d->front_torque_pct == 35);
    CHECK(d->gears.size() == 1 && std::fabs(d->gears[0] - 9.04f) < 1e-4f);
    CHECK(d->front_tire && d->rear_tire && d->rear_tire->width_mm_5 == 55);
    CHECK(d->pokes.size() == 1 && d->pokes[0].table == data::TableId::Chassis && d->pokes[0].offset == 0x1D);

    // Hard errors name the problem.
    CHECK(!data::parse_mod_car("[car]\nlabel = x\nname = X\n", &err) && err.find("donor") != std::string::npos);
    CHECK(!data::parse_mod_car("[car]\nlabel = Bad Label\nname = X\ndonor = y\n", &err));
    CHECK(!data::parse_mod_car("[car]\nlabel = x\nname = X\ndonor = y\n[powertrain]\ntype = electric\n", &err));
    // Unknown keys are warnings, not errors.
    warns.clear();
    CHECK(data::parse_mod_car("[car]\nlabel = x\nname = X\ndonor = y\nspoiler = big\n", &err, &warns).has_value());
    CHECK(warns.size() == 1);
}

TEST(adpcm_encode_roundtrip) {
    std::vector<s16> pcm(28 * 200);
    for (size_t i = 0; i < pcm.size(); ++i)
        pcm[i] = s16(12000 * std::sin(i * 0.05) + 6000 * std::sin(i * 0.31));
    Bytes enc = ps_adpcm_encode(pcm.data(), pcm.size(), nullptr);
    CHECK(enc.size() == 200 * 16);
    PsAdpcmDecoder dec;
    double sig = 0, err = 0;
    s16 out[28];
    for (size_t f = 0; f < 200; ++f) {
        dec.decode_frame(enc.data() + f * 16, out);
        for (size_t i = 0; i < 28; ++i) {
            double s = pcm[f * 28 + i], e = out[i] - s;
            sig += s * s;
            err += e * e;
        }
    }
    CHECK(10 * std::log10(sig / std::max(err, 1.0)) > 30); // dB
}

TEST(motor_sound_banks) {
    MotorBanks b = make_motor_banks({});
    CHECK(b.whine && b.inverter);
    for (const auto* bank : {b.whine.get(), b.inverter.get()}) {
        CHECK(bank->layers.size() == 3);
        for (size_t i = 0; i < bank->layers.size(); ++i) {
            if (i) CHECK(bank->layers[i].base_rpm > bank->layers[i - 1].base_rpm);
            sound::SpuSample s = bank->decode_layer(i);
            CHECK(s.looped && s.loop_start == 28); // silent frame, then the loop
            int peak = 0;
            for (s16 v : s.pcm) peak = std::max(peak, std::abs(int(v)));
            CHECK(peak > 10000);
        }
    }
}

TEST(body_fit_maps_axles) {
    scene::Model m;
    scene::Mesh body;
    body.name = "body";
    for (float z : {-1.3f, 1.3f, -2.3f, 2.3f}) {
        scene::Vertex v;
        v.pos[0] = 0.85f, v.pos[1] = 1.4f, v.pos[2] = z;
        v.normal[1] = 1;
        body.vertices.push_back(v);
    }
    scene::Mesh wheel = body;
    wheel.name = "wheel_fl";
    m.meshes = {body, wheel};
    // 4600 x 1700 x 1400, wheelbase 2600 -> 4800 x 1900 x 1600, wheelbase 2900.
    fit_body(m, {4600, 2600, 1700, 1400, 4800, 2900, 1900, 1600});
    const auto& v = m.meshes[0].vertices;
    CHECK(std::fabs(v[0].pos[2] + 1.45f) < 1e-4f && std::fabs(v[1].pos[2] - 1.45f) < 1e-4f); // axles
    CHECK(std::fabs(v[2].pos[2] + 2.40f) < 1e-3f); // front overhang 1.0 m -> 0.95 m (1900 / 2000)
    CHECK(std::fabs(v[0].pos[0] - 0.85f * 1900 / 1700) < 1e-4f);
    CHECK(std::fabs(v[0].pos[1] - 1.6f) < 1e-4f);
    CHECK(m.meshes[1].vertices[0].pos[2] == -1.3f); // wheels untouched
}

TEST(electric_vehicle_launch) {
    // An R34-like test car turned into a single-speed EV: pulls away from rest
    // with no clutch slip and no idle, and regen slows it on lift-off.
    sim::CarSpec s = sim::CarSpec::test_car(sim::Drive::AWD);
    s.engine.electric = true;
    s.engine.curve_rpm = {0, 5000, 10000, 16000};
    s.engine.curve_torque = {280, 280, 200, 120}; // within the tyres' grip: no traction control on this car
    s.engine.rev_limit_rpm = 16000;
    s.engine.inertia = 0.06f;
    s.engine.regen_torque = 150;
    s.engine.regen_power = 100000;
    s.gearbox.ratios = {1.0f};
    s.gearbox.final_drive = 9.0f;
    s.gearbox.reverse = 1.0f;
    s.finalize();
    CHECK(s.engine.idle_rpm == 0);
    auto car = sim::Vehicle::create(s);
    Flat g;
    car->reset({}, {});
    sim::ControlInputs in;
    in.throttle = 1;
    const double h = 1.0 / 60;
    for (int i = 0; i < 60; ++i) car->step(in, g, h);
    float v1 = car->state().speed_kmh;
    CHECK(v1 > 15); // moving after 1 s
    CHECK(car->state().gear == 1);
    // Motor speed follows the wheels through the reduction (direct drive).
    float wheel_rpm = v1 / 3.6f / s.axle[1].tire_radius * 60 / (2 * 3.14159265f);
    CHECK(std::fabs(car->state().engine_rpm - wheel_rpm * 9.0f) < 0.1f * wheel_rpm * 9.0f + 50);
    for (int i = 0; i < 240; ++i) car->step(in, g, h);
    float v5 = car->state().speed_kmh;
    CHECK(v5 > 70);
    in.throttle = 0;
    for (int i = 0; i < 60; ++i) car->step(in, g, h);
    CHECK(car->state().speed_kmh < v5 - 8); // regen: > ~0.2 g including drag
}

TEST(modcar_disc_end_to_end) {
    Disc* disc = test_disc();
    if (!disc) SKIP("GT3R_DISC not set");
    data::CarDatabase db;
    CHECK(db.load(disc->volume()));
    size_t cars = db.cars().size();
    std::string err;
    auto def = data::parse_mod_car(kMinimal, &err);
    CHECK(def.has_value());
    CHECK(data::add_mod_car(db, *def, &err));
    db.rebuild();
    CHECK(db.cars().size() == cars + 1);
    const data::Car* c = db.car("test_ev");
    CHECK(c && c->name == "Test EV" && c->year == 2026);
    CHECK(db.model_of(*c) == db.model_of(*db.car("legacy_gtb_96"))); // borrowed shell
    CHECK(db.car_by_model(db.model_of(*c))->label == "legacy_gtb_96"); // the model still maps to the disc car
    auto spec = sim::build_car_spec(db, "test_ev", &err);
    CHECK(spec.has_value());
    CHECK(spec->engine.electric && spec->gearbox.ratios.size() == 1);
    CHECK(std::fabs(spec->gearbox.ratios[0] * spec->gearbox.final_drive - 9.04f) < 0.01f);
    CHECK(std::fabs(spec->peak_torque() - 600) < 6);
    CHECK(std::fabs(spec->axle[1].tire_width - 0.275f) < 1e-4f);
    // Upgrade parts were cloned for the new car (tyres, suspension), engine parts were not.
    CHECK(!db.parts_for(data::TableId::FrontTire, "test_ev").empty());
    CHECK(db.parts_for(data::TableId::TurbineKit, "test_ev").empty());
    CHECK(db.mod_car("test_ev") && db.mod_car("test_ev")->fit.active());
    // A second definition with a disc car's label is refused.
    def->label = "legacy_gtb_96";
    CHECK(!data::add_mod_car(db, *def, &err));
}

// --- custom bodies: JSON, PNG and binary glTF -------------------------------------

#include "formats/json.h"
#include "formats/model/gltf.h"
#include "formats/png_reader.h"
#include "formats/png_writer.h"

#include <cstring>

TEST(json_parse) {
    auto v = json::Value::parse(R"({"a": [1, 2.5, -3e2], "s": "xé\n", "t": true, "n": null, "o": {"k": 1}})");
    CHECK(v["a"].size() == 3 && v["a"][2].number() == -300);
    CHECK(v["s"].string() == "x\xC3\xA9\n");
    CHECK(v["t"].boolean() && v["n"].is_null() && v["o"]["k"].integer() == 1);
    CHECK(v["missing"]["deeper"].is_null()); // lookups chain safely
    bool threw = false;
    try {
        json::Value::parse("{\"a\": [1, 2}");
    } catch (const FormatError&) {
        threw = true;
    }
    CHECK(threw);
}

TEST(png_roundtrip) {
    std::vector<u32> px(7 * 5);
    for (size_t i = 0; i < px.size(); ++i) px[i] = u32(i * 2654435761u) | 0xFF000000u;
    PngImage img = decode_png(encode_png(7, 5, px));
    CHECK(img.width == 7 && img.height == 5 && img.rgba == px);
}

namespace {
// One triangle with a "paint" material, facing +Z (the glTF front), 1 m in front of the origin.
Bytes tiny_glb() {
    float pos[9] = {0, 0, 1, 1, 0, 1, 0, 1, 1};
    u16 idx[4] = {0, 1, 2, 0};
    Bytes bin(sizeof(pos) + sizeof(idx));
    std::memcpy(bin.data(), pos, sizeof(pos));
    std::memcpy(bin.data() + sizeof(pos), idx, sizeof(idx));
    std::string js =
        R"({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"mesh":0}],)"
        R"("meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1,"material":0}]}],)"
        R"("materials":[{"name":"Body Paint","pbrMetallicRoughness":{"baseColorFactor":[1,1,1,1]}}],)"
        R"("buffers":[{"byteLength":44}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},)"
        R"({"buffer":0,"byteOffset":36,"byteLength":6}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"},)"
        R"({"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}]})";
    while (js.size() % 4) js += ' ';
    Bytes out(12);
    auto put32 = [&](u32 v) { size_t o = out.size(); out.resize(o + 4); std::memcpy(out.data() + o, &v, 4); };
    std::memcpy(out.data(), "glTF", 4);
    u32 v2 = 2, total = u32(12 + 8 + js.size() + 8 + bin.size());
    std::memcpy(out.data() + 4, &v2, 4);
    std::memcpy(out.data() + 8, &total, 4);
    put32(u32(js.size()));
    put32(0x4E4F534A);
    out.insert(out.end(), js.begin(), js.end());
    put32(u32(bin.size()));
    put32(0x004E4942);
    out.insert(out.end(), bin.begin(), bin.end());
    return out;
}
} // namespace

TEST(glb_import) {
    GlbOptions o;
    o.paint = std::array<float, 3>{1.0f, 0.0f, 0.0f};
    scene::Model m = load_glb(tiny_glb(), o, "tiny");
    CHECK(m.meshes.size() == 2 && m.meshes[0].name == "body" && m.meshes[1].name == "body_env");
    const scene::Mesh& b = m.meshes[0];
    CHECK(b.vertices.size() == 3 && b.indices.size() == 3 && b.primitives.size() == 1);
    // glTF faces +Z; GT3 noses point to -Z and +X is the car's right: a half turn about Y.
    CHECK(b.vertices[1].pos[0] == -1 && b.vertices[0].pos[2] == -1);
    CHECK(b.primitives[0].flags & scene::kTintable);
    CHECK(b.vertices[0].color == 0xFF0000FFu); // the paint colour (RGBA8 in memory order R, G, B, A)
    CHECK(b.vertices[0].normal[2] < -0.99f);   // computed normal, turned with the model (faces -Z now)
    CHECK(m.meshes[1].primitives[0].flags & scene::kEnvironmentMap);
}

// --- the shipped example mod (examples/mods/example_coupe) --------------------------
// Loads it the way the game does, checks its body and drives it, so the example that
// docs/MODDING.md points modders to can't silently stop working.

#include "base/file.h"
#include "formats/model/wheel_spec.h"

TEST(example_mod_car) {
    Disc* disc = test_disc();
    if (!disc) SKIP("GT3R_DISC not set");
    const std::string dir = std::string(GT3R_SOURCE_DIR) + "/examples/mods/example_coupe";
    const std::string label = "gt3r_concept_coupe_24";
    data::CarDatabase db;
    CHECK(db.load(disc->volume()));

    // The definition parses without errors or warnings (every key is one the loader knows).
    auto text = read_host_file(dir + "/cars/" + label + ".ini");
    CHECK(text.has_value());
    if (!text) return;
    std::string err;
    std::vector<std::string> warns;
    auto def = data::parse_mod_car(std::string_view(reinterpret_cast<const char*>(text->data()), text->size()), &err, &warns);
    CHECK(def.has_value() && warns.empty());
    for (const std::string& w : warns) std::printf("    warning: %s\n", w.c_str());
    CHECK(def && def->paints.size() == 6 && !def->model_file.empty() && !def->wheel.empty());

    data::load_mod_maker_file(db, dir + "/makers/gt3r.ini", "example_coupe");
    data::load_mod_car_files(db, {dir + "/cars/" + label + ".ini"}, "example_coupe");
    db.rebuild();
    CHECK(db.mod_car_errors().empty());
    const data::Car* c = db.car(label);
    CHECK(c != nullptr);
    if (!c) return;
    auto s = db.summarize(*c);
    CHECK(s.maker == "GT3R" && s.weight_kg == 1420 && s.wheelbase_mm == 2799);
    CHECK(db.mod_makers().size() == 1 && db.mod_makers()[0].region == "gr");
    const data::ModCar* m = db.mod_car(label);
    CHECK(m && m->for_sale && !m->picture.empty() && !m->thumb.empty());
    if (!m) return;
    for (const std::string* png : {&m->picture, &m->thumb}) {
        auto b = read_host_file(*png);
        CHECK(b.has_value());
        if (b) {
            PngImage img = decode_png(*b);
            CHECK((img.width == 640 && img.height == 448) || (img.width == 51 && img.height == 27));
        }
    }

    // The body: a glTF the loader reads, sized like the [body] numbers, with paint and glass.
    auto glb = read_host_file(m->model_file);
    CHECK(glb.has_value() && glb->size() < 4u * 1024 * 1024);
    if (!glb) return;
    scene::Model body = load_glb(*glb, GlbOptions{}, label);
    size_t tris = 0;
    bool paint = false, glass = false;
    for (const scene::Mesh& mesh : body.meshes) {
        if (mesh.name != "body") continue;
        tris += mesh.indices.size() / 3;
        for (const scene::Primitive& p : mesh.primitives) {
            paint = paint || (p.flags & scene::kTintable);
            glass = glass || (p.flags & scene::kAlphaBlend);
        }
    }
    CHECK(tris > 20000 && tris < 60000);
    CHECK(paint && glass);
    // GT3 axes: nose towards -Z, ground at y = 0, origin at the wheelbase midpoint.
    CHECK(std::fabs((body.bounds_max[2] - body.bounds_min[2]) - 4.357f) < 0.02f);
    CHECK(std::fabs((body.bounds_max[0] - body.bounds_min[0]) - 2.34f) < 0.03f); // with the mirrors
    CHECK(body.bounds_min[2] < -2.2f && body.bounds_max[2] < 2.1f);               // longer front overhang
    CHECK(body.bounds_min[1] > 0.05f && body.bounds_min[1] < 0.15f && std::fabs(body.bounds_max[1] - 1.149f) < 0.02f);
    // The engine's wheels match the modelled ones (768 mm across).
    auto ws = wheel_spec_for_car(db, label);
    CHECK(ws.has_value() && std::fabs(ws->radius[0] - 0.384f) < 0.01f && std::fabs(ws->radius[1] - 0.384f) < 0.01f);

    // It drives: full throttle on a flat plane passes 100 km/h in under 8 s and keeps pulling.
    auto spec = sim::build_car_spec(db, label, &err);
    CHECK(spec.has_value());
    if (!spec) return;
    auto car = sim::Vehicle::create(*spec);
    Flat g;
    car->reset({}, {});
    sim::ControlInputs in;
    in.throttle = 1;
    const double h = 1.0 / sim::kDefaultTickHz;
    for (int i = 0; i < int(8 / h); ++i) car->step(in, g, h);
    CHECK(car->state().speed_kmh > 100);
    for (int i = 0; i < int(12 / h); ++i) car->step(in, g, h);
    CHECK(car->state().speed_kmh > 180 && car->state().speed_kmh < 290);
}
