// Tests for the course decoders: container, GTRW road, AI lines, course model.

#include "test_framework.h"

#include "formats/course/course_model.h"

#include <cmath>
#include <cstring>

using namespace gt3r;
using namespace gt3r::course;

namespace {

void put32(Bytes& b, size_t at, u32 v) {
    if (b.size() < at + 4) b.resize(at + 4);
    for (int i = 0; i < 4; ++i) b[at + i] = u8(v >> (8 * i));
}
void put16(Bytes& b, size_t at, u16 v) {
    if (b.size() < at + 2) b.resize(at + 2);
    b[at] = u8(v), b[at + 1] = u8(v >> 8);
}
void putf(Bytes& b, size_t at, float f) {
    u32 v;
    std::memcpy(&v, &f, 4);
    put32(b, at, v);
}

// A GTRW with one segment (a flat 10x10 quad at y = 2), one grid slot,
// one wall polyline and two checkpoints.
Bytes make_gtrw() {
    Bytes b(0x200, 0);
    std::memcpy(b.data(), "GTRW", 4);
    put32(b, 0x14, 0x1234);
    putf(b, 0x18, 100.f);
    put16(b, 0x22, 1), put32(b, 0x24, 0x60); // marker
    put32(b, 0x60, 2), putf(b, 0x64, 50.f);
    put32(b, 0x28, 1), put32(b, 0x2C, 0x70); // grid
    putf(b, 0x70, 5), putf(b, 0x74, 0), putf(b, 0x78, 5), putf(b, 0x7C, 1.5f);
    put32(b, 0x30, 1), put32(b, 0x34, 0x80); // segments
    put16(b, 0x80, 4), put16(b, 0x82, 2), put32(b, 0x84, 0x100), put32(b, 0x88, 0x140), put32(b, 0x90, 0x160);
    const float v[4][2] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    for (int i = 0; i < 4; ++i) {
        putf(b, 0x100 + 16 * i, v[i][0]), putf(b, 0x104 + 16 * i, 2.f), putf(b, 0x108 + 16 * i, v[i][1]);
        put32(b, 0x10C + 16 * i, 0xFFFFFFFF);
    }
    put16(b, 0x140, 0), put16(b, 0x142, 1), put16(b, 0x144, 2), put32(b, 0x14C, 8);
    put16(b, 0x150, 0), put16(b, 0x152, 2), put16(b, 0x154, 3), put32(b, 0x15C, 8);
    put32(b, 0x38, 3), put32(b, 0x3C, 0x170); // wall: 3 points, one polyline
    for (int i = 0; i < 3; ++i) putf(b, 0x170 + 16 * i, float(i));
    put32(b, 0x17C, 3), put32(b, 0x19C, u32(-3) & 0xFFFF);
    put32(b, 0x40, 2), put32(b, 0x44, 0x1A0); // checkpoints
    putf(b, 0x1A0, 0), putf(b, 0x1A4, 2), putf(b, 0x1A8, 10), putf(b, 0x1AC, 2), putf(b, 0x1B0, 0);
    putf(b, 0x1C0, 0), putf(b, 0x1C4, 8), putf(b, 0x1C8, 10), putf(b, 0x1CC, 8), putf(b, 0x1D0, 6);
    return b;
}

} // namespace

TEST(course_gtrw_synthetic) {
    Bytes b = make_gtrw();
    Road r = Road::parse(b);
    CHECK(r.course_id == 0x1234);
    CHECK(r.length == 100.f);
    CHECK(r.markers.size() == 1 && r.markers[0].distance == 50.f);
    CHECK(r.grid.size() == 1 && r.grid[0].yaw == 1.5f);
    CHECK(r.segments.size() == 1 && r.triangle_count() == 2);
    CHECK(r.segments[0].triangles[0].surface == 8);
    CHECK(r.walls.size() == 1 && r.walls[0].size() == 3);
    u32 surface = 0;
    auto h = r.height_at(3, 7, 1e30f, &surface);
    CHECK(h && std::fabs(*h - 2.f) < 1e-5f && surface == 8);
    CHECK(!r.height_at(11, 5));
    std::vector<Vec3> c = r.centre_line();
    CHECK(c.size() == 2 && std::fabs(c[1].z - 8.f) < 1e-5f && std::fabs(c[1].y - 2.f) < 1e-5f);

    Bytes bad = b;
    put16(bad, 0x144, 9); // triangle index out of range
    bool threw = false;
    try {
        Road::parse(bad);
    } catch (const FormatError&) {
        threw = true;
    }
    CHECK(threw);
}

TEST(course_trialmt) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    const Volume::Entry* e = disc->volume().find("courses/US/trialmt");
    CHECK(e != nullptr);
    if (!e) return;
    Bytes data = disc->volume().read(*e);
    CHECK(CourseFile::looks_like_course(data));
    Course c = load_course(data);
    CHECK(c.file.find(kSlotRoad) && c.file.find(kSlotRoad)->tag == "GTRW");
    CHECK(c.file.find(kSlotScenery) && c.file.find(kSlotScenery)->tag == "GTM1");
    CHECK(std::fabs(c.road.length - 3922.f) < 1.f);
    CHECK(c.road.segments.size() == 114);
    CHECK(c.road.checkpoints.size() == 254);
    CHECK(c.road.grid.size() == 8);
    CHECK(c.road.walls.size() == 3);
    // Every grid slot sits on the road.
    for (const GridSlot& g : c.road.grid) CHECK(c.road.height_at(g.pos.x, g.pos.z).has_value());
    // AI line 0: control points lie on the road (X mirrored from the sim frame).
    CHECK(c.ai.lines.size() == 7 && c.ai.lines[0].points.size() == 33);
    size_t on_road = 0;
    for (const AiPoint& p : c.ai.lines[0].points) on_road += c.road.height_at(p.pos.x, p.pos.z).has_value();
    CHECK(on_road * 10 >= c.ai.lines[0].points.size() * 9);
    std::vector<Vec3> line = c.ai.lines[0].sample(c.road, 2.f);
    CHECK(line.size() > 1500); // a full ~3.9 km lap
    size_t line_on_road = 0;
    for (const Vec3& p : line) line_on_road += c.road.height_at(p.x, p.z).has_value();
    CHECK(line_on_road * 100 >= line.size() * 98);

    CourseModelOptions opt;
    opt.textures = false;
    scene::Model m = build_course_model(data, opt, "trialmt");
    size_t tris = 0;
    for (const scene::Mesh& mesh : m.meshes) tris += mesh.indices.size() / 3;
    CHECK(m.meshes.size() > 100 && tris > 50000);
}

TEST(course_all_us) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    size_t files = 0, ok = 0;
    for (const auto& e : disc->volume().entries()) {
        if (e.path.rfind("courses/US/", 0) != 0) continue;
        ++files;
        Bytes data = disc->volume().read(e);
        try {
            Course c = load_course(data);
            bool good = !c.road.segments.empty() && c.road.length > 0 && c.ai.lines.size() == 7;
            CourseModelOptions opt;
            opt.textures = false;
            good &= !build_course_model(data, opt).meshes.empty();
            ok += good;
        } catch (const std::exception& ex) {
            std::printf("  %s: %s\n", e.path.c_str(), ex.what());
        }
    }
    CHECK(files == 80);
    CHECK(ok == files);
}
