#pragma once

// Track runtime (milestone M2): everything the simulation and the race scene
// need to know about a course at run time, built from the decoded GTRW road
// data (src/formats/course/road.h) and AI lines.
//
//   auto track = track::Track::load(vfs, "trialmt");          // courses/US/trialmt
//   sim::SurfaceHit h = track->probe(x, z, wheel_y + 1.0f);  // sim::TrackQuery
//   track::Pose p = track->grid_pose(0);                     // start slot 0
//   track::LapTimer timer(*track);  timer.update(car_pos, race_time);
//
// Frames and units are those of src/sim/sim_types.h: world = right-handed,
// +Y up, metres; car frame nose towards -Z. Track distance ("s") is metres
// along the lap from the start/finish line, in [0, length()).
//
// See docs/dev/TRACK.md for behaviour details (off-road fallback, bridges,
// wall contact, lap timing rules).

#include "formats/course/ai_lines.h"
#include "formats/course/road.h"
#include "sim/sim_types.h"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace gt3r {
class Vfs;
}

namespace gt3r::track {

// Raw SurfaceHit::surface values produced by Track::probe():
//   bits 0-7  material (GTRW triangle +0x08), bits 8-15 flags (triangle +0x0C).
// Material meanings come from where each value lies on the courses (top-down
// maps of all 80 US courses); see docs/formats/course.md for the evidence.
enum class Surface : u8 {
    Tarmac,   // material 0: racing surface (and pit lane)
    Kerb,     // material 1: thin strips along corner edges
    Grass,    // material 2: verges
    Gravel,   // material 3: wide run-off on corner exits (Midfield, Laguna, Slalom)
    Sand,     // material 4: run-off traps, Laguna Seca only
    Dirt,     // material 5: rally stages (Tahiti, Swiss Alps)
    Paving,   // material 7: road sections in Rome / Grand Valley / Deep Forest (cobbles?)
    Loose,    // material 8: patches on Swiss Alps (smtsouth); name is a guess (the game gives it the road tyre compound)
    Rough,    // material 9: road sections on Akasaka and Special Stage Route 5
    OffRoad,  // no road triangle below the point (see TRACK.md)
    Unknown,  // material 6 or > 9 (not seen on the disc)
};

const char* surface_name(Surface s);
// Typed class of a raw SurfaceHit::surface value (from its material byte).
Surface surface_type(u32 raw);
inline u8 surface_material(u32 raw) { return u8(raw); }
inline u8 surface_flags(u32 raw) { return u8(raw >> 8); }
// Flag bits (triangle +0x0C).
constexpr u8 kSurfacePitLane = 0x01; // pit lane: the game counts wheels on it (pit logic)
constexpr u8 kSurfaceWet = 0x02;     // wet road: wet circle tests and Route 5 (not route5_dry)
constexpr u8 kSurfaceFlag4 = 0x04;   // only on 80 triangles of L_testcourse1
constexpr u8 kSurfaceCovered = 0x08; // tunnels / covered sections (hypothesis)
constexpr u8 kSurfaceZone = 0x10;    // stripes across the road and some zones (hypothesis)
// The surface index the game's tyre code uses (0..14): the material, except
// on wet triangles where materials 0 and 7 become 12, 1 becomes 13, 9 becomes 14.
int surface_physics_index(u32 raw);

// A placement in the world: position on the road surface and orientation of
// the car frame (nose -Z) facing along the lap.
struct Pose {
    sim::Vec3 position;
    sim::Quat orientation;
    sim::Vec3 forward{0, 0, -1}; // world direction of the car's nose (unit, horizontal)
    float yaw = 0;               // rotation about +Y applied to the car frame (radians)
};

// Projection of a point onto the lap.
struct TrackPosition {
    float distance = 0;   // metres along the lap from the start line, [0, length)
    float lateral = 0;    // signed offset from the centre line, + = left of travel direction
    sim::Vec3 centre;     // closest centre-line point (y from the road)
    sim::Vec3 direction;  // unit travel direction of the centre line there (horizontal)
};

// Wall point flags (upper half of the GTRW wall point tag, read by the game
// as the u16 at point +0x0E).
constexpr u16 kWallCorner = 0x0001; // vertex also tested against the car's edges
constexpr u16 kWallSensor = 0x0002; // segment starting here only records crossings (none on the US disc)

struct WallPoint {
    sim::Vec3 pos;
    u16 flags = 0;
};

// One wall polyline. When `closed`, the last point links back to the first
// (the game's loops: the last tag holds -count); edge i runs from point i to
// point (i + 1) % size.
struct WallLine {
    std::vector<WallPoint> points;
    bool closed = false;
};

// Contact between a sphere and the trackside walls/barriers. Walls are
// vertical: the test is done in the XZ plane against wall polylines.
struct WallContact {
    bool hit = false;
    sim::Vec3 point;       // closest point on the wall (y = query y)
    sim::Vec3 normal;      // horizontal unit vector from the wall towards the query centre
    float penetration = 0; // radius - distance (> 0 when hit)
    int wall = -1;         // index into walls()
    int edge = -1;         // segment index within that polyline
};

class Track : public sim::TrackQuery {
public:
    // Loads courses/<region>/<label> through the VFS. Throws FormatError when
    // the file is missing or malformed.
    static std::unique_ptr<Track> load(const Vfs& vfs, std::string_view label, std::string_view region = "US");
    // From an already read (decompressed) course file.
    static std::unique_ptr<Track> from_course_file(ByteSpan data, std::string label = {});
    // From a decoded road (and optional AI lines); used by tests.
    Track(course::Road road, course::AiLines ai = {}, std::string label = {});
    ~Track() override;

    Track(const Track&) = delete;
    Track& operator=(const Track&) = delete;

    const std::string& label() const { return label_; }
    const course::Road& road() const { return road_; }
    const course::AiLines& ai_lines() const { return ai_; }
    float length() const { return length_; }

    // --- surface (sim::TrackQuery) ------------------------------------------
    // Highest road triangle under (x, z) at or below y_max. Uses a uniform
    // grid built at load time; overlapping road (bridges, tunnels) resolves by
    // y_max. Misses return hit = false, surface = 0, segment = -1 (OffRoad).
    sim::SurfaceHit probe(float x, float z, float y_max) const override;
    // Like probe(), but never "misses": off the road mesh it returns a ground
    // estimate (height of the nearest road edge, normal up, surface OffRoad);
    // hit is false in that case. See TRACK.md.
    sim::SurfaceHit probe_or_ground(float x, float z, float y_max) const;

    // --- start grid ----------------------------------------------------------
    int grid_count() const { return int(grid_.size()); }
    // Slot i (0 = pole). Position on the road surface; facing along the lap.
    Pose grid_pose(int slot) const;

    // --- walls ---------------------------------------------------------------
    const std::vector<std::vector<course::Vec3>>& walls() const { return road_.walls; }
    // The same polylines with per-point flags and closure (edge indices in
    // WallContact refer to these, including the closing edge).
    const std::vector<WallLine>& wall_lines() const { return wall_lines_; }
    // Deepest contact of a sphere (centre, radius) with any solid wall edge
    // (sensor edges are skipped; closed lines include their closing edge),
    // ignoring walls whose height span (with a few metres of margin) does not
    // include centre.y. hit = false when nothing is within `radius`.
    WallContact wall_contact(const sim::Vec3& centre, float radius) const;
    // All contacts (one per wall edge touched), for box-shaped bodies the
    // caller can test each corner sphere.
    void wall_contacts(const sim::Vec3& centre, float radius, std::vector<WallContact>& out) const;

    // --- lap geometry --------------------------------------------------------
    // Projects (x, z) onto the centre line. `hint` (a previous distance, or a
    // negative value for none) restricts the search to +-hint_window metres,
    // which keeps the result stable where the track passes near itself.
    TrackPosition locate(const sim::Vec3& p, float hint = -1, float hint_window = 150) const;
    // Centre-line point at track distance s (wrapped) with lateral offset d.
    sim::Vec3 point_at(float s, float lateral = 0) const;
    sim::Vec3 direction_at(float s) const;
    // A pose on the centre line (lateral 0) at track distance s, facing along
    // the lap; used to respawn a car that left the track.
    Pose respawn_pose(float s) const;

    // AI line `index` (0 = racing line; < 0 = the first non-empty line of
    // 0, 1, 2, 5, 6, since some courses leave line 0 empty) as a dense polyline about `step`
    // metres apart that follows the road: each control point is located as
    // (distance, lateral) and the line is interpolated in that space, so its
    // track distance increases monotonically. Empty if the line is absent.
    std::vector<sim::Vec3> racing_line(int index = -1, float step = 2.0f) const;

    // Checkpoint lines in lap order (by distance).
    const std::vector<course::Checkpoint>& checkpoints() const { return cps_; }

    // Diagnostics: "game" when the course's own lookup grid is used, "built"
    // when it was missing or failed validation and a grid was built instead.
    const char* index_source() const;

    struct Impl;

private:
    std::string label_;
    course::Road road_;
    course::AiLines ai_;
    float length_ = 0;
    std::vector<course::GridSlot> grid_;
    std::vector<course::Checkpoint> cps_; // sorted by distance
    std::vector<WallLine> wall_lines_;
    std::unique_ptr<Impl> impl_;
};

// Lap timing for one car. Track distance is followed continuously (each
// update searches near the previous distance), and the unwrapped progress
// must climb through a whole lap length before the next start-line crossing
// counts, so reversing over the line or cutting across the infield does not
// count laps. Sectors split the lap into `sector_count` equal track-distance
// parts unless set explicitly.
class LapTimer {
public:
    explicit LapTimer(const Track& track, int sector_count = 3);

    // Feed the car position each tick (or every few ticks) with the race clock
    // in seconds. Returns true on the tick a lap was completed.
    bool update(const sim::Vec3& position, double time);
    // Resets to "before the start line"; the first crossing starts lap 1.
    void reset(double time = 0);
    // Re-locates the car (after a respawn) without counting progress.
    void teleport(const sim::Vec3& position);

    void set_sector_distances(std::vector<float> ends); // ascending, last = lap length

    int laps_completed() const { return laps_; }
    bool lap_started() const { return started_; }
    double current_lap_time(double now) const { return started_ ? now - lap_start_ : 0; }
    double last_lap_time() const { return last_lap_; }          // < 0 until a lap is done
    double best_lap_time() const { return best_lap_; }          // < 0 until a lap is done
    int current_sector() const { return sector_; }
    const std::vector<double>& sector_times() const { return sector_times_; }      // current lap
    const std::vector<double>& last_sector_times() const { return last_sectors_; } // previous lap
    float distance() const { return distance_; }         // current track distance
    float lap_progress() const;                          // 0..1 within the current lap
    // Total race distance travelled (laps * length + distance); monotonic when
    // driving forwards, decreases when reversing.
    double race_distance() const;
    // True after the car has moved backwards along the lap for a while.
    bool wrong_way() const { return wrong_way_; }

private:
    const Track& track_;
    int sector_count_;
    std::vector<float> sector_ends_;
    float distance_ = 0;
    bool located_ = false;
    bool started_ = false;
    int laps_ = 0;
    double lap_start_ = 0, sector_start_ = 0;
    double last_lap_ = -1, best_lap_ = -1;
    int sector_ = 0;
    std::vector<double> sector_times_, last_sectors_;
    double progress_ = 0;  // unwrapped track distance (0 = start line, lap k ends at k * length)
    float backwards_ = 0; // metres travelled backwards recently
    bool wrong_way_ = false;
};

} // namespace gt3r::track
