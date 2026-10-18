#pragma once

// GT3's data logger, the Analyzer (menu/<LANG>/logger1.pmb page
// "replay_logger"): two laps, TRACK A (red) and TRACK B (blue), compared sector by sector.
// Reached from the Replay Theater's Analyzer (tracks loaded from replay files on its top
// page, REPLAY or GRAPH) and from the time trial quick menu's DATA LOGGER (A the last lap,
// B the best lap of the session, GRAPH).
//
// Screen (the page's layout; the original fills it in code):
// - the bottom band (bg, indicator_back): per track a status panel (LoggerReplayStatus at
//   TRACK A / TRACK B: SPEED, LAP TIME, SECTOR TIME, SECTOR AVER. SPEED at the cursor) and
//   above it the sector bar (LoggerSector_Sectors: the icon_sector<n> labels, the cursor's
//   sector lit). Sectors are the course's data-logger map points (logger/<course>, LGM0).
// - GRAPH: the traces against the distance round the lap (speed; accel and brake strips) in
//   LoggerRaceGraph_Graph, the course map (the LGM0 picture) with both cars' places at the
//   cursor (graph_point), "L2 JUMP R2"; Circle / Cross opens EXPRESSION (SPEED / ACCEL /
//   BRAKE on/off, DISPLAY A / B / AB) in the middle.
// - REPLAY: the race screen plays track A's lap with track B as a ghost above the band; the
//   control row START / STOP / SLOW / CAMERA.
// Keys (M: matched by playing the original): Left / Right move the graph cursor (or
// the control cursor in REPLAY), L1 / R1 (L2 / R2) jump a sector back / on, Triangle /
// Square leave. The drawing of the traces, map dots and the sector bar is ours.

#include "menu/menu_renderer.h"
#include "menu/menu_scene.h"
#include "race/ghost.h"
#include "race/race_panels.h"
#include "text/game_font.h"

#include <SDL3/SDL_render.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gt3r {
class Vfs;
}

namespace gt3r::race {

// One analysed lap: samples per simulation tick (LapTrace: distance from the line, speed,
// pedals) and the car's states for the replay view.
struct AnalyzerTrack {
    LapTrace trace;
    GhostLap ghost;
    double hz = 60;
    std::string title; // "TRACK A" panel's car name
    bool valid() const { return trace.size() > 1; }
    double time_at(size_t i) const { return double(i) / hz; }
    size_t index_at_distance(float d) const; // first sample at or past d
};

// Re-simulates a replay and keeps its best lap (the whole run when it has no lap).
// `make` returns a fresh RaceVehicle-like object (reset / step / state), `distance_of`
// maps a position to the track distance (0..lap_length).
template <class Make, class DistanceOf>
AnalyzerTrack analyze_replay(const Replay& r, Make&& make, DistanceOf&& distance_of, float lap_length);

// The course's data-logger map (logger/<course>, "LGM0"): the 192 x 128 picture, its
// world -> picture transform (as the HUD map's GTCM) and the sector points.
struct LoggerMap {
    float ox = 0, oz = 0, scale = 1, rot = 0; // rot in radians
    int width = 0, height = 0;
    std::vector<u32> rgba;                 // the picture
    std::vector<std::array<float, 3>> points; // sector points (world)
    static std::optional<LoggerMap> parse(ByteSpan data); // nullopt when not an LGM0
    // Picture coordinates (x right, y down) of a world position.
    std::pair<float, float> project(float x, float z) const;
};

// Sector boundaries (track distances, ascending, first 0) from the map's points.
std::vector<float> logger_sectors(const LoggerMap& m, const std::function<float(float, float, float)>& distance_of,
                                  float lap_length);

class Analyzer {
public:
    enum class View { Graph, Replay };
    enum Control { Start, Stop, Slow, Camera };

    bool load(const Vfs& vfs, SDL_Renderer* r, text::GameFonts& fonts, const std::string& lang = "US");
    // The course (map picture, sectors) and the laps; `b` may be invalid (A only).
    void set_course(SDL_Renderer* r, std::optional<LoggerMap> map, std::vector<float> sectors, float lap_length);
    void set_tracks(AnalyzerTrack a, AnalyzerTrack b);
    void open(View v);
    View view() const { return view_; }
    bool imperial = true;

    // One 60 Hz frame (menu keys, menu::flow::MenuKeys bits). False: the analyzer was left.
    bool tick(u16 held);
    std::vector<std::string> take_sounds();

    // GRAPH cursor (distance round the lap); REPLAY: the race screen moves it with the lap.
    float cursor() const { return cursor_; }
    void set_cursor(float d);
    // REPLAY controls: playing / slow motion / the camera button's presses (the screen cycles
    // its views), and a sector jump the screen performs (seek to that distance).
    bool playing() const { return playing_; }
    bool slow() const { return slow_; }
    int take_camera_presses() { return std::exchange(camera_presses_, 0); }
    std::optional<float> take_seek() { return std::exchange(seek_, std::nullopt); }
    const AnalyzerTrack& track(int i) const { return i == 0 ? a_ : b_; }
    int sector_at(float d) const;

    void draw(SDL_Renderer* r, const SDL_FRect& dst);

private:
    void draw_graph(SDL_Renderer* r, const SDL_FRect& dst);
    void jump(int dir);
    std::optional<menu::MenuScene> scene_;
    std::unique_ptr<menu::MenuRenderer> renderer_;
    text::GameFonts* fonts_ = nullptr;
    SDL_Texture* map_tex_ = nullptr;
    std::optional<LoggerMap> map_;
    std::vector<float> sectors_;
    float lap_length_ = 0;
    AnalyzerTrack a_, b_;
    View view_ = View::Graph;
    float cursor_ = 0;
    bool panel_ = false; // EXPRESSION open
    int panel_row_ = 0;  // 0 speed, 1 accel, 2 brake, 3 display
    bool show_speed_ = true, show_accel_ = true, show_brake_ = true;
    int display_ = 2;    // 0 A, 1 B, 2 AB
    int control_ = Start;
    bool playing_ = true, slow_ = false;
    int camera_presses_ = 0;
    std::optional<float> seek_;
    u16 held_ = 0, ignore_ = 0xFFFF;
    int repeat_ = 0, frames_ = 0;
    std::vector<std::string> sounds_;

public:
    ~Analyzer();
};

// --- implementation of the template ---------------------------------------------------

template <class Make, class DistanceOf>
AnalyzerTrack analyze_replay(const Replay& r, Make&& make, DistanceOf&& distance_of, float lap_length) {
    AnalyzerTrack t;
    t.hz = r.tick_hz;
    auto v = make();
    if (!v) return t;
    if (r.start_speed > 0) v->reset_moving(r.start, r.start_speed);
    else v->place(r.start);
    ReplayPlayer p(&r);
    const double dt = 1.0 / r.tick_hz;
    const bool whole = r.best_lap < 0 || r.best_lap_end <= r.best_lap_begin;
    const u32 begin = whole ? 0 : r.best_lap_begin, end = whole ? r.ticks() : r.best_lap_end;
    float prev = -1, base = 0;
    while (!p.done()) {
        while (const ReplayEvent* e = p.pending_event())
            if (e->kind == ReplayEvent::Respawn) v->reset(e->pose);
        const u32 tick = p.tick();
        sim::ControlInputs in = p.next_inputs();
        if (tick >= begin && tick < end) {
            const sim::VehicleState& s = v->state();
            float d = distance_of(s.position);
            // A lap starts on the line: a first sample just short of it counts from -0.
            if (prev < 0 && !whole && lap_length > 0 && d > lap_length * 0.5f) base = -lap_length;
            // Unwrapped distance from the lap's first sample (the line for a lap).
            if (prev >= 0 && lap_length > 0) {
                if (d < prev - lap_length * 0.5f) base += lap_length;
                else if (d > prev + lap_length * 0.5f) base -= lap_length;
            }
            prev = d;
            float u = d + base;
            if (t.trace.dist.empty() && whole) base -= u, u = 0;
            t.trace.dist.push_back(std::max(0.0f, u));
            t.trace.speed_kmh.push_back(s.speed_kmh);
            t.trace.throttle.push_back(in.throttle);
            t.trace.brake.push_back(in.brake);
            t.trace.x.push_back(s.position.x);
            t.trace.z.push_back(s.position.z);
            t.ghost.add(s);
        }
        v->step(in, dt);
        if (tick + 1 >= end) break;
    }
    t.trace.time = whole ? double(t.trace.size()) * dt : r.best_lap;
    return t;
}

} // namespace gt3r::race
