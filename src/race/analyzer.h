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

// The part of a replay the Analyzer shows: its best lap, or the whole run when it has none
// (ticks [begin, end)).
struct AnalyzerLap {
    u32 begin = 0, end = 0;
    bool whole = true;
};
AnalyzerLap analyzer_lap(const Replay& r);

// Builds a track from a run's states, one per tick of the lap (the state before the tick's
// step and that tick's inputs). Distances (`d`: 0..lap_length on the track) are unwrapped
// from the lap's first sample: a lap counts from the line, a whole run from 0.
class LapSampler {
public:
    LapSampler(double hz, float lap_length, bool whole);
    void add(const sim::VehicleState& s, const sim::ControlInputs& in, float d);
    // `lap_time` < 0: the whole run (its length).
    AnalyzerTrack take(double lap_time);

private:
    AnalyzerTrack t_;
    float lap_length_;
    bool whole_;
    float prev_ = -1, base_ = 0;
};

// The course's data-logger map (logger/<course>, "LGM0"): the 192 x 128 picture, its
// world -> picture transform (the HUD map's GTCM fields, applied differently: see project())
// and the sector points.
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

} // namespace gt3r::race
