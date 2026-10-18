#pragma once

// In-race HUD, reproducing GT3's layout (facts from the race workstream's
// HUD research, summarised in docs/dev/RACE.md "HUD"):
//
// - The game lays the HUD out on a 640x448 frame; elements aligned with bit 4
//   ("left/right + wide") are pushed outward by 106.67 units in 16:9 mode.
//   We draw that 16:9 layout onto the 640x360 UI: x' = (x + 106.67) * 0.75,
//   y' = y * 360 / 448.
// - Meter cluster per view: remote (chase/TV cameras) = revb_NN tachometer at
//   the bottom right with digital speed and gear inside; onboard (bumper/roof)
//   = rev_NN tachometer + speed_XXX speedometer at the bottom centre.
// - Round meters: face stretched to its box; pivot = box centre + offset;
//   angle = a0 + 90 + t (a1 - a0) degrees (y down, clockwise); tapered red
//   needle from n0 r to n1 r; a thin segmented band at 0.95..0.98 r, red from
//   the shift-up rpm.
// - Tach scale = floor((rev limit + 1500) / 1000) * 1000; face = first of
//   06..12, 18 that covers it.
// - Gear: font "gearr", unlit "8" behind the digit, amber (R in red), plus a
//   blinking shift light (3 frames on / 3 off) above the shift-up rpm.
// - Digital speed: font "speed", amber, "%3d", unit label in sw_12.
// - Mini-map: the course's GTCM image and transform (see set_course_map),
//   player dot 1pcar_dot on top of ecar_dot opponents.

#include "text/game_font.h"

#include <SDL3/SDL_render.h>

#include <map>
#include <array>
#include <string>
#include <vector>

namespace gt3r {
class Vfs;
}

namespace gt3r::race {

struct HudData {
    float speed_kmh = 0;
    float rpm = 0;
    float rev_limit_rpm = 7000; // tach scale and red band
    float shift_rpm = 6800;     // red band start, shift light
    float top_speed_kmh = 250;  // estimate; selects the speedometer scale
    bool imperial = false;      // mph (NTSC-U): speed_120mp/160mp/200mp faces, "mph" label
    bool has_turbo = false;     // boost meter shown
    float boost = 0;            // kgf/cm^2 (needle 0..2)
    int gear = 1;          // -1 R, 0 N
    bool automatic = true;
    bool rev_limiter = false;
    bool onboard = false;  // onboard cluster (bumper/roof cameras) instead of the remote one
    float throttle = 0, brake = 0;
    int lap = 0;           // current lap number (1-based; 0 = not started)
    double lap_time = 0, last_lap = -1, best_lap = -1;
    bool wrong_way = false;
    int total_laps = 0;     // race: laps to run (0 = time trial)
    int position = 0, cars = 0; // race: current position of `cars`
    double countdown = -1;  // race: seconds to the start (> 0), 0 = "GO", < 0 = none
    std::string car_name, course_name;
    std::string camera;
    // Mini-map: world x/z of the player and of the other cars.
    float player_x = 0, player_z = 0;
    std::vector<std::pair<float, float>> others;
    bool show_map = true;
    bool show_meters = true; // meter cluster (replays: REPLAY OPTIONS ITEM "easy meter")
    bool show_times = true;  // lap / time block (replays: ITEM "time")
    bool show_rank = true;   // race position (left)
    bool split = false;      // 2P half (racemode 7 layout)
    double finish_t = -1;    // seconds since this player finished (FINISH / PLAYER n WIN! for 4 s)
    int winner_player = 0;   // 2P: 1/2 = this half's player won
    // Part of the 640x360 UI area no touch control covers (App::touch().safe_area());
    // the right-hand and top blocks move inside it. Full area = the normal layout.
    SDL_FRect safe_area{0, 0, 640, 360};
    // Time-limited race: seconds left (shown instead of the lap count); < 0 = lap race.
    double time_left = -1;
    // Licence test: gold / silver / bronze target times (s) shown under the clock; no lap count.
    std::array<double, 3> license_targets{-1, -1, -1};
    // Tyre gauge (races with tyre wear): per-wheel level -63..127 (FL, FR, RL, RR).
    bool show_tires = false;
    std::array<int, 4> tire_levels{};
    int layer = 0;            // 0 all, 1 all but the lap / time block, 2 that block only
    bool four_three = false;  // drawn into a 4:3 viewport (original resolution): no 16:9 widening
    bool auto_drive = false;  // "AUTO DRIVE" (the computer drives: pit lane, flying start run-up)
    // Odometer cluster: the car's total distance (km or miles, by `imperial`) and this race's
    // trip distance (m); indicator lamps: 0 off, 1 on, 2 acting (blinks).
    double odometer_km = 0, trip_m = 0;
    int asm_lamp = 0, tcs_lamp = 0;
    bool handbrake_lamp = false, oil_lamp = false;
    bool clutch_engaged = true; // gear digit at half brightness when false
    int hint_gear = -1;         // recommended gear ("> n" in red after the digit), -1 = none
    bool hint_blink = false;    // blinks red / black in braking zones
    std::string intro_title;  // pre-race shot: the course name in capitals over the scene
    std::string event_text;   // race information pop-up, e.g. "Skyline Pitting in..."
    float event_alpha = 0;
};

class Hud {
public:
    Hud();
    ~Hud();
    Hud(const Hud&) = delete;
    Hud& operator=(const Hud&) = delete;

    // Loads fonts and the display images; missing assets fall back to placeholders.
    void load(const Vfs& vfs, SDL_Renderer* r);
    // Course map (course file slot 33, "GTCM"); empty span clears it.
    void set_course_map(SDL_Renderer* r, ByteSpan gtcm);
    // 2P battle half (digital speed, bar tachometer "meter_mini", mini-map at 70 %, no round
    // meters). 16:9: racemode 8 (426.667x448 per half); horizontal: racemode 7 (640x224).
    // `horizontal`: GT3's 4:3 top/bottom split layout (racemode 7, 640x224 per half).
    void draw_split(SDL_Renderer* r, const HudData& d, const SDL_FRect& half, bool horizontal = false);
    // Replays: no HUD, the "GRAN TURISMO 3" mark (gt_logo_mini) at the bottom right.
    void draw_replay_mark(SDL_Renderer* r, const SDL_FRect* viewport = nullptr, bool four_three = false);
    // HUD images (display.imgs names) side by side from (x, y_top) of the 640 x 448 frame,
    // 1 unit apart, at `alpha` (the replay view mode indicator).
    void draw_images_row(SDL_Renderer* r, const std::vector<std::string>& names, float x, float y_top, float alpha,
                         const SDL_FRect* viewport = nullptr);
    // GT3's race pause: CONTINUE / EXIT buttons over the running scene (`selected` 0 / 1).
    void draw_pause(SDL_Renderer* r, int selected, const SDL_FRect* viewport = nullptr, bool four_three = false);
    // GT3's replay pause: CONTINUE / EXIT / REPLAY OPTIONS on a dark strip (`selected` 0..2).
    void draw_replay_pause(SDL_Renderer* r, int selected, const SDL_FRect* viewport = nullptr, bool four_three = false);
    // `viewport` (UI coordinates): draw into that rectangle instead of the whole screen (split screen).
    void draw(SDL_Renderer* r, const HudData& d, const SDL_FRect* viewport = nullptr);
    text::GameFonts& fonts() { return fonts_; }

    // "1'23.456" (or "-'--.---" for negative times).
    static std::string format_time(double seconds);
    // Tachometer scale (rpm) for a rev limit, as the game picks it.
    static int tach_scale(float rev_limit_rpm);

private:
public:
    static SDL_Color tire_color(int level);

private:
    struct Image {
        SDL_Texture* tex = nullptr;
        float w = 0, h = 0;
    };
    struct Meter {
        const Image* face = nullptr;
        float x, y, w, h;   // box in 640x448 units (already aligned)
        float ox, oy, r;    // pivot offset and radius in face pixels
        float a0, a1;       // degrees
        float n0, n1;       // needle radius fractions
        float scale;        // full scale value
        float red_from;     // band turns red here (0 = no red)
        float step;         // band segment size (0 = no band)
    };
    void draw_meter(SDL_Renderer* r, const Meter& m, float value);
    // `baseline`: y is the glyph baseline (the game's pen y for the sw_*, odo and speed fonts);
    // otherwise the top of the line. `spacing`: extra pixels after each glyph.
    void text(SDL_Renderer* r, float x, float y, const std::string& s, const char* font, float scale, SDL_Color c,
              text::HAlign a, bool baseline = false, float spacing = 0);
    float text_width(const std::string& s, const char* font, float scale = 1, float spacing = 0);
    // Lamp row / glyph runs in individual colours (odometer font glyphs), pen = left baseline.
    void glyph_run(SDL_Renderer* r, float x, float y, const std::vector<std::pair<std::string, SDL_Color>>& run,
                   const char* font);
    void polygon_strip(SDL_Renderer* r, const std::vector<SDL_FPoint>& pts, SDL_Color c); // HUD units
    const Image* image(const std::string& name) const;

    text::GameFonts fonts_;
    std::map<std::string, Image> images_; // display.imgs entries by name
    bool fonts_ok_ = false;
    // Course map.
    Image map_;
    // tire_gage split by its alpha codes: the four tyre bodies and the outlines.
    std::array<Image, 4> tire_body_{};
    Image tire_outline_;
    float map_ox_ = 0, map_oz_ = 0, map_scale_ = 1, map_rot_ = 0;
    int blink_ = 0, shift_counter_ = 0, lamp_counter_ = 0;
};

} // namespace gt3r::race
