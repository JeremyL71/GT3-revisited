#pragma once

// Private to the race screen: the RaceScreen class, split over the screen_race*.cpp files
// (one file per concern; the method declarations below are grouped by the file that
// defines them). The race screen code map is in docs/dev/RACE.md; the public entry points
// are in screen_race.h and screen_online.h.

#include "app/app.h"
#include "app/screen_gtmode.h"
#include "app/screen_touch_overlay.h"
#include "app/screen_registry.h"
#include "app/screen_online.h"
#include "app/screen_race.h"
#include "menu/flow/game_hooks.h"
#include "menu/flow/options.h"
#include "menu/flow/menu_sound.h"
#include "audio/engine_sound.h"
#include "audio/motor_sound.h"
#include "base/file.h"
#include "base/log.h"
#include "base/hitch.h"
#include "formats/model/wheel_spec.h"
#include "ai/ai_driver.h"
#include "game/gt_mode_menu.h"
#include "game/gt_save.h"
#include "sim/models.h"
#include "game/data/car_database.h"
#include "race/backend.h"
#include "race/camera_rig.h"
#include "race/controls.h"
#include "race/field.h"
#include "race/car_sfx.h"
#include "race/ghost.h"
#include "audio/course_ambience.h"
#include "formats/sound/crss.h"
#include "race/demo_replay.h"
#include "render/prize_view.h"
#include "race/license.h"
#include "track/track.h"
#include "race/pit.h"
#include "race/billboards.h"
#include "race/lamp_glows.h"
#include "race/race_panels.h"
#include "race/sun_glare.h"
#include "race/results.h"
#include "race/pause_menu.h"
#include "race/paint.h"
#include "race/pool_draw.h"
#include "race/hud.h"
#include "race/interp.h"
#include "race/race_audio.h"
#include "race/race_scene.h"
#include "race/arcade_car.h"
#include "race/point_lights.h"
#include "formats/text/mltx.h"
#include "race/pylons.h"
#include "race/tyre_smoke.h"
#include "race/wet_road.h"
#include "race/road_shade.h"
#include "race/replay.h"
#include "race/replay_options.h"
#include "race/analyzer.h"
#include "race/replay_library.h"
#include "race/net_race.h"
#include "game/net_car.h"
#include "net/session.h"
#include "race/replay_cameras.h"
#include "formats/course/course_file.h"
#include "render/view3d.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gt3r::race_screen {

using namespace race;

// GT3 keeps each player's last change-view choice for the next race (the options object,
// written every frame by the camera manager); a new game starts in the in-car view.
struct LastView {
    race::CameraMode mode = race::CameraMode::Bumper;
    bool mirror = true;
};
extern LastView g_last_view[2];

// The menus' logical keys from a pad (menus spec §2): confirm = Circle or Cross,
// cancel = Triangle or Square, the left stick doubles the D-pad.
u16 menu_keys(const PadState& pad);

struct CourseEntry {
    std::string label, name;
};
struct CarEntry {
    std::string label, name, model;
};

struct DebugPreset {
    std::string course, car;
    int cam = -1, paint = 0;
    bool autodrive = false, manual = false, have = false, hud = true, perf = false, gpusync = false, sheen = true, pause = false;
    int pause_sel = 0;       // pause=1: the cursor on this button (screenshots)
    bool prerace = false;    // open GT mode's pre-race quick menu (screenshots)
    bool gt_result = false;  // gtq=1: the GT-mode result quick menu (screenshots)
    bool watch = false, save = false, ghost = true, music = true;
    double warp = 0, quit = 0, rwarp = 0;
    double lic_finish = 0;   // tests: a running licence test finishes at this elapsed time (forced result)
    bool demo_drive = false; // tests: the player's run driven by the disc's demo inputs (judged as a real run)
    double leave = 0; // tests: back to the menus this many seconds after the finish (posts the result)
    std::string replay; // file to watch ("last" = newest)
    std::string event;  // race against the event's opponents (e.g. AEAT0001)
    bool two_player = false; // 2P battle (split screen, pad 2)
    bool quick = false;      // after warp: open the post-race quick menu (if finished)
    int laps = 0;            // time trial: end after N laps (results), 0 = endless
    std::string split;       // 2P: "h" (top/bottom) or "v" (side by side); empty = config
    std::string car2;        // player 2's car (default: player 1's)
    std::string tt_event;    // time trial with this event's start (flying starts: licence tests, time attack)
    // Scripted player input (tests / screenshots): "thr:1@0;steer:-1@3" sets throttle to 1 at
    // 0 s and steering to full left at 3 s (seconds since the run started; brk, steer, thr).
    std::string script;
    int prize = -1;          // licence result: show this medal (0 gold .. 2 bronze), for screenshots
    bool demo = false;       // start the demonstration (licence: the disc's demo run)
    bool pit = false;        // auto-drive takes the pit lane (and stops) at the next entry marker
    std::string qm;          // quick menu: choose this item when it opens ("option", "record", "data_logger", ...)
    int ropts = -1;          // replays (screenshots): open REPLAY OPTIONS with the cursor on this row
    int rmode = -1;          // replays: start in this view mode, mode indicator shown
    float lcursor = -1;      // the Analyzer (screenshots): its cursor at this distance (m); lpanel=1: EXPRESSION open
    float oyaw = 0.6f, opitch = 0.25f, odist = 7.0f;
    float at = -1, at_kmh = 150; // time trial: start moving at this distance past the line (debug shots)
    float turn = 0;              // debug shots: the standing start turned about +Y by this many degrees
};

// The GT3R_RACE debug preset (have = false when the variable is not set).
DebugPreset read_preset();

// The course file of a course label (any region's folder), or "".
std::string course_path(const Vfs& vfs, const std::string& label);

// Rolling frame statistics, logged every few seconds.
struct Perf {
    double acc_frame = 0, acc_draw = 0, max_frame = 0, window = 0;
    int frames = 0;
    double avg_frame = 0, avg_draw = 0, peak_frame = 0;
    void add(double frame_ms, double draw_ms) {
        acc_frame += frame_ms, acc_draw += draw_ms, max_frame = std::max(max_frame, frame_ms), ++frames;
        window += frame_ms;
    }
    bool roll() {
        if (window < 3000 || frames == 0) return false;
        avg_frame = acc_frame / frames, avg_draw = acc_draw / frames, peak_frame = max_frame;
        acc_frame = acc_draw = max_frame = window = 0;
        frames = 0;
        return true;
    }
};

// Brake-lamp histories (GT3: a 4-frame history of "brake pedal > 0" per car; lamp factor k / 4):
// [0] player, [1] player 2, [2 + i] opponent i.
struct BrakeLamps {
    std::vector<std::array<u8, 4>> hist;
    void push(size_t car, float brake) {
        if (hist.size() <= car) hist.resize(car + 1, std::array<u8, 4>{});
        std::array<u8, 4>& h = hist[car];
        h[0] = h[1], h[1] = h[2], h[2] = h[3], h[3] = brake > 0 ? 1 : 0;
    }
    bool now(size_t car) const { return car < hist.size() && hist[car][3] != 0; }
    float factor(size_t car) const {
        if (car >= hist.size()) return 0;
        const std::array<u8, 4>& h = hist[car];
        return float(h[0] + h[1] + h[2] + h[3]) / 4.0f;
    }
    void clear() { hist.clear(); }
};

// A race started from the interactive menus (menu::flow::GameRequest).
struct MenuLaunch {
    menu::flow::GameRequest request;
};

class RaceScreen : public Screen {
public:
    RaceScreen() : preset_(read_preset()) {}
    explicit RaceScreen(MenuLaunch l);
    // Online battle (docs/dev/ONLINE.md): the session's race; leaving returns to the lobby.
    explicit RaceScreen(const OnlineLaunch& l);
    ~RaceScreen() override;

    void update(App& app, double dt) override;
    void on_event(App& app, const SDL_Event& e) override;
    // Phones: driving touch controls while racing, menu controls otherwise.
    TouchLayout touch_layout() const override;
    void render(App& app, Ui& ui) override;

private:
    // ===== Types =====

    enum class Stage { Select, Drive };
    enum class Panel { None, Options, Settings, Records, Logger }; // quick-menu panels
    struct SoundCar {
        float cd = 0.35f;   // body (racing modify, table 5) record +0x39 / 100
        bool whine = false; // gear part (table 17) +0x10 != 0: non-stock gearbox
    };
    // GT3's replay director (view mode 1) rolls a colour filter at every cut (attract demo):
    // on with p 0.1 (kept with p 0.8), preset curve from {red, blue, cyan, (0.3), (1)}, saturation
    // 1 / random / 0 (0.7 / 0.15 / 0.15); separately (p 0.25) black / white levels, swapped
    // (negative) in 1.25 % of shots. They replace the course grade for the shot.
    struct ShotFilter {
        bool on = false, levels = false;
        float curve[3] = {0.5f, 0.5f, 0.5f}, saturation = 1, black = 0, white = 1;
    };
    struct BestLicense {
        int medal = -1;   // 0 gold .. 2 bronze, -1 finished without a medal
        double time = -1; // judged time (s)
    };
    using FieldLap = RaceField::LapInfo;
    enum class NetWait { None, Loading, Starting }; // online: the start handshake

    // ===== Methods, by the file that defines them =====

    // Screen core (screen_race.cpp)
    void init(App& app);
    void toast(std::string s);
    void log_perf(bool force);

    // Launching (screen_race_launch.cpp)
    void resolve_online();
    void resolve_launch();

    // Run setup (screen_race_setup.cpp)
    TrackPose flying_pose(float metres_before) const;
    void start(App& app, const Replay* replay = nullptr);
    void warp(App& app, double seconds);
    void build_tv_cameras();
    void load_saved_ghost();
    void restart();
    void drop_vehicle();
    bool build_vehicle(App& app);
    bool build_vehicle_inner(App& app);
    void reset_run_state();
    static std::string respawn_mode(App& app);
    std::optional<Bytes> read_car(App& app, const std::string& model) const;
    // Flying start (physics spec §9.1, 9.4: licence tests and time trials with the event's
    // distance and speed): the computer drives the player's car until the first line crossing.
    bool flying_runup() const { return !race_mode_ && tt_start_.flying() && track_ && !track_->lap_started(); }

    // Drive loop (screen_race_drive.cpp)
    sim::ControlInputs scripted_inputs(double t) const;
    void next_view(CameraRig& cam, int player);
    void respawn();
    void apply_respawn(const TrackPose& p);
    sim::ControlInputs autodrive(const sim::VehicleState& s);
    void tick(App& app, const PadState& pad);
    void update_drive(App& app, double dt);
    static ForceFeedbackState feedback(const sim::Vehicle& v, const sim::ControlInputs& in, float wall, float car);
    void update_gear_hint(const sim::VehicleState& s);
    void update_dirt(double dt);
    void update_pylons(double dt);
    void update_smoke(double dt);

#if GT3R_XR
    // VR headsets (screen_race_vr.cpp): while driving, the race in stereo from the head pose,
    // seated in the car (in-car views) or floating behind it (chase views); the HUD, mirror and
    // menus on the panel in front.
    bool vr_stereo(App& app, bool split) const;
    render::Mat4 vr_base(const xr::Settings& st) const; // the seated space's origin -> world
    void render_vr_eyes(App& app, Ui& ui, const render::DrawOptions& o);
#endif

    // Drawing the race (screen_race_render.cpp)
    bool update_env_cube();
    void draw_mirror(App& app, Ui& ui, const SDL_FRect& frame, const render::DrawOptions& main, bool p2 = false,
                     bool half = false);
    CarShadowParams shadow_params() const;
    void draw_car_icon(App& app, int car, const SDL_FRect& rc, size_t slot, bool on_bar = false);
    CarLook car_look(const sim::VehicleState& s, bool player) const;
    static TyreSmoke::View smoke_view(const CameraRig& c, float aspect, float fovy);
    void collect_items(const render::Vec3& eye, const sim::Vec3& right = {1, 0, 0}, bool player = true,
                       bool second = true, const TyreSmoke::View& smoke_v = {});
    size_t player_items_[2] = {0, 0}; // collect_items: the player's car is items_[first, end)
    HudData hud_data(int player);
    float sun_visibility(const render::Vec3& eye, float fovy, float rect_h);
    bool split_horizontal(App& app) const;
    static sim::Vec3 cam_right(const CameraRig& c);
    static sim::Vec3 cam_forward(const CameraRig& c);
    render::DrawOptions draw_options() const;
    void render_drive(App& app, Ui& ui);
    void add_wet_road(const render::Vec3& eye, float aspect, const render::DrawOptions& o) {
        add_wet_road(eye, camera_.view(), camera_.projection(aspect),
                     render::Frustum::symmetric(camera_.fovy(), aspect), o);
    }
    void add_wet_road(const render::Vec3& eye, const render::Mat4& view, const render::Mat4& proj,
                      const render::Frustum& fr, const render::DrawOptions& o);
    std::vector<CarLampGlows::Car> lamp_cars() const;
    static sim::Vec3 to_sim(const render::Vec3& v) { return {v.x, v.y, v.z}; }
    FieldLap field_lap2() const { return field_ && field_->has_second() ? field_->second_laps() : FieldLap{}; }

    // Race audio (screen_race_audio.cpp)
    void music_cues(App& app);
    void update_course_ambience(App& app);
    SoundCar car_sound_facts(const std::string& label) const;
    static CarExtraSounds::Step extra_step(const sim::Vehicle& v, float throttle);
    static CarExtraSounds::Frame extra_frame(const sim::Vehicle& v, const sim::VehicleState& s, const SoundCar& car,
                                             bool onboard, float x, float z, SoundView view);
    void silence(App& app);
    // The original's global SE gain: race SE in live play, replay SE in replays and demos.
    bool se_on() const { return replaying_ || demo_ || attract_ ? replay_se_ : race_se_; }

    // Selection (screen_race_select.cpp)
    void update_select(App& app);
    bool select_entries(const std::string& course, const std::string& car);
    void load_replay_file(App& app, const ReplayFile& f);
    void render_select(App&, Ui& ui);

    // Quick menu (screen_race_quick_menu.cpp)
    void start_free_run(App& app);
    void end_free_run(App& app);
    std::string quick_menu_layout(bool before) const;
    bool menu_before_run() const;
    void fill_license_menu(App& app);
    std::string records_key() const;
    void fill_top3();
    void open_prerace_menu(App& app);
    void open_quick_menu(App& app);
    void update_quick_menu(App& app, double dt);
    std::vector<OptionRow> option_rows() const;
    void change_option(App& app, int row, int dir);
    void update_panel(App& app);
    void open_records(App& app);
    void draw_panel(App& app, const SDL_FRect& dst);
    bool results_panel_up() const { return field_ && finished_ && !replaying_ && !two_player_ && finish_wait_ >= 5.0; }
    bool championship() const { return from_menu_ && launch_.request.champ_round > 0; }

    // Pause (screen_race_pause.cpp)
    std::vector<std::string> pause_items() const;
    PauseMenu::Kind pause_kind() const;
    void open_pause(App& app);
    void update_pause(App& app, double dt);
    void online_pause_note(Ui& ui);
    void render_pause(Ui& ui);

    // Results (screen_race_results.cpp)
    static std::string ascii(const std::string& s);
    std::string car_display_name(const std::string& label, bool full = false) const;
    std::vector<ResultRow> result_rows(bool full_names = false) const;
    std::vector<ResultRow> result_menu_rows() const;
    void latch_result();
    menu::flow::GameResult build_result() const;
    void leave(App& app);
    int presented_medal() const;
    std::string race_text(const char* key, const char* fallback);
    // Tells the menus how the requested race went (GT mode awards prizes from it).
    void post_result() { menu::flow::post_game_result(result_latch_.for_leave(build_result())); }
    // Frames since the result was decided (the presenter's clock).
    int result_frame() const { return int(std::lround(finish_wait_ * 60.0)); }

    // Replays (screen_race_replay.cpp)
    Replay finished_recording() const;
    ReplayKind replay_kind() const;
    void end_replay(App& app);
    void watch_replay();
    void start_playback(const Replay& r);
    void stop_playback();
    void save_current_replay();
    void draw_replay_items(Ui& ui, const SDL_FRect& rect);
    void apply_replay_settings(App& app, const ReplaySettings& rs);
    void update_replay_focus();
    void draw_mode_display(Ui& ui);
    void update_shot_filter();
    void apply_shot_filter(render::DrawOptions& o) const;
    std::string replay_dir(App& app) const { return app.data_dir() + "replays"; }

    // Analyzer (screen_race_analyzer.cpp)
    void load_analyzer(App& app);
    void open_session_analyzer(App& app);
    void setup_logger(App& app);
    void logger_seek(App& app, float d);
    bool update_logger(App& app, double dt);

    // Online (screen_race_online.cpp)
    void configure_online_field(FieldConfig& cfg);
    int net_index(u8 player) const;
    void online_retire(int i);
    void online_event(App& app, const net::Event& e);
    bool online_update(App& app);
    int online_ticks(int n);
    void online_before_step(double dt);
    void online_after_step();
    void online_ghosts(const net::Session& ns);
    int online_finish_position() const;

    // Licence tests (screen_race_license.cpp)
    sim::ControlInputs demo_inputs(size_t i) const;
    void start_demo();
    void start_preview();
    void stop_preview(App& app);
    void end_demo(App& app);
    static std::string license_title(const std::string& code);
    void begin_license();
    void update_license(const sim::VehicleState& s, double dt);
    // The disc demo's steering is +-4096 with positive to the left; ours is positive right.
    static inline float kDemoSteerSign = std::getenv("GT3R_DEMO_SIGN") ? float(std::atof(std::getenv("GT3R_DEMO_SIGN"))) : -1.0f;

    // ===== State =====

    DebugPreset preset_;
    MenuLaunch launch_;
    std::shared_ptr<const sim::CarSpec> launch_spec_;
    std::string launch_model_;
    std::vector<double> lap_times_;
    double finish_wait_ = 0, all_done_at_ = -1;
    bool finish_menu_shown_ = false;
    bool p1_won_ = false, p2_won_ = false; // 2P: who crossed first
    int finish_record_rank_ = -1;
    double saved_best_ = -1; // best lap among saved replays (time trial records)
    bool quick_ = false;
    bool results_loaded_ = false;
    Panel panel_ = Panel::None;
    int panel_sel_ = 0;
    LapLogger logger_;
    bool imperial_ = true; // speeds in mph
    std::shared_ptr<ai::AiCourse> hint_course_;   // recommended-gear hint (passive AI bookkeeping)
    std::unique_ptr<ai::AiDriver> hint_driver_;
    std::shared_ptr<ai::AiCourse> runup_course_;  // rolling / flying run-up: the CPU AI driver
    std::unique_ptr<ai::AiDriver> runup_driver_;
    int hint_gear_ = -1;
    bool hint_blink_ = false;
    double car_odometer_m_ = 0, car_oil_m_ = 0; // garage car condition at the start
    double car_dirt_ = 0;
    int launch_tire_stage_ = -1;
    std::shared_ptr<const sim::CarSpec> launch_spec2_; // 2P: player 2's arcade car (own parts, tyres)
    int car2_paint_ = -1, manual2_ = -1;               // 2P: player 2's paint and gearbox (-1 = default) // arcade car: tyre stage used (10 racing, 11 drift, ...); -1 = none
    bool dirt_dims_ = true;               // false on the wet Route 5 (GT3 uses 0 there)
    std::string course_label_for_dirt_;   // course label (course table category)
    CoursePointLights point_lights_;
    BrakeLamps brake_lamps_;
    std::string car_folder_ = "cars/day/"; // the course's car time of day (day / eve / night)
    CarLampGlows car_lamps_;
    WetRoad wet_;                                // the wet-road reflection pass (course slots 9 / 13)
    std::vector<render::DrawItem> wet_items_;    // its buffer's scene
    Pylons pylons_;
    bool pylon_hit_ = false; // the player's car touched a pylon (licence: FAIL)
    TyreSmoke smoke_;                             // tyre smoke / dirt dust (course slots 26 / 32)
    // Per-car light sets this frame (stable addresses; filled while collecting the draw items).
    mutable std::deque<render::ItemLights> item_lights_;
    bool env_cube_on_ = true; // [video] car_reflections
    bool shadows_on_ = true;  // [video] car_shadows
    std::vector<render::DrawItem> env_items_;
    bool env_gt3_scene_ = false; // the map holds GT3's reflection scene: use GT3's lookup
    std::vector<RecordEntry> records_;
    std::vector<std::string> record_names_;
    RecordsPanel records_panel_;
    bool records_loaded_ = false;
    bool demo_ = false;                  // quick menu "demonstration": the computer drives
    std::optional<Replay> pre_demo_run_; // the player's run, kept for replay / save
    bool replay_from_quick_ = false;
    bool post_replay_ = false, post_replay_done_ = false; // 2P: the dimmed post-race replay     // replay started from the quick menu: back to it at the end
    bool abandoned_ = false;
    bool attract_ = false, attract_interrupted_ = false; // title-screen attract demo
    bool theater_ = false;     // started by the Replay Theater (theater_replay / theater_demo / logger)
    // The Analyzer (data logger): the theater's (logger_mode_) or the quick menu's.
    Analyzer analyzer_;
    bool analyzer_loaded_ = false, logger_mode_ = false;
    u32 logger_begin_ = 0, logger_end_ = 0; // track A's lap in its replay (ticks)
    double logger_acc_ = 0;
    // REPLAY OPTIONS and the replay view mode (presentation.md §G).
    ReplaySettings replay_settings_;
    ReplayOptions replay_opts_;
    bool replay_opts_open_ = false, replay_opts_loaded_ = false;
    double replay_opts_acc_ = 0;
    static constexpr double kModeDisplaySeconds = 2.0; // mode indicator: alpha min(1, 4 t^2) (M: start)
    double mode_display_t_ = 0;
    int replay_focus_ = -1;     // followed car in field replays: -1 the player, else an opponent
    int replay_focus_shot_ = -2; // AUTO: the shot the focus was chosen for
    std::string theater_demo_; // theater_demo: the demonstration's event code
    static constexpr double kAttractSeconds = 1800.0 / 60.0; // RaceArcadeDemo's 1800-step timer
    float demo_start_kmh_ = 0;    // demo events: every car starts at this speed
    bool setting_pending_ = false;
    bool respawn_all_ = false;           // [race] respawn = all: the player's car too
    bool tt_failed_ = false;
    SunGlare sun_;
    bool no_course_fog_ = SDL_getenv("GT3R_NO_COURSE_FOG") != nullptr; // compare with the old linear fog
    LampGlows lamps_;
    int sun_log_ = 0;
    Billboards billboards_;
    bool original_res_ = false, original_nearest_ = true;
    double script_time_ = 0;
    double leave_clock_ = 0; // preset leave: seconds since the finish
    std::optional<BestLicense> best_license_; // the best finished licence run of this visit
    int demo_drive_tick_ = 0; // preset demo_drive: ticks since the run started
    bool preview_ = false;               // licence menu: the demonstration in the preview box
    bool run_preview_ = false;           // ... or the player's own run replaying (after a test)
    int exit_transition_ = -1;           // frames into the race -> menu transition (-1: none)
    static constexpr int kExitHold = 10, kExitWash = 8;
    bool prerace_ = false;
    bool free_run_ = false;              // free run before a GT race (back to the pre-race menu)
    int free_run_laps_ = 0;               // the pre-race quick menu (before the race starts)
    bool music_on_ = true, rumble_on_ = true, rumble2_on_ = true;
    bool aids_options_apply_ = false, asm_on_ = true, tcs_on_ = true;
    bool race_se_ = true, replay_se_ = true, ghost_on_ = true, wide_mode_ = false;
    QuickMenu quick_menu_;
    RaceResultLatch result_latch_;
    std::optional<std::vector<ResultRow>> latched_rows_; // the result menu's rows, kept with the latched result
    PauseMenu pause_menu_;
    bool pause_loaded_ = false;
    menu::flow::MenuSounds quick_sounds_; // the menus' sound effects (quick / pause menus)
    bool quick_sounds_tried_ = false;
    ResultsOverlay results_;
    TimeTrialOverlay tt_overlay_;
    LicenseResultOverlay license_overlay_;
    std::unique_ptr<render::PrizeView> prize_; // licence medal model
    int shown_medal_ = -1;
    bool run_recorded_ = false;     // this licence run went into the records (name entry)
    bool gt_after_race_done_ = false, prize_shown_ = false, auto_replay_pending_ = false; // GT race: prize page, replay
    bool any_run_recorded_ = false; // GameResult::records_done
    std::optional<text::StringTable> race_strings_;
    bool race_strings_tried_ = false;
    double odometer_m_ = 0;
    bool from_menu_ = false;
    bool scripted_ = false; // a menu launch with GT3R_RACE set (no pre-run quick menu unless qm=)
    App* app_ = nullptr;
    bool debug_ = SDL_getenv("GT3R_RACE_DEBUG") != nullptr;
    int debug_ticks_ = 0;
    bool show_perf_ = false;
    u16 race_prev_ = 0; // race pad buttons of the last frame (edges)
    bool init_ = false, db_ok_ = false, hud_loaded_ = false, paused_ = false;
    Stage stage_ = Stage::Select;
    std::string error_, course_name_;
    data::CarDatabase db_;
    std::vector<CourseEntry> courses_;
    std::vector<CarEntry> cars_;
    int course_sel_ = 0, car_sel_ = 0, column_ = 0, pause_sel_ = 0;
    bool pause_more_ = false;
    int scripted_kind_ = 0;
    ShotFilter shot_filter_;
    int last_shot_ = -2;
    PaintRng filter_rng_; // the replay director's per-shot rolls
    u32 paint_seed_ = GameRandom::kBootSeed;   // race_info_random() before this race's paint draw
    std::optional<u32> paint_seed_override_;     // the next build re-runs a replay's paint draw
    SDL_FRect frame_rect_{0, 0, 0, 0}; // the 3D frame's rectangle last drawn (UI units)

    CarSetup car_;
    std::unique_ptr<RaceTrack> track_;
    std::unique_ptr<RaceVehicle> own_vehicle_;
    RaceVehicle* vehicle_ = nullptr; // own_vehicle_ or the field's player
    std::unique_ptr<RaceField> field_;
    std::vector<std::shared_ptr<CarVisual>> opp_visuals_;
    bool race_mode_ = false;
    // --- online battle (docs/dev/ONLINE.md) ---
    bool online_ = false;
    // The online race's state (screen_race_online.cpp; only read in online races).
    struct Online {
        NetWait wait = NetWait::None;
        double start_at = 0;         // net::now_ms() of the countdown start
        long long ticks = 0;         // ticks run since then
        std::vector<int> player;     // opponent index -> player id
        std::vector<int> paint;      // opponent index -> paint
        std::vector<std::string> names;
        std::vector<RemoteCar> cars;
        bool finish_sent = false, lost = false, loaded_sent = false, left = false;
        u32 state_tick = 0;
        Bytes car_info; // our game::OnlineCar
        bool physics = SDL_getenv("GT3R_NET_PREDICT") == nullptr || SDL_getenv("GT3R_NET_PREDICT")[0] != 'd'; // "dead" = dead reckoning
        bool dirt = false;
        int contacts_applied = 0; // hits only the other machine saw
        std::vector<bool> ghost;   // remote cars currently ghosted (any rule)
        std::vector<Plausibility> checks;
        std::vector<double> claims; // finish claims not yet checked against what we saw
        int respawn_ticks = 0;      // our states carry "respawned" this many more ticks
        net::InputFrame inputs[net::kInputHistory];
        int input_count = 0;
    } net_;
    // 2P battle (split screen): player 2 drives the field's second car with pad 2.
    bool two_player_ = false;
    OpponentEngines opp_engines_;
    RaceSfx sfx_;
    CarSfx car_sfx_{"p1_"}, car_sfx2_{"p2_"};
    float ffb_wall_ = 0, ffb_car_ = 0, ffb_wall2_ = 0, ffb_car2_ = 0; // strongest hits since the last frame
    CarExtraSounds extra_{"p1_"}, extra2_{"p2_"};
    SoundCar sound_car_, sound_car2_;
    CourseAmbience crss_;
    CourseAmbiencePlayer crss_player_;
    bool crss_ok_ = false;
    int last_count_ = -1;
    CarColors colors_;
    ControlMapper controls2_;
    CameraRig camera2_;
    RumbleModel rumble2_;
    EngineSound engine2_;
    sim::ControlInputs last_in2_;
    sim::VehicleState render_state2_;
    std::unique_ptr<render::View3D> view2_;
    std::unique_ptr<render::View3D> mirror_view_, mirror_view2_;
#if GT3R_XR
    std::unique_ptr<render::View3D> vr_views_[2]; // the eyes (screen_race_vr.cpp)
#endif
    std::vector<std::unique_ptr<render::View3D>> icon_views_; // 2P results panel car pictures
    std::unique_ptr<CarVisual> car2_visual_;
    bool finished2_ = false;
    int finish_position2_ = 0;
    u16 pad2_prev_ = 0;
    std::string race_event_;
    std::vector<std::string> race_opponents_;
    int race_laps_ = 0;
    bool rolling_ = false;   // rolling start (GT endurance)
    PitLane pit_;
    double pit_msg_time_ = 0;     // "%s Pitting in..." shown for 3 s
    std::string pit_msg_text_;
    std::string license_event_;   // licence test event ("" = not a licence test)
    u32 event_paint_id_ = 0xFFFFFFFFu; // licence test: the event's slot-0 colour id (0xFFFFFFFF = none)
    double machine_mark_m_ = 0;   // machine test distance (400 / 1000 m), 0 = none
    double machine_dist_ = 0, machine_time_ = -1;
    float top_speed_kmh_ = 0;     // best speed of the run (GameResult::top_speed_kmh)
    LicenseRun license_, demo_judge_;
    double license_dist_ = 0;          // the judge's previous unwrapped track distance
    bool license_dist_valid_ = false;
    std::optional<DemoReplay> disc_demo_; // demo/<event>: the game's demonstration run
    bool disc_demo_tried_ = false;
    size_t demo_tick_ = 0;
    bool demo_off_ = false; // the demonstration's car put two wheels off the road: the AI drives on
    LicenseTimes license_times_;
    float license_progress_ = 0;  // metres since the start line at the last tick
    double tt_hold_ = 0;     // seconds of standing-start hold left (time trial / licence)
    EventStart tt_start_;    // time trial / licence event start (flying start when initial speed > 0)
    double time_limit_ = 0;  // time-limited race (s), 0 = laps
    bool finished_ = false;
    int finish_position_ = 0;
    std::unique_ptr<CourseScene> course_scene_;
    std::unique_ptr<CarVisual> car_visual_;
    std::unique_ptr<render::View3D> view_;
    std::vector<render::DrawItem> items_;

    FixedStep step_{sim::kDefaultTickHz};
    sim::VehicleState prev_, render_state_;
    sim::ControlInputs last_in_;
    ControlMapper controls_;
    RumbleModel rumble_;
    CameraRig camera_;
    Hud hud_;
    EngineSound engine_;
    MotorBanks motor_; // synthesised banks when the player's car is an electric mod car
    ReplayRecorder recorder_;
    Replay replay_;
    ReplayPlayer player_;
    bool replaying_ = false;
    CameraMode drive_camera_ = CameraMode::Bumper; // GT3 starts every race in the in-car view
    GhostLap ghost_, cur_lap_;
    double ghost_time_ = -1;
    u32 lap_ticks_ = 0, lap_begin_tick_ = 0;
    bool was_started_ = false;
    std::unique_ptr<GtrcScript> gtrc_;
    ReflectionDiscMesh reflection_discs_; // point-light courses: the lamps' discs in the reflection map
    ReplayDirector director_;
    ReverbAmbience ambience_;
    RaceMusic music_;
    int music_start_type_ = 2;
    double music_license_end_s_ = -1;   // the live licence run's end (race time), for its replay's finish cue
    bool music_license_failed_ = false; // the field event's start type (start jingle; music spec §2)
    std::string toast_;
    double toast_time_ = 0;
    bool show_replays_ = false;
    std::vector<ReplayFile> replay_files_;
    int replay_sel_ = 0;
    int auto_shift_ = 0;
    double stuck_time_ = 0, best_progress_ = -1;
    double race_time_ = 0, offroad_time_ = 0, clock_ = 0, frame_ms_ = 0;
    float lowest_ = -1000;
    Uint64 last_counter_ = 0;
    Perf perf_;
};

} // namespace gt3r::race_screen
