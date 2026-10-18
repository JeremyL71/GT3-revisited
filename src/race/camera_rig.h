#pragma once

// Race cameras: GT3-style chase camera (lags behind the car's heading),
// bumper and roof cameras (rigid to the body), and a free orbit around the
// car for debugging (right stick / mouse orbit, L2/R2 zoom).

#include "input/virtual_pad.h"
#include "race/replay_cameras.h"
#include "render/math3d.h"
#include "sim/sim_types.h"
#include "sim/vehicle.h"

#include <SDL3/SDL_events.h>

#include <optional>
#include <vector>

namespace gt3r::race {

enum class CameraMode : u8 { Chase, ChaseFar, Bumper, Roof, Orbit, Tv, Count };

// A trackside replay camera: fixed position, covers the car while it is near
// track distance `s` (metres along the lap).
struct TvCamera {
    render::Vec3 position;
    float s = 0;
    float zoom = 1; // > 1 narrows the field of view
};
const char* camera_mode_name(CameraMode m);

class CameraRig {
public:
    void set_mode(CameraMode m);
    CameraMode mode() const { return mode_; }
    // R1 (change view). GT3 has two player views, in-car (bumper, with the rear-view mirror) and
    // chase (its camera point 1; points 1 and 2 are the same in every car). We add the bumper
    // view without the mirror and a far chase: bumper -> bumper (no mirror) -> chase -> far chase.
    // With gt3_views only GT3's two alternate. Other modes (debug orbit, roof) go to the chase.
    void next_mode();
    void set_gt3_views(bool on) { gt3_views_ = on; }
    // GT3's in-car camera vibration (on by default; [race] camera_shake = off disables it).
    void set_shake(bool on) { shake_on_ = on; }
    // The player's view (mode + mirror), e.g. restored from the last race.
    void set_view(CameraMode m, bool mirror) { set_mode(m), mirror_ = mirror; }
    // Whether the onboard view shows the rear-view mirror.
    bool mirror() const { return mirror_ || gt3_views_; }
    // The car's camera points 0 (in-car) and 1 (chase) from its GTCI block: the eye sits at -p in
    // the car model's frame (x right, y up, z back; origin at the wheelbase midpoint on the body's
    // lowest point). Defaults are GT3's fallbacks.
    void set_car_points(const render::Vec3& p0, const render::Vec3& p1) { point_[0] = p0, point_[1] = p1; }
    // L1 held: look back (heading + pi, pitch negated); no mirror while held.
    void set_look_back(bool on) { look_back_ = on; }
    // GT3's pre-race shot (StartingCamera, cameras/start.cam): while `on`, the camera follows the
    // file's record 0 in the car's frame as t goes 0 -> 1 (eye, target and zoom curves).
    void set_intro(bool on, float t) { intro_ = on, intro_t_ = t; }
    // cameras/start.cam's record (without it: a straight move from 30 m ahead to the front corner).
    void set_start_shot(std::optional<GtrcRecord> r) { start_shot_ = std::move(r); }
    bool intro() const { return intro_; }
    bool look_back() const { return look_back_; }

    // Snap the smoothed chase state to the car (after reset/respawn).
    void snap(const sim::VehicleState& s);
    // Advance smoothing and orbit controls; `s` is the interpolated render state.
    void update(const sim::VehicleState& s, const sim::VehicleSetup& setup, const PadState& pad, double dt);
    bool handle_event(const SDL_Event& e);

    render::Vec3 eye() const { return eye_; }
    render::Mat4 view() const { return view_; }
    render::Mat4 projection(float aspect) const;
    float fovy() const { return fovy_; }
    void set_orbit(float yaw, float pitch, float dist) { orbit_yaw_ = yaw, orbit_pitch_ = pitch, orbit_dist_ = dist; }
    // Replay (TV) cameras and the followed car's track distance (set before update()).
    void set_tv_cameras(std::vector<TvCamera> cams, float track_length) { tv_ = std::move(cams), tv_length_ = track_length; }
    void set_track_distance(float s) { track_s_ = s; }
    // The course's own replay camera script (GTRC); used by the TV mode when set.
    void set_replay_director(ReplayDirector* d) { director_ = d; }
    void set_lap(int lap) { lap_ = lap; }
    int replay_record() const { return director_record_; }
    int tv_index() const { return tv_active_; }
    // The eye is in the car: own car hidden, mirror, in-car HUD and sounds. Not during the
    // pre-race shot, which looks at the car from outside (GT3 draws it with the chase HUD).
    bool onboard() const { return (mode_ == CameraMode::Bumper || mode_ == CameraMode::Roof) && !intro_; }

    float near_plane = 0.1f, far_plane = 6000.0f;

private:
    CameraMode mode_ = CameraMode::Bumper; // GT3's default view: in-car (both halves in 2P)
    render::Vec3 eye_{0, 2, 5};
    render::Mat4 view_;
    float fovy_ = 0.8f;
    bool snapped_ = false;
    float chase_yaw_ = 0;    // smoothed heading (radians, same convention as the car: nose = (-sin, 0, -cos))
    float chase_height_ = 0; // smoothed car height
    // GT3 chase lags (spec: physics step at 60 Hz): yaw spring 30 e - 10 w (trapezoid), pitch
    // spring 20 e - 10 w (semi-implicit Euler).
    float lag_yaw_ = 0, lag_yaw_w_ = 0, lag_yaw_acc_ = 0, lag_pitch_ = 0, lag_pitch_w_ = 0;
    double lag_time_ = 0;
    render::Vec3 point_[2] = {{0, -0.8f, 0}, {0, -1.5f, -6.0f}};
    bool look_back_ = false;
    bool mirror_ = true;
    // In-car vibration (GT3's OnBoardCamera shakers): random kicks scaled by the tyres' slip,
    // through a lag, stepped at 60 Hz; see update().
    struct Shaker {
        float v = 0, x = 0, u_prev = 0;
        void step(float u) { v += ((u - u_prev) - 12.0f * v) / 60.0f, x += v / 60.0f, u_prev = u; }
    };
    float shake_random(); // GT3's uniform [0, 1) draw (two RNG steps)
    bool shake_on_ = true;
    Shaker shake_yaw_, shake_pitch_;
    float shake_speed_ = 1.0f / 12.0f;
    u32 shake_rng_ = 0x87654321u;
    double shake_time_ = 0;
    bool gt3_views_ = false;
    bool intro_ = false;
    float intro_t_ = 0;
    std::optional<GtrcRecord> start_shot_;
    float orbit_yaw_ = 0.6f, orbit_pitch_ = 0.25f, orbit_dist_ = 7.0f;
    bool dragging_ = false;
    std::vector<TvCamera> tv_;
    float tv_length_ = 0, track_s_ = 0;
    int tv_active_ = -1;
    ReplayDirector* director_ = nullptr;
    int lap_ = 0, director_record_ = -1;
};

} // namespace gt3r::race
