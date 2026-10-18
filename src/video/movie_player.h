#pragma once

// Real-time movie playback: decodes a Movie on a worker thread, plays its
// audio on an SDL audio stream owned by the player, and presents the frame
// that is due on the audio clock (wall clock for silent movies) through an
// IYUV streaming texture.
//
//   MoviePlayer player;
//   player.start(open_movie(vfs, "stream/opening.pss"));
//   // every frame:  player.update(renderer);  draw player.texture()

#include "video/field_layout.h"
#include "video/movie.h"

#include <SDL3/SDL_audio.h>
#include <SDL3/SDL_render.h>

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace gt3r::video {

class MoviePlayer {
public:
    MoviePlayer() = default;
    ~MoviePlayer();
    MoviePlayer(const MoviePlayer&) = delete;
    MoviePlayer& operator=(const MoviePlayer&) = delete;

    // Starts playback (stopping any current movie). Frames and audio before
    // `start_seconds` are decoded and discarded.
    void start(std::unique_ptr<Movie> movie, float volume = 0.8f, double start_seconds = 0.0);
    // Plays a movie in a loop (menu backgrounds): `open` is called again each
    // time the movie ends, e.g. [&vfs] { return open_movie(vfs, "stream/menu_bg2.pss"); }.
    void start_loop(std::function<std::unique_ptr<Movie>()> open, float volume = 0.8f);
    void stop();
    bool active() const { return worker_.joinable(); }

    // Main thread, once per rendered frame: advances the clock and uploads
    // the frame that is due. Returns true when the texture changed.
    bool update(SDL_Renderer* renderer);
    SDL_Texture* texture() const { return texture_; }
    // Source and destination rectangles for drawing the current picture into
    // `display`: the whole frame, or for field-split movies (field_layout.h)
    // the half holding the field that is due, positioned with the half-line
    // offset of its field so the picture does not bob up and down.
    void picture_rects(const SDL_FRect& display, SDL_FRect& src, SDL_FRect& dst) const;
    // Draws the current picture into `display` (no-op before the first frame).
    void draw(SDL_Renderer* renderer, const SDL_FRect& display) const;
    // Display rectangle for a 4:3 TV picture fitted into a w x h area.
    SDL_FRect fit_4_3(float x, float y, float w, float h) const;
    bool field_split() const { return field_split_; }

    // Blocks until a decoded frame is queued (or decoding ended / timeout).
    // For non-interactive use such as screenshots.
    bool wait_for_frame(double timeout_seconds);

    void set_paused(bool p);
    bool paused() const { return paused_; }
    void set_volume(float v);
    // All frames presented and the audio drained.
    bool finished() const;

    double position() const { return clock_; } // seconds since the first frame
    const MovieInfo& info() const { return info_; }

    struct Stats {
        u64 decoded = 0, shown = 0, dropped = 0;
        double decode_ms_avg = 0, decode_ms_max = 0; // per frame, worker thread
        double av_drift_ms = 0;  // presented frame time minus clock at upload
        bool audio = false;      // audio clock in use
    };
    Stats stats() const;

private:
    void run();
    double audio_clock() const;

    void begin(std::unique_ptr<Movie> movie, float volume, double start_seconds);

    std::unique_ptr<Movie> movie_;
    std::function<std::unique_ptr<Movie>()> loop_open_;
    MovieInfo info_;
    std::thread worker_;
    std::atomic<bool> quit_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> video_done_{false};
    std::atomic<bool> audio_done_{false};
    std::atomic<bool> field_split_{false};
    FieldLayoutDetector layout_; // worker thread
    double start_seconds_ = 0;

    mutable std::mutex mutex_;
    std::deque<Frame> ready_; // decoded, in display order
    std::vector<Frame> free_;
    Stats stats_;

    SDL_AudioStream* audio_ = nullptr;
    std::atomic<u64> audio_frames_put_{0}; // including frames skipped before start_seconds
    float volume_ = 0.8f;

    SDL_Texture* texture_ = nullptr;
    int tex_w_ = 0, tex_h_ = 0;
    double clock_ = 0;
    double shown_time_ = 0; // presentation time of the frame in the texture
    u64 last_ticks_ = 0;
};

} // namespace gt3r::video
