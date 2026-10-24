#pragma once

// The race music's markers (music §8): the race music object follows the playing song's
// ads.inf marker tracks (sound::MadsMarkers) every 60 Hz frame, before the cameras. For
// each track it keeps the event "a marker passed this frame" and the interval "seconds to
// the next marker", which the replay cameras turn into cut requests and shot lengths
// (presentation §G14, race::CutClock).
//
// Per frame, with now = the song's audible position + 1/30 s (two frames ahead), for every
// track with markers: the next marker t = marker[cursor] / rate; t < now: event, cursor + 1
// (one marker per track per frame at most); past the last marker the next one is the first
// plus the song's length (as if it looped); interval = t - now (slightly negative on the
// frame a marker fires). A new piece (or the same one restarted) fires every marked track
// once on its first frame, cursors back to 0 and t = the first marker. Without a marked
// song (jingles, licence runs, an empty playlist, a stop) every interval is "none" (FLT_MAX)
// and no event fires.

#include "formats/sound/mads.h"

#include <array>
#include <cfloat>
#include <optional>
#include <utility>

namespace gt3r::race {

class MusicMarkers {
public:
    static constexpr int kTracks = sound::MadsMarkers::kTracks;
    static constexpr float kNone = FLT_MAX; // an interval without a next marker
    static constexpr float kLead = 1.0f / 30.0f;

    // A new piece starts (or the music stops): `song` = its markers, nullptr for a piece
    // without them. The next step() fires every marked track once.
    void start(const sound::MadsMarkers* song);
    // One 60 Hz frame at the song's audible position (seconds).
    void step(double position);

    float interval(int track) const { return track >= 0 && track < kTracks ? interval_[size_t(track)] : kNone; }
    bool event(int track) const { return track >= 0 && track < kTracks && event_[size_t(track)]; }
    bool marked() const { return song_.has_value(); }

private:
    static std::array<float, kTracks> filled(float v) {
        std::array<float, kTracks> a;
        a.fill(v);
        return a;
    }
    std::optional<sound::MadsMarkers> song_;
    std::array<size_t, kTracks> cursor_{};
    std::array<float, kTracks> interval_ = filled(kNone);
    std::array<bool, kTracks> event_{};
    bool restart_ = false;
};

} // namespace gt3r::race
