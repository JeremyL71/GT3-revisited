#include "race/music_markers.h"

namespace gt3r::race {

void MusicMarkers::start(const sound::MadsMarkers* song) {
    song_.reset();
    if (song && song->any() && song->rate > 0) song_ = *song;
    cursor_.fill(0);
    interval_.fill(kNone);
    event_.fill(false);
    restart_ = true;
}

void MusicMarkers::step(double position) {
    const float now = float(position) + kLead;
    const bool restart = std::exchange(restart_, false);
    for (int i = 0; i < kTracks; ++i) {
        interval_[size_t(i)] = kNone;
        event_[size_t(i)] = false;
        if (!song_) continue;
        const std::vector<u32>& m = song_->tracks[size_t(i)];
        if (m.empty()) continue;
        const float rate = float(song_->rate);
        size_t& c = cursor_[size_t(i)];
        float t = -FLT_MAX;
        if (c < m.size()) {
            t = float(m[c]) / rate;
            if (t < now) { // passed this frame
                event_[size_t(i)] = true;
                if (++c == m.size()) t = -FLT_MAX;
            }
        }
        if (t < 0) t = float(m[0]) / rate + float(song_->length) / rate; // after the last: as if looped
        if (restart) {
            c = 0;
            event_[size_t(i)] = true;
            t = float(m[0]) / rate;
        }
        interval_[size_t(i)] = t - now;
    }
}

} // namespace gt3r::race
