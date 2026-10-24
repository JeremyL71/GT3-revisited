#include "race/replay_focus.h"

#include <algorithm>

namespace gt3r::race {

namespace {

// Per rank 0..5 (presentation §G3): the AUTO score a car starts from, its chance of +10 a
// frame, and its battle bonus factor.
constexpr float kRankBase[6] = {400, 300, 200, 100, 100, 100};
constexpr float kRankRate[6] = {1.0f / 60, 0.01f, 1.0f / 150, 0.0056666671f, 0.0046666670f, 1.0f / 300};
constexpr float kRankFactor[6] = {1.5f, 0.95f, 0.85f, 0.75f, 0.7f, 0.65f};
constexpr float kTopScore = 1200;

float dist2(const ReplayFocus::Car& a, const ReplayFocus::Car& b) {
    const render::Vec3 d = a.pos - b.pos;
    return d.x * d.x + d.y * d.y + d.z * d.z;
}

} // namespace

int ReplayFocus::relation(const Car& a, const Car& b) { return dist2(a, b) < 100.0f ? 1 : 0; }

int ReplayFocus::next_car(int from, int dir, int cars) {
    if (cars <= 0) return 0;
    return ((from + (dir < 0 ? -1 : 1)) % cars + cars) % cars;
}

void ReplayFocus::begin() {
    e_ = {};
    for (Entry& e : e_) e.partner = -1;
    hold_.fill(kStartHold);
    rank_.fill(-1);
    prev_rank_.fill(-1);
    rel_ = {};
}

void ReplayFocus::reset_interest(int followed, int target, const std::vector<Car>& cars, u32 seed) {
    rng_.x = seed;
    pick_ = prev_pick_ = followed;
    pending_ = false;
    frames_ = 0;
    score_.fill(0);
    bonus_.fill(0);
    const int n = std::min(int(cars.size()), kMaxCars);
    for (int i = 0; i < n; ++i) {
        const int r = std::clamp(cars[size_t(i)].rank, 0, 5);
        if (target == Top) score_[size_t(i)] = r == 0 ? kTopScore : 0;
        else if (target == Manual) score_[size_t(i)] = i == followed ? kTopScore : 0;
        else score_[size_t(i)] = kRankBase[r];
    }
}

int ReplayFocus::partner(int car) const {
    if (car < 0 || car >= kMaxCars || !e_[size_t(car)].fight) return -1;
    return e_[size_t(car)].partner;
}

int ReplayFocus::battle(int car) const { return car >= 0 && car < kMaxCars ? e_[size_t(car)].state : None; }

bool ReplayFocus::battle_changed(int car) const { return car >= 0 && car < kMaxCars && e_[size_t(car)].changed; }

int ReplayFocus::hold(int car) const { return car >= 0 && car < kMaxCars ? hold_[size_t(car)] : 0; }

void ReplayFocus::set_hold(int car, int frames) {
    if (car >= 0 && car < kMaxCars) hold_[size_t(car)] = frames;
}

float ReplayFocus::score(int car) const { return car >= 0 && car < kMaxCars ? score_[size_t(car)] : 0; }

float ReplayFocus::bonus(int car) const { return car >= 0 && car < kMaxCars ? bonus_[size_t(car)] : 0; }

int ReplayFocus::rank_index(int car) const { return std::clamp(rank_[size_t(car)], 0, 5); }

// The partner search: the best-placed car within 20 m whose relation is at least
// `min_relation`; failing that, among the cars related by 1 or more, the one placed lowest.
int ReplayFocus::search(int car, int min_relation, const std::vector<Car>& cars) const {
    const int n = std::min(int(cars.size()), kMaxCars);
    int best = -1, best_rank = 7;
    for (int j = 0; j < n; ++j) {
        if (rel_[size_t(car)][size_t(j)] < min_relation) continue;
        const int r = rank_[size_t(j)];
        if (r < best_rank && dist2(cars[size_t(car)], cars[size_t(j)]) < 400.0f) best = j, best_rank = r;
    }
    if (best >= 0) return best;
    best_rank = -1;
    int best_rel = 0;
    for (int j = 0; j < n; ++j) {
        const int rel = rel_[size_t(car)][size_t(j)];
        if (rel < 1) continue;
        const int r = rank_[size_t(j)];
        if (!(best_rank < r) || rel < best_rel) continue;
        if (dist2(cars[size_t(car)], cars[size_t(j)]) < 400.0f) best = j, best_rank = r, best_rel = rel;
    }
    return best;
}

void ReplayFocus::step_battles(const std::vector<Car>& cars, bool settled) {
    const int n = std::min(int(cars.size()), kMaxCars);
    for (Entry& e : e_) e.changed = false;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) rel_[size_t(i)][size_t(j)] = i == j ? 0 : relation(cars[size_t(i)], cars[size_t(j)]);
    prev_rank_ = rank_;
    rank_.fill(-1);
    for (int i = 0; i < n; ++i) rank_[size_t(i)] = cars[size_t(i)].rank;

    auto reset = [](Entry& e) { e.fight = false, e.partner = -1, e.state = None, e.timer = 0; };
    auto set = [](Entry& e, int state, int partner) {
        e.state = state, e.partner = partner, e.fight = state != Candidate, e.timer = 0;
    };
    for (int i = 0; i < n; ++i) {
        Entry& e = e_[size_t(i)];
        bool held = false;
        if (hold_[size_t(i)] > 0) --hold_[size_t(i)], held = true;
        const int p = e.partner;
        if (p < 0) { // no partner: after 240 frames the search for a candidate
            if (held) continue;
            if (e.timer < 240) {
                ++e.timer;
                continue;
            }
            if (!settled) continue;
            const int c = search(i, 1, cars);
            if (c >= 0) set(e, Candidate, c);
            continue;
        }
        if (held) continue;
        const int other = search(i, 2, cars);
        int rel = rel_[size_t(i)][size_t(p)];
        if (rel == 0) rel = rel_[size_t(p)][size_t(i)];
        switch (e.state) {
        case Candidate:
            if (!settled) {
                reset(e), e.changed = true;
            } else if (other >= 0 && other != p) {
                set(e, Fight, other), e.changed = true;
            } else if (rel == 1) {
                if (++e.timer >= 121) set(e, After, p), e.changed = true;
            } else if (rel >= 2 && rel <= 3) {
                set(e, Fight, p), e.changed = true;
            } else {
                reset(e), e.changed = true;
            }
            break;
        case Fight:
            if (dist2(cars[size_t(i)], cars[size_t(p)]) > 900.0f) {
                reset(e), e.changed = true;
            } else if (e.timer < 240) {
                ++e.timer;
            } else if (rel != 2 && rel != 3) {
                set(e, After, p);
            }
            break;
        case After:
            if (dist2(cars[size_t(i)], cars[size_t(p)]) > 900.0f) {
                reset(e), e.changed = true;
            } else if (!settled) {
            } else if (other >= 0 && other != p) {
                set(e, Fight, other), e.changed = true;
            } else if (++e.timer >= 241) {
                reset(e), e.changed = true;
            } else if (rel == 2 || rel == 3) {
                set(e, Fight, p);
            }
            break;
        default: reset(e); break;
        }
    }
}

void ReplayFocus::step_scores(const std::vector<Car>& cars, int followed, int target) {
    const int n = std::min(int(cars.size()), kMaxCars);
    ++frames_;
    if (target == Auto) {
        const int rival = partner(followed);
        for (int i = 0; i < n; ++i) {
            if (i == followed || i == rival) {
                score_[size_t(i)] -= 1.0f / 30.0f;
            } else {
                rng_.next();
                if (rng_.rand01() < kRankRate[rank_index(i)]) score_[size_t(i)] += 10.0f;
            }
        }
        for (int i = n; i < kMaxCars; ++i) score_[size_t(i)] = bonus_[size_t(i)] = 0;
        for (int i = 0; i < n; ++i) { // places gained / lost since the last frame
            const int was = prev_rank_[size_t(i)], now = rank_[size_t(i)];
            if (was < 0) continue;
            if (now < was) score_[size_t(i)] += 150.0f;
            else if (was < now) score_[size_t(i)] -= 100.0f;
        }
        for (int i = 0; i < n; ++i) score_[size_t(i)] = std::clamp(score_[size_t(i)], 0.0f, kTopScore);
        for (int i = 0; i < n; ++i) {
            const int st = e_[size_t(i)].state;
            const float b = st == Candidate ? 100.0f : st == Fight ? 400.0f : st == After ? 200.0f : 0.0f;
            bonus_[size_t(i)] = b * kRankFactor[rank_index(i)];
        }
    } else {
        for (int i = 0; i < n; ++i) {
            bonus_[size_t(i)] = 0;
            score_[size_t(i)] = (target == Top ? rank_[size_t(i)] == 0 : i == followed) ? kTopScore : 0.0f;
        }
    }
    if (pending_) return;
    // The re-pick: never in the first 240 frames, then with a chance growing to certainty at 1800.
    float chance = std::max(float(frames_ - 240) / 1560.0f, 0.0f);
    bool look = false;
    if (chance > 1.0f) chance = 1.0f, look = true;
    if (frames_ % 30 == 0) {
        float r = rng_.rand01();
        if (r >= 1.0f) r = 0;
        if (r < chance) look = true;
    }
    if (!look || n <= 0) return;
    int best = std::clamp(pick_, 0, n - 1);
    float top = score_[size_t(best)] + bonus_[size_t(best)];
    for (int i = 0; i < n; ++i)
        if (score_[size_t(i)] + bonus_[size_t(i)] > top) top = score_[size_t(i)] + bonus_[size_t(i)], best = i;
    if (best != pick_) pick_ = best, pending_ = true;
}

void ReplayFocus::step(const std::vector<Car>& cars, int followed, int target, bool battles, bool settled) {
    if (battles) step_battles(cars, settled);
    step_scores(cars, followed, target);
}

int ReplayFocus::take_pending(int cars) {
    const int n = std::min(cars, kMaxCars);
    const int left = std::clamp(prev_pick_, 0, kMaxCars - 1);
    const int left_rank = rank_index(left);
    prev_pick_ = pick_;
    pending_ = false;
    frames_ = 0;
    if (pick_ < 0 || pick_ >= kMaxCars) return pick_;
    score_[size_t(pick_)] += 200.0f;
    const int st = e_[size_t(pick_)].state;
    const float f = st == Candidate ? 0.98f : st == Fight ? 0.84f : st == After ? 0.95f : 1.0f;
    for (int i = 0; i < n; ++i)
        if (i != pick_) score_[size_t(i)] *= f;
    score_[size_t(left)] = kRankBase[left_rank];
    for (int i = 0; i < n; ++i) score_[size_t(i)] = std::clamp(score_[size_t(i)], 0.0f, kTopScore);
    return pick_;
}

} // namespace gt3r::race
