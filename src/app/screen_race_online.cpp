// Online battle glue (docs/dev/ONLINE.md): the field from the session's grid, network
// events, the synchronised start and tick pacing, remote car prediction, plausibility
// reports and ghosting.
// (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"

namespace gt3r::race_screen {

// Every player simulates only its own car; the others are posed from their snapshots
// (RemoteCar prediction) as kinematic cars in the field. All players start the
// countdown at the same wall-clock instant and pace their ticks from it.

namespace {

constexpr double kOnlineCountdown = 4.0; // s on the grid after the synchronised start
constexpr u32 kGoDelayMs = 1500;          // host: Go -> countdown start
constexpr double kTicksPerMs = sim::kDefaultTickHz / 1000.0;
constexpr double kContactWindow = 12.0 / sim::kDefaultTickHz; // our own contact this recent: theirs is a duplicate
constexpr double kStartGhost = 3.0; // s after the start signal

} // namespace

void RaceScreen::configure_online_field(FieldConfig& cfg) {
    net::Session& ns = net::online();
    cfg.countdown = kOnlineCountdown;
    cfg.opponents.clear();
    cfg.max_opponents = 0;
    cfg.event.clear();
    cfg.wear = {};
    net_.player.clear(), net_.paint.clear(), net_.names.clear();
    const std::vector<u8>& grid = ns.grid();
    for (size_t slot = 0; slot < grid.size(); ++slot) {
        u8 id = grid[slot];
        if (int(id) == ns.my_id()) {
            cfg.player_slot = int(slot);
            continue;
        }
        const net::Player* p = ns.player(id);
        if (!p) continue; // left before the race
        FieldConfig::Remote rm{p->car, int(slot), nullptr};
        // Their car as they race it (garage parts and settings: weight and size count in contact).
        if (const Bytes* info = ns.car_info(id))
            if (std::optional<game::OnlineCar> oc = game::decode_online_car(*info)) {
                rm.car = oc->car;
                std::string err;
                if (std::optional<sim::CarSpec> sp = game::online_car_spec(db_, *oc, &err))
                    rm.spec = std::make_shared<const sim::CarSpec>(std::move(*sp));
                else LOG_WARN("Online: %s's car: %s", p->name.c_str(), err.c_str());
            }
        cfg.remote.push_back(rm);
        net_.player.push_back(id);
        net_.paint.push_back(p->paint);
        net_.names.push_back(p->name);
    }
    net_.cars.assign(net_.player.size(), RemoteCar{});
    net_.wait = ns.joined() ? NetWait::Loading : NetWait::None;
    net_.lost = !ns.joined();
    net_.ticks = 0;
    net_.state_tick = 0;
    net_.input_count = 0;
    net_.ghost.clear();
    net_.checks.clear();
    net_.claims.clear();
    net_.respawn_ticks = 0;
    net_.finish_sent = net_.loaded_sent = false;
}

int RaceScreen::net_index(u8 player) const {
    for (size_t i = 0; i < net_.player.size(); ++i)
        if (net_.player[i] == int(player)) return int(i);
    return -1;
}

void RaceScreen::online_retire(int i) {
    if (!field_ || i < 0 || i >= field_->opponents() || field_->opponent_retired(i)) return;
    field_->set_remote_retired(i);
    if (size_t(i) < opp_visuals_.size()) opp_visuals_[size_t(i)].reset(); // no longer drawn
}

void RaceScreen::online_event(App& app, const net::Event& e) {
    using K = net::Event::Kind;
    switch (e.kind) {
    case K::Go:
        net_.start_at = e.start_at_ms;
        if (net_.wait != NetWait::None) net_.wait = NetWait::Starting;
        break;
    case K::Finish:
        if (int i = net_index(e.player); i >= 0 && field_) {
            field_->set_remote_finish(i, e.time);
            net_.claims.resize(net_.player.size(), -1);
            net_.claims[size_t(i)] = e.time;
            LOG_INFO("Online: %s finished in %.3f s", net_.names[size_t(i)].c_str(), e.time);
        }
        break;
    case K::Contact:
        // Another player's machine saw their car hit ours. If ours saw the contact too (within
        // 0.2 s) it has already been solved here; a ghosted car never touches us.
        if (int i = net_index(e.player); i >= 0 && field_ && !field_->opponent_ghost(i) && !field_->opponent_retired(i)) {
            if (field_->last_contact_with(i) >= field_->race_time() - kContactWindow) break;
            field_->apply_player_impulse(e.contact.point_x, e.contact.point_z, e.contact.jx, e.contact.jz);
            ++net_.contacts_applied;
        }
        break;
    case K::Flag: { // host: a warning about a player
        const net::Session& ns = net::online();
        const net::Player* sp = ns.player(e.player);
        const net::Player* rp = ns.player(e.flag.reporter);
        toast(std::string("Warning: ") + (sp ? sp->name : "?") + " " + net::flag_text(e.flag.kind) + " (seen by " +
              (rp ? rp->name : "?") + ")");
        break;
    }
    case K::Retire:
        if (int i = net_index(e.player); i >= 0) {
            online_retire(i);
            toast(net_.names[size_t(i)] + " left the race");
        }
        break;
    case K::Disconnected:
    case K::Rejected:
        net_.lost = true;
        for (int i = 0; i < int(net_.player.size()); ++i) online_retire(i);
        if (net_.wait != NetWait::None) net_.wait = NetWait::None; // drive on alone
        toast("Connection lost: " + e.text);
        break;
    case K::Lobby: // the host went back to the lobby: the race is over for everyone
        net_.left = true;
        leave(app);
        break;
    default: break; // roster / settings: the lobby reads the session when it resumes
    }
}

// Every frame: network in/out and the start handshake. false = skip the rest of the frame.
bool RaceScreen::online_update(App& app) {
    net::Session& ns = net::online();
    ns.poll();
    while (std::optional<net::Event> e = ns.next_event()) {
        online_event(app, *e);
        if (net_.left) return false;
    }
    if (net_.wait == NetWait::Loading) {
        if (!net_.loaded_sent) ns.loaded(), net_.loaded_sent = true;
        if (ns.is_host() && ns.all_loaded()) {
            ns.go(kGoDelayMs);
            while (std::optional<net::Event> e = ns.next_event()) online_event(app, *e);
        }
        if (net_.wait == NetWait::Loading) toast("Waiting for the other players...");
    }
    if (ns.race_start_ms() > 0) net_.start_at = ns.race_start_ms(); // refined by the clock sync
    if (net_.wait == NetWait::Starting) {
        if (ns.now() >= net_.start_at) net_.wait = NetWait::None, step_.reset();
        else toast("Get ready!");
    }
    online_ghosts(ns);
    if (finished_ && !net_.finish_sent && field_) {
        net_.finish_sent = true;
        double t = race_time_;
        for (const RaceField::Standing& st : field_->standings())
            if (st.car == -1) t = st.time;
        ns.send_finish(t, track_->best_lap_time());
    }
    return true;
}

// Ticks to run this frame: none before the start, then as many as the wall clock since
// the synchronised start asks for (a stalled frame catches up instead of falling behind).
int RaceScreen::online_ticks(int n) {
    if (net_.wait != NetWait::None) {
        step_.reset();
        return 0;
    }
    if (net_.lost || net_.start_at <= 0) return n;
    long long target = (long long)std::floor((net::online().now() - net_.start_at) * kTicksPerMs);
    int k = int(std::clamp<long long>(target - net_.ticks, 0, 30));
    net_.ticks += k;
    return k;
}

void RaceScreen::online_before_step(double dt) {
    if (!field_) return;
    net::Session& ns = net::online();
    double now = field_->race_time();
    for (size_t i = 0; i < net_.player.size() && int(i) < field_->opponents(); ++i) {
        // Physics-based prediction: a shadow of their car on our course.
        if (!net_.cars[i].physics() && net_.physics)
            if (const sim::Vehicle* ov = field_->opponent_vehicle(int(i))) {
                std::unique_ptr<sim::Vehicle> sh = sim::Vehicle::create(ov->spec());
                sh->set_dirt_course(net_.dirt);
                net_.cars[i].attach(std::move(sh), &track_->query(), &track_->world());
            }
        if (const net::Session::Remote* r = ns.remote(u8(net_.player[i]));
            r && (!net_.cars[i].has() || r->snap.tick > net_.cars[i].latest().tick)) {
            net::CarSnapshot snap = r->snap;
            const sim::Vehicle* ov = field_->opponent_vehicle(int(i));
            complete_snapshot(snap, field_->opponent_setup(int(i)), ov ? ov->spec().mass : 1200.0f, kOnlineCountdown);
            net_.cars[i].receive(snap, now);
            // Plausibility (what their car can physically do); the host hears about it.
            if (net_.checks.size() < net_.player.size()) {
                net_.checks.resize(net_.player.size());
                for (size_t k = 0; k < net_.checks.size(); ++k)
                    if (const sim::Vehicle* cv = field_->opponent_vehicle(int(k)))
                        net_.checks[k].set_car(game::geared_top_speed(cv->spec()));
            }
            bool contact = field_->last_contact_with(int(i)) > now - 1.0;
            auto report = [&](std::optional<Plausibility::Flag> f) {
                if (f) ns.report_flag(u8(net_.player[i]), f->kind, f->value);
            };
            report(net_.checks[i].on_state(snap, contact));
            if (net_.cars[i].physics())
                report(net_.checks[i].on_prediction_error(net_.cars[i].last_error(), net_.cars[i].last_error_horizon(), contact));
        }
        // Its finish claim against when we saw it cross the line.
        if (i < net_.claims.size() && net_.claims[i] >= 0 && field_->remote_observed_finish(int(i)) >= 0 &&
            i < net_.checks.size()) {
            if (auto f = net_.checks[i].on_finish(net_.claims[i], field_->remote_observed_finish(int(i))))
                ns.report_flag(u8(net_.player[i]), f->kind, f->value);
            net_.claims[i] = -1;
        }
        if (net_.cars[i].has()) field_->set_remote_pose(int(i), net_.cars[i].predict(now + dt));
    }
}

void RaceScreen::online_after_step() {
    net::Session& ns = net::online();
    const sim::Vehicle* sv = vehicle_->sim_vehicle();
    if (!field_ || !sv || !ns.joined()) return;
    // This tick's input in front of the last ones (each state repeats them: no input is lost
    // with a datagram).
    std::copy_backward(net_.inputs, net_.inputs + net::kInputHistory - 1, net_.inputs + net::kInputHistory);
    net_.inputs[0] = net::pack_input(last_in_, controls_.reverse());
    net_.input_count = std::min(net_.input_count + 1, net::kInputHistory);
    ns.send_state(make_snapshot(*sv, ++net_.state_tick, track_->laps_completed(), net_.inputs, net_.input_count,
                                net_.respawn_ticks > 0));
    if (net_.respawn_ticks > 0) --net_.respawn_ticks;
    // Hits our machine saw: their owners apply them if their machines missed them.
    for (const RaceField::RemoteHit& h : field_->take_remote_hits())
        if (size_t(h.opponent) < net_.player.size()) {
            net::Event::Hit hit{net_.state_tick, h.point_x, h.point_z, h.jx, h.jz};
            ns.send_contact(u8(net_.player[size_t(h.opponent)]), hit);
        }
    ns.poll(); // out at once
}

// Lobby setting: a car whose round trip to us is above the threshold drives as a ghost
// (drawn translucent, no contact): late positions make contact unfair.
void RaceScreen::online_ghosts(const net::Session& ns) {
    if (!field_) return;
    u16 limit = ns.settings().ghost_ping_ms;
    // Race distances (player and opponents) for the lapped-car rule.
    double mine = 0;
    std::vector<double> dist(size_t(field_->opponents()), 0.0);
    for (const RaceField::Standing& st : field_->standings()) {
        if (st.car == -1) mine = st.distance;
        else if (st.car >= 0 && size_t(st.car) < dist.size()) dist[size_t(st.car)] = st.distance;
    }
    const double lap = track_ ? track_->length() : 0;
    net_.ghost.resize(net_.player.size(), false);
    for (size_t i = 0; i < net_.player.size() && int(i) < field_->opponents(); ++i) {
        net::Link l = ns.link(u8(net_.player[i]));
        const char* why = nullptr;
        // Late positions make contact unfair (lobby setting).
        if (limit > 0 && l.known && l.rtt_ms > double(limit)) why = "round trip above the lobby's limit";
        // The start: everyone launches at once and small timing differences would decide the
        // first corner.
        else if (field_->race_time() < kStartGhost) why = "start";
        // A car being lapped (or lapping us): the leader should not be held up by a backmarker,
        // nor the backmarker punted.
        else if (lap > 0 && std::fabs(dist[i] - mine) > lap * 0.9) why = "lapped";
        bool ghost = why != nullptr;
        if (ghost != net_.ghost[i]) {
            net_.ghost[i] = ghost;
            field_->set_remote_ghost(int(i), ghost);
            LOG_INFO("Online: %s %s%s%s (round trip %.0f ms)", net_.names[i].c_str(), ghost ? "ghosted: " : "solid again",
                     ghost ? why : "", "", l.rtt_ms);
        }
    }
}

int RaceScreen::online_finish_position() const {
    std::vector<RaceField::Standing> st = field_->standings();
    for (size_t i = 0; i < st.size(); ++i)
        if (st[i].car == -1) return int(i) + 1;
    return 1;
}

} // namespace gt3r::race_screen
