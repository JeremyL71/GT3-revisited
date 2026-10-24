#include "race/field.h"

#include "ai/ai_driver.h"
#include "ai/ai_rules.h"
#include "ai/safety_respawn.h"
#include "base/bytes.h"
#include "base/log.h"
#include "game/data/car_database.h"
#include "sim/car_spec.h"
#include "sim/track_walls.h"
#include "sim/world.h"
#include "race/interp.h"
#include "race/pit.h"
#include "track/track.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace gt3r::race {

namespace {

std::string base_car_of(const data::CarDatabase& db, const std::string& label) {
    if (db.car(label)) return label;
    if (const data::CarVariant* v = db.variant(label)) return v->car;
    for (const data::CarVariant& v : db.arcade_cars())
        if (v.label == label) return v.car;
    return {};
}

class Field;

class PlayerVehicle : public RaceVehicle {
public:
    explicit PlayerVehicle(Field& f) : f_(f) {}
    const char* backend_name() const override { return "sim::World (field)"; }
    void reset(const TrackPose& p) override;
    void place(const TrackPose& p) override;
    void set_dirt_course(bool dirt) override;
    void reset_moving(const TrackPose& p, float speed_mps) override;
    void step(const sim::ControlInputs& in, double dt) override;
    const sim::VehicleState& state() const override;
    const sim::VehicleSetup& setup() const override;
    float last_impact() const override { return impact_; }
    float boost() const override;
    float wall_impulse() const override;
    const sim::Vehicle* sim_vehicle() const override;
    void begin_pit_stop(const sim::Quat& box) override;
    void set_engine_condition(const sim::EngineCondition& c) override;
    sim::EngineCondition engine_condition() const override;
    void set_reverse_request(bool on) override;
    void set_driver_kind(int kind) override;
    void set_scripted_aids(const sim::ScriptedAids& a) override;
    float car_impulse() const override { return contact_; }
    float impact_ = 0, contact_ = 0;

private:
    Field& f_;
};

} // namespace

EventStart event_start(const data::Event& ev) {
    EventStart e;
    e.laps = ev.laps;
    e.cars = ev.cars_in_race;
    ByteSpan b = ev.bytes;
    if (b.size() < 0x1F4) return e;
    e.initial_kmh = float(b[0x198]);
    e.time_limit_s = 60.0 * double(b[0x1E4]); // u8 (+0x1E5 is the league byte)
    e.flying_m = -float(s16(rd16(b, 0x1E6)));
    e.type = b[0x1F3];
    switch (e.type) {
    case 0: e.countdown = double(b[0x1F2]); break;
    case 1: e.countdown = 4.0; break;
    case 2: e.countdown = 10.0; break;
    default: e.countdown = 0; break;
    }
    return e;
}

sim::Vec3 grid_back(const sim::Vec3& p, const sim::Quat& q, const sim::VehicleSetup& setup) {
    // physics §9.2: the CoG is moved back from the slot by the CoG->front extent; the car
    // origin sits cog_offset behind / below the CoG (horizontal part).
    float ext = setup.cog_front_extent;
    sim::Vec3 fwd = rotate(q, {0, 0, -1});
    sim::Vec3 c = rotate(q, setup.cog_offset);
    return {p.x - fwd.x * ext - c.x, p.y, p.z - fwd.z * ext - c.z};
}

sim::Vec3 grid_back(const sim::Vec3& p, const sim::Quat& q, const sim::CarSpec& spec) {
    std::unique_ptr<sim::Vehicle> v = sim::Vehicle::create(spec);
    if (!v) return p;
    return grid_back(p, q, v->setup());
}

// kg per PS: mass over the peak of torque x rpm on the full-throttle curve.
float power_to_weight(const sim::CarSpec& s) {
    float best_w = 0;
    for (size_t i = 0; i < s.engine.curve_rpm.size() && i < s.engine.curve_torque.size(); ++i)
        best_w = std::max(best_w, s.engine.curve_torque[i] * s.engine.curve_rpm[i] * 0.10471976f); // N*m * rad/s
    float ps = best_w / 735.49875f;
    return ps > 1 ? s.mass / ps : 1e9f;
}

bool ranks_ahead(const RaceField::Standing& a, const RaceField::Standing& b) {
    if (a.laps != b.laps) return a.laps > b.laps;
    if (a.finished != b.finished) return !a.finished; // a finished car's place on the lap stopped at the line
    if (a.finished) return a.time < b.time;
    return a.distance > b.distance;
}

bool finishes_at_line(int laps_completed, int race_laps, bool leader_finished, double race_time, double time_limit) {
    return (race_laps > 0 && laps_completed == race_laps) || leader_finished || (time_limit > 0 && race_time >= time_limit);
}

namespace {

class Field : public RaceField {
public:
    Field(const RaceTrack& rt, const track::Track& t, const FieldConfig& cfg)
        : track_(t), cfg_(cfg), walls_(t), world_(rt.query(), &walls_), player_(*this) {}

    bool build(const data::CarDatabase& db, std::string* error) {
        course_ = std::make_shared<ai::AiCourse>(track_);
        const data::Event* ev = cfg_.event.empty() ? nullptr : db.event(cfg_.event);
        labels_ = cfg_.opponents;
        if (labels_.empty() && ev) labels_ = ev->entries;
        if (int(labels_.size()) > cfg_.max_opponents) labels_.resize(size_t(cfg_.max_opponents));
        if (ev) rubber_ = ai::RubberBand::from_event(*ev);
        int slots = std::max(track_.grid_count(), 1);
        struct Entry {
            std::string label, model;
            sim::CarSpec spec;
        };
        std::vector<Entry> entries;
        for (const std::string& label : labels_) {
            std::string err;
            std::optional<sim::CarSpec> spec = sim::build_car_spec(db, label, &err);
            std::string car = base_car_of(db, label);
            const data::Car* c = car.empty() ? nullptr : db.car(car);
            if (!spec || !c) {
                LOG_WARN("Race field: opponent %s: %s", label.c_str(), err.c_str());
                continue;
            }
            entries.push_back({label, db.model_of(*c), *spec});
        }
        // Grid slot of each entry: given order, or by power-to-weight (GT mode).
        std::vector<int> slot_of(entries.size());
        for (size_t i = 0; i < entries.size(); ++i) slot_of[i] = int(i);
        if (cfg_.grid_by_power) {
            std::vector<size_t> order(entries.size());
            for (size_t i = 0; i < order.size(); ++i) order[i] = i;
            std::vector<float> kg_per_ps(entries.size());
            for (size_t i = 0; i < entries.size(); ++i) kg_per_ps[i] = power_to_weight(entries[i].spec);
            std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return kg_per_ps[a] < kg_per_ps[b]; });
            for (size_t r = 0; r < order.size(); ++r) slot_of[order[r]] = int(r);
        }
        for (size_t i = 0; i < entries.size(); ++i) {
            Entry& e = entries[i];
            ai::adapt_spec_for_ai(e.spec);
            std::unique_ptr<sim::Vehicle> v = sim::Vehicle::create(e.spec);
            if (!v) continue;
            v->set_cpu_driver(true); // the original's CPU clutch request and AI aid block
            Opp o;
            o.label = e.label;
            o.model = e.model;
            int index = slot_of[i];
            track::Pose pose = index < slots ? track_.grid_pose(index)
                                             : track_.respawn_pose(track_.length() - 12.0f * float(index));
            // physics §9.2: the slot point is the car's nose (moved back by its front extent).
            pose.position = grid_back(pose.position, pose.orientation, v->setup());
            if (cfg_.start_kmh > 0) v->spawn_moving(pose.position, pose.orientation, cfg_.start_kmh / 3.6f);
            else v->reset(pose.position, pose.orientation); // standing start: no spawn hold
            o.v = v.get();
            o.world_index = world_.add(std::move(v));
            // ai §9.1/ai §9.2: the course's set for the car's driveline class (the built-in set of
            // the course category when the course has none), then the event's overrides for that
            // class; the set's aid block is what the physics reads for the car.
            const int cls = ai::driveline_class(e.spec);
            ai::DriverParams params = ai::course_params(track_.driver_sets(), cfg_.course_category, cls);
            if (ev) ai::apply_event(params, *ev, cls);
            o.v->set_scripted_aids(params.aids);
            o.driver = std::make_unique<ai::AiDriver>(course_, params, index);
            o.driver->reset(o.v->state());
            o.timer = std::make_unique<track::LapTimer>(track_);
            o.timer->set_lap1_from_go(true); // a race: every car's lap 1 from GO
            o.timer->update(pose.position, 0);
            o.prev = o.v->state();
            // CPU pit stops (ai §13): only in races with tyre wear and a pit lane.
            if (cfg_.wear.on()) {
                o.pit = std::make_unique<PitLane>();
                if (!o.pit->load(track_, int(opps_.size()) + 1)) o.pit.reset();
            }
            opps_.push_back(std::move(o));
        }
        // Online battle: the other players' cars, posed from the network (no AI, never stepped).
        for (const FieldConfig::Remote& r : cfg_.remote) {
            std::string err;
            std::optional<sim::CarSpec> spec = r.spec ? std::optional<sim::CarSpec>(*r.spec) : sim::build_car_spec(db, r.car, &err);
            std::string car = base_car_of(db, r.car);
            const data::Car* c = car.empty() ? nullptr : db.car(car);
            std::unique_ptr<sim::Vehicle> v = spec && c ? sim::Vehicle::create(*spec) : nullptr;
            if (!v) { // a car this disc / mod set lacks (e.g. another player's mod car): shown as ours
                LOG_WARN("Race field: remote car %s: %s; shown as %s", r.car.c_str(), err.c_str(), cfg_.player.c_str());
                car = base_car_of(db, cfg_.player);
                c = car.empty() ? nullptr : db.car(car);
                spec = cfg_.player_spec ? std::optional<sim::CarSpec>(*cfg_.player_spec)
                                        : sim::build_car_spec(db, cfg_.player, &err);
                v = spec && c ? sim::Vehicle::create(*spec) : nullptr;
            }
            if (!v) {
                if (error) *error = "remote player's car " + r.car + ": " + err;
                return false;
            }
            Opp o;
            o.label = r.car;
            o.model = db.model_of(*c);
            o.remote = true;
            track::Pose pose = r.slot < slots ? track_.grid_pose(r.slot)
                                              : track_.respawn_pose(track_.length() - 12.0f * float(r.slot));
            pose.position = grid_back(pose.position, pose.orientation, v->setup());
            v->reset(pose.position, pose.orientation);
            o.v = v.get();
            o.world_index = world_.add(std::move(v));
            world_.set_kinematic(o.world_index, true);
            o.timer = std::make_unique<track::LapTimer>(track_);
            o.timer->set_lap1_from_go(true);
            o.timer->update(pose.position, 0);
            o.prev = o.v->state();
            labels_.push_back(r.car);
            opps_.push_back(std::move(o));
        }
        if (opps_.size() != labels_.size()) { // keep labels() = the cars actually in the race
            labels_.clear();
            for (const Opp& o : opps_) labels_.push_back(o.label);
        }
        std::string err;
        std::optional<sim::CarSpec> pspec;
        if (cfg_.player_spec) pspec = *cfg_.player_spec;
        else pspec = sim::build_car_spec(db, cfg_.player, &err);
        if (!pspec) {
            if (error) *error = "player car " + cfg_.player + ": " + err;
            return false;
        }
        std::unique_ptr<sim::Vehicle> pv = sim::Vehicle::create(*pspec);
        if (!pv) {
            if (error) *error = "player car " + cfg_.player + " could not be created";
            return false;
        }
        me_ = world_.add(std::move(pv));
        if (!cfg_.second_player.empty() || cfg_.second_spec) {
            std::optional<sim::CarSpec> s2;
            if (cfg_.second_spec) s2 = *cfg_.second_spec;
            else s2 = sim::build_car_spec(db, cfg_.second_player, &err);
            std::unique_ptr<sim::Vehicle> v2 = s2 ? sim::Vehicle::create(*s2) : nullptr;
            if (!v2) {
                if (error) *error = "second player car " + cfg_.second_player + ": " + err;
                return false;
            }
            TrackPose g2 = second_grid();
            g2.position = grid_back(g2.position, g2.orientation, v2->setup());
            v2->reset(g2.position, g2.orientation);
            me2_ = world_.add(std::move(v2));
            timer2_ = std::make_unique<track::LapTimer>(track_);
            timer2_->set_lap1_from_go(true);
            timer2_->update(g2.position, 0);
            prev2_ = world_.vehicle(me2_).state();
        }
        // Tyre wear (endurance events; single player only): every car, a fresh set.
        if (cfg_.wear.on())
            for (int i = 0; i < world_.size(); ++i) world_.vehicle(i).set_tire_wear(cfg_.wear);
        world_.set_car_collisions(true);
        player_timer_ = std::make_unique<track::LapTimer>(track_);
        player_timer_->set_lap1_from_go(true);
        snaps_.resize(size_t(world_.size()));
        race_time_ = -cfg_.countdown;
        order_.push_back(-1);
        if (me2_ >= 0) order_.push_back(-2);
        for (size_t i = 0; i < opps_.size(); ++i) order_.push_back(int(i));
        rank();
        LOG_INFO("Race field: %d opponents (event %s), grid %s, player on grid slot %d", int(opps_.size()),
                 cfg_.event.c_str(), cfg_.grid_by_power ? "by power-to-weight" : "in order", player_slot());
        return true;
    }

    int player_slot() const {
        if (cfg_.player_slot >= 0) return std::min(cfg_.player_slot, std::max(track_.grid_count(), 1) - 1);
        return std::min(int(opps_.size()), std::max(track_.grid_count(), 1) - 1);
    }
    int second_slot() const { return std::min(int(opps_.size()) + 1, std::max(track_.grid_count(), 1) - 1); }
    // 2P handicap: distance before the line of player 1 / player 2 (0 = grid).
    float handicap_back(bool second) const {
        float h = cfg_.handicap_m;
        if (h == 0 || (cfg_.second_player.empty() && !cfg_.second_spec)) return 0;
        bool back = second ? h > 0 : h < 0;
        return back ? std::max(std::fabs(h), 20.0f) : 10.0f;
    }
    TrackPose player_grid() const override {
        if (float d = handicap_back(false); d > 0) {
            track::Pose p = track_.respawn_pose(track_.length() - d);
            return {p.position, p.orientation};
        }
        track::Pose p = track_.grid_pose(player_slot());
        return {p.position, p.orientation};
    }

    // --- RaceField ---
    RaceVehicle& player() override { return player_; }
    int opponents() const override { return int(opps_.size()); }
    const std::string& opponent_label(int i) const override { return opps_[size_t(i)].label; }
    const std::string& opponent_model(int i) const override { return opps_[size_t(i)].model; }
    const sim::VehicleState& opponent_state(int i) const override { return opps_[size_t(i)].v->state(); }
    const sim::VehicleState& opponent_prev_state(int i) const override { return opps_[size_t(i)].prev; }
    const sim::VehicleSetup& opponent_setup(int i) const override { return opps_[size_t(i)].v->setup(); }
    int opponent_laps(int i) const override { return opps_[size_t(i)].timer->laps_completed(); }
    int opponent_lap_count(int i) const override {
        const track::LapTimer& t = *opps_[size_t(i)].timer;
        return lap_counter(t.lap_started(), t.laps_completed());
    }
    double race_time() const override { return race_time_; }
    int position_of(int car) const {
        auto it = std::find(order_.begin(), order_.end(), car);
        return it == order_.end() ? 0 : int(it - order_.begin()) + 1;
    }
    int player_position() const override { return position_of(-1); }
    double player_finish() const override { return finish_player_; }
    double second_finish() const override { return me2_ >= 0 ? finish2_ : -1; }
    int contacts_last_tick() const override { return int(world_.contacts().size()); }
    const std::vector<std::string>& labels() const override { return labels_; }

    bool has_second() const override { return me2_ >= 0; }
    void set_second_inputs(const sim::ControlInputs& in) override { in2_ = in; }
    void set_second_reverse(bool on) override {
        if (me2_ >= 0) world_.vehicle(me2_).set_reverse_request(on);
    }
    const sim::VehicleState& second_state() const override { return world_.vehicle(me2_).state(); }
    const sim::VehicleState& second_prev_state() const override { return prev2_; }
    const sim::VehicleSetup& second_setup() const override { return world_.vehicle(me2_).setup(); }
    TrackPose second_grid() const override {
        if (float d = handicap_back(true); d > 0) {
            track::Pose p = track_.respawn_pose(track_.length() - d);
            return {p.position, p.orientation};
        }
        int slot = second_slot();
        if (slot == player_slot()) { // a one-slot grid: start 8 m behind
            track::Pose p = track_.respawn_pose(track_.length() - 8.0f);
            return {p.position, p.orientation};
        }
        track::Pose p = track_.grid_pose(slot);
        return {p.position, p.orientation};
    }
    void reset_second(const TrackPose& p) override {
        world_.vehicle(me2_).spawn(p.position, p.orientation);
        timer2_->teleport(p.position);
        prev2_ = world_.vehicle(me2_).state();
    }
    float second_impact() const override { return impact2_; }
    float second_boost() const override { return world_.vehicle(me2_).telemetry().boost; }
    const sim::Vehicle* opponent_vehicle(int i) const override { return opps_.at(size_t(i)).v; }
    float opponent_brake(int i) const override { return opps_.at(size_t(i)).brake; }
    float opponent_car_impulse(int i) const override { return opps_.at(size_t(i)).contact; }
    float second_brake() const override { return brake2_; }
    const sim::Vehicle* second_vehicle() const override { return me2_ >= 0 ? &world_.vehicle(me2_) : nullptr; }
    float second_wall_impulse() const override { return world_.vehicle(me2_).telemetry().wall_impulse; }
    float second_car_impulse() const override { return contact2_; }
    LapInfo second_laps() const override {
        LapInfo l;
        if (!timer2_) return l;
        double t = std::max(race_time_, 0.0);
        l.laps = timer2_->laps_completed();
        l.started = timer2_->lap_started();
        l.wrong_way = timer2_->wrong_way();
        l.current = timer2_->current_lap_time(t);
        l.last = timer2_->last_lap_time();
        l.best = timer2_->best_lap_time();
        l.race_distance = timer2_->race_distance();
        return l;
    }
    void set_laps(int laps) override { laps_ = laps; }
    std::vector<int> take_pit_entries() override { return std::exchange(pit_entries_, {}); }
    const std::vector<CarCrossing>& last_crossings() const override { return crossings_; }
    bool opponent_in_pits(int i) const override { const Opp& o = opps_.at(size_t(i)); return o.pit && o.pit->driving(); }
    Standing standing_of(int car) const {
        Standing s;
        s.car = car;
        const track::LapTimer& t = car == -1 ? *player_timer_ : car == -2 ? *timer2_ : *opps_[size_t(car)].timer;
        s.time = car == -1 ? finish_player_ : car == -2 ? finish2_ : opps_[size_t(car)].finish;
        s.finished = s.time >= 0;
        s.laps = t.laps_completed(); // (a finished car's timer is frozen at its finish)
        s.distance = t.race_distance();
        return s;
    }
    // The race order after a tick: ranks_ahead over the last order (a stable sort, as the
    // original's insertion sort: ties keep their places).
    void rank() {
        std::vector<Standing> v;
        for (int car : order_) v.push_back(standing_of(car));
        std::stable_sort(v.begin(), v.end(), ranks_ahead);
        for (size_t i = 0; i < v.size(); ++i) order_[i] = v[i].car;
    }
    std::vector<Standing> standings() const override {
        std::vector<Standing> out;
        for (int car : order_) out.push_back(standing_of(car));
        return out;
    }
    bool opponent_remote(int i) const override { return opps_.at(size_t(i)).remote; }
    void set_remote_pose(int i, const RemoteCar::Pose& p) override {
        Opp& o = opps_.at(size_t(i));
        if (!o.remote) return;
        o.pose = p;
        o.have_pose = true;
    }
    void set_remote_finish(int i, double t) override {
        Opp& o = opps_.at(size_t(i));
        if (o.remote && o.finish < 0) o.finish = t;
        o.finished = true; // a finished car drives on without contact
        update_contact(i);
    }
    void set_remote_retired(int i) override {
        Opp& o = opps_.at(size_t(i));
        o.retired = true;
        update_contact(i);
    }
    bool opponent_retired(int i) const override { return opps_.at(size_t(i)).retired; }
    std::vector<RemoteHit> take_remote_hits() override { return std::exchange(remote_hits_, {}); }
    double last_contact_with(int i) const override { return opps_.at(size_t(i)).last_contact; }
    double remote_observed_finish(int i) const override { return opps_.at(size_t(i)).observed_finish; }
    void apply_player_impulse(float px, float pz, float jx, float jz) override {
        world_.vehicle(me_).apply_impulse({double(px), double(pz)}, {double(jx), double(jz)});
        player_.contact_ = std::max(player_.contact_, std::hypot(jx, jz));
    }
    void set_remote_ghost(int i, bool on) override {
        Opp& o = opps_.at(size_t(i));
        if (!o.remote) return;
        o.ghost = on;
        update_contact(i);
    }
    bool opponent_ghost(int i) const override { const Opp& o = opps_.at(size_t(i)); return o.ghost || o.finished; }
    void update_contact(int i) {
        const auto& o = opps_.at(size_t(i));
        world_.set_active(o.world_index, !(o.ghost || o.finished || o.retired));
    }
    int second_position() const override { return timer2_ ? position_of(-2) : 0; }

    // --- player vehicle ---
    void reset_player(const TrackPose& p, float speed_mps = 0, bool grid = false) {
        if (speed_mps > 0) world_.vehicle(me_).spawn_moving(p.position, p.orientation, speed_mps);
        else if (grid) world_.vehicle(me_).reset(p.position, p.orientation);
        else world_.vehicle(me_).spawn(p.position, p.orientation);
        player_timer_->teleport(p.position);
    }
    const sim::Vehicle& player_vehicle() const { return world_.vehicle(me_); }
    void player_pit_stop(const sim::Quat& box) { world_.vehicle(me_).begin_pit_stop(box); }
    void player_engine_condition(const sim::EngineCondition& c) { world_.vehicle(me_).set_engine_condition(c); }
    void player_reverse(bool on) { world_.vehicle(me_).set_reverse_request(on); }
    void player_driver_kind(int kind) { world_.vehicle(me_).set_driver_kind(kind); }
    void player_scripted_aids(const sim::ScriptedAids& a) { world_.vehicle(me_).set_scripted_aids(a); }
    void set_dirt_course(bool dirt) {
        for (int i = 0; i < world_.size(); ++i) world_.vehicle(i).set_dirt_course(dirt);
    }

    void step(const sim::ControlInputs& in, double dt) {
        for (int i = 0; i < world_.size(); ++i) {
            const sim::Vehicle& v = world_.vehicle(i);
            ai::CarSnapshot& s = snaps_[size_t(i)];
            s.position = v.state().position;
            s.orientation = v.state().orientation;
            s.velocity = v.state().velocity;
            s.half_length = v.setup().length * 0.5f;
            s.half_width = v.setup().width * 0.5f;
            s.front = v.setup().cog_front_extent, s.rear = v.setup().cog_rear_extent; // front / rear extents from the CoG
            {
                const sim::Quat& q = v.state().orientation;
                sim::Vec3 u{q.x, q.y, q.z}, c = v.setup().cog_offset;
                sim::Vec3 t = u.cross(c) * 2.0f;
                s.position = v.state().position + c + t * q.w + u.cross(t); // the CoG (the car's reference point, ai §14)
            }
            s.time_scale = world_.time_scale(i);
        }
        double player_distance = player_timer_->race_distance();
        // Rubber band (ai §14, pass mode 0): a chain from the leading human car along the
        // ranking. Index 0 = player 1, 1 = player 2 (when present), then the opponents.
        std::vector<float> band;
        if (rubber_.enabled() && race_time_ >= 0) {
            std::vector<double> dist{player_distance};
            std::vector<bool> no_speedup{false};
            int ref = 0;
            if (me2_ >= 0 && timer2_) {
                dist.push_back(timer2_->race_distance()), no_speedup.push_back(false);
                if (dist[1] > dist[0]) ref = 1;
            }
            for (const Opp& o : opps_) {
                dist.push_back(o.timer->race_distance());
                no_speedup.push_back(o.v->telemetry().on_pit_surface);
            }
            band = rubber_.chain(dist, ref, no_speedup);
        }
        const size_t band_first = me2_ >= 0 && timer2_ ? 2 : 1;
        // 2P boost: the trailing player's time step runs faster (the original's rubber band).
        if (me2_ >= 0 && timer2_) {
            float s1 = 1, s2 = 1;
            if (cfg_.boost_amount > 0 && race_time_ >= 0) {
                double gap = player_distance - timer2_->race_distance();
                float g = float(std::fabs(gap)), a = cfg_.boost_amount;
                float sc = g >= cfg_.boost_full || g >= float(track_.length()) ? 1 + a
                           : g > cfg_.boost_start ? 1 + a * (g - cfg_.boost_start) / (cfg_.boost_full - cfg_.boost_start)
                                                  : 1.0f;
                (gap > 0 ? s2 : s1) = sc;
            }
            world_.set_time_scale(me_, s1);
            world_.set_time_scale(me2_, s2);
        }
        for (Opp& o : opps_) {
            o.prev = o.v->state();
            if (o.remote) {
                if (o.have_pose) {
                    sim::collide::Body b = o.v->collision_body();
                    b.pos = {double(o.pose.body_x), double(o.pose.body_z)};
                    b.heading = o.pose.heading;
                    b.vel = {double(o.pose.state.velocity.x), double(o.pose.state.velocity.z)};
                    b.vy = o.pose.state.velocity.y;
                    b.yaw_rate = o.pose.yaw_rate;
                    world_.vehicle(o.world_index).set_remote_pose(o.pose.state, b);
                    o.brake = o.pose.brake;
                }
                continue;
            }
            ai::AiContext ctx{race_time_, snaps_, o.world_index};
            ctx.rank = int(&o - opps_.data()); // grid order (pole = 0) while the race has not started
            // Countdown (ai §10): the clutch request is 0 until GO; the driver blips the throttle.
            world_.vehicle(o.world_index).set_clutch_hold(race_time_ < 0);
            world_.vehicle(o.world_index).set_race_hold(race_time_ < 0);
            {
                sim::ControlInputs ai_in = o.driver->update(*o.v, ctx, dt);
                o.brake = ai_in.brake;
                world_.set_inputs(o.world_index, ai_in);
            }
            world_.vehicle(o.world_index).set_reverse_request(o.driver->reverse_request()); // backing out of walls
            world_.vehicle(o.world_index).set_driver_kind(o.driver->kind());
            if (o.pit) { // armed by worn tyres (the "tires worn" flag): pit lane, stop, back to the line
                sim::Vehicle& v = world_.vehicle(o.world_index);
                const sim::VehicleTelemetry& t = v.telemetry();
                PitLane::In pi;
                pi.state = v.state();
                pi.on_pit_surface = t.on_pit_surface;
                pi.armed = t.tires_worn;
                for (int k = 0; k < 4; ++k) pi.want_service = pi.want_service || t.tire_wear[size_t(k)] > v.tire_wear_params().fresh;
                pi.race_over = o.finish >= 0 || (laps_ > 0 && o.timer->laps_completed() >= laps_ - 1);
                pi.stopping = v.spawn_hold_active();
                PitLane::Out po = o.pit->update(pi);
                if (po.entered) pit_entries_.push_back(int(&o - opps_.data()));
                if (po.begin_stop) v.begin_pit_stop(po.box);
                // The AI driver drives the pit lane itself (ai §13): the state's line (from the next
                // tick) and driver kind (2 lane, 3 stop line, 4 leaving; physics §8.6); at the exit
                // back to the main line and to racing.
                if (po.line) o.driver->set_line(po.line, po.blend);
                if (po.exited) o.driver->reset_recovery();
                if (po.auto_drive || po.exited) o.driver->set_kind(po.kind);
                v.set_driver_kind(o.driver->kind());
                if (po.hold) { // the 5 s stop: held with the brake
                    sim::ControlInputs held;
                    held.brake = 1;
                    world_.set_inputs(o.world_index, held);
                    o.brake = 1;
                    v.set_reverse_request(false);
                }
                if (po.auto_drive) {
                    world_.set_time_scale(o.world_index, 1); // no rubber band in the pits
                    o.respawn.hold(race_time_);              // not "lost" while in the pits
                }
            }
            float scale = 1;
            size_t bi = band_first + size_t(&o - opps_.data());
            if (bi < band.size()) scale = band[bi];
            world_.set_time_scale(o.world_index, scale);
        }
        sim::ControlInputs pin = in;
        // held on the grid until the start signal by the race hold (physics §9.3: in place, brakes off,
        // the engine can rev), every car of the field alike
        world_.vehicle(me_).set_race_hold(race_time_ < 0);
        sim::Vec3 before = world_.vehicle(me_).state().velocity;
        world_.set_inputs(me_, pin);
        sim::Vec3 before2;
        if (me2_ >= 0) {
            sim::ControlInputs p2 = in2_;
            world_.vehicle(me2_).set_race_hold(race_time_ < 0);
            prev2_ = world_.vehicle(me2_).state();
            before2 = prev2_.velocity;
            world_.set_inputs(me2_, p2);
            brake2_ = p2.brake;
            in2_.shift_up = in2_.shift_down = false; // edge requests are used once
        }
        world_.step(dt);
        race_time_ += dt;
        player_.contact_ = 0;
        contact2_ = 0;
        for (Opp& o : opps_) o.contact = 0;
        for (const sim::World::Contact& c : world_.contacts()) {
            for (Opp& o : opps_)
                if (o.world_index == c.a || o.world_index == c.b) o.contact = std::max(o.contact, c.impulse);
            // Online: the player's car against a remote car (its owner may not have seen it).
            int other = c.a == me_ ? c.b : c.b == me_ ? c.a : -1;
            for (size_t k = 0; other >= 0 && k < opps_.size(); ++k)
                if (opps_[k].remote && opps_[k].world_index == other) {
                    float sign = c.b == other ? 1.0f : -1.0f; // the normal points from a to b
                    remote_hits_.push_back({int(k), c.point.x, c.point.z, c.normal.x * c.impulse * sign,
                                            c.normal.z * c.impulse * sign});
                    opps_[k].last_contact = race_time_;
                }
            if (c.a == me_ || c.b == me_) player_.contact_ = std::max(player_.contact_, c.impulse);
            if (me2_ >= 0 && (c.a == me2_ || c.b == me2_)) contact2_ = std::max(contact2_, c.impulse);
        }
        sim::Vec3 dv = world_.vehicle(me_).state().velocity - before;
        dv.y = 0;
        player_.impact_ = std::max(0.0f, dv.length() - 2.0f * 9.81f * float(dt));
        const bool player_crossed = player_timer_->update(world_.vehicle(me_).state().position, std::max(race_time_, 0.0));
        bool second_crossed = false;
        if (me2_ >= 0) {
            sim::Vec3 dv2 = world_.vehicle(me2_).state().velocity - before2;
            dv2.y = 0;
            impact2_ = std::max(0.0f, dv2.length() - 2.0f * 9.81f * float(dt));
            second_crossed = timer2_->update(world_.vehicle(me2_).state().position, std::max(race_time_, 0.0));
        }
        double t = std::max(race_time_, 0.0);
        for (Opp& o : opps_) {
            if (o.remote) { // lap timing from where it is shown; its finish comes from its owner
                o.crossed = o.timer->update(o.v->state().position, t);
                continue;
            }
            // The non-original safety respawn (ai §12; [race] respawn = off: never, as the original).
            if (o.respawn.lost(track_, o.v->state(), race_time_) && cfg_.cpu_respawn) {
                ai::respawn_car(world_.vehicle(o.world_index), *o.timer, *o.driver, track_);
                o.respawn.respawned(race_time_);
            }
            o.crossed = o.timer->update(o.v->state().position, t);
            o.respawn.progress(o.timer->race_distance(), race_time_);
        }
        // This tick's split / line crossings in the last race order (the leader first).
        crossings_.clear();
        for (int car : order_) {
            const track::LapTimer* tm = car == -1 ? player_timer_.get()
                                        : car == -2 ? timer2_.get()
                                                    : opps_[size_t(car)].timer.get();
            if (!tm) continue;
            for (const track::LapTimer::Crossing& c : tm->crossings())
                crossings_.push_back({car, {c.point, c.line, c.lap, c.ms, c.split_ms}});
        }
        // Finishes at the line (finishes_at_line), judged in the last race order with the
        // leader first, so a car crossing on the tick the leader finishes finishes too. The
        // finish time is the crossing's; the car's timer stops there (no more laps).
        auto finishes = [&](const track::LapTimer& tm) {
            return finishes_at_line(tm.laps_completed(), laps_, standing_of(order_.front()).finished,
                                    tm.last_crossing_time(), cfg_.time_limit);
        };
        for (int car : order_) {
            if (car == -1) {
                if (player_crossed && finish_player_ < 0 && finishes(*player_timer_))
                    finish_player_ = player_timer_->last_crossing_time(), player_timer_->freeze();
            } else if (car == -2) {
                if (second_crossed && finish2_ < 0 && finishes(*timer2_)) finish2_ = timer2_->last_crossing_time(), timer2_->freeze();
            } else {
                Opp& o = opps_[size_t(car)];
                if (!o.crossed) continue;
                // A remote car: when we saw it finish (its owner's claim is checked against it).
                if (o.remote && o.observed_finish < 0 && (o.finish >= 0 || finishes(*o.timer)))
                    o.observed_finish = o.timer->last_crossing_time(), o.timer->freeze();
                else if (!o.remote && o.finish < 0 && finishes(*o.timer)) {
                    o.finish = o.timer->last_crossing_time(), o.timer->freeze();
                    // The run-on after the finish (ai §1.1): the AI drives on, slower (kind 1).
                    o.driver->set_params(ai::after_finish(o.driver->params()));
                    if (o.driver->kind() == ai::kKindNormal) o.driver->set_kind(ai::kKindAi);
                }
            }
        }
        rank();
    }

private:
    struct Opp {
        std::string label, model;
        sim::Vehicle* v = nullptr;
        int world_index = -1;
        std::unique_ptr<ai::AiDriver> driver;
        std::unique_ptr<track::LapTimer> timer;
        std::unique_ptr<PitLane> pit;
        ai::SafetyRespawn respawn;
        sim::VehicleState prev;
        double finish = -1;
        bool crossed = false; // completed a lap on the last tick
        float brake = 0; // last tick's brake pedal
        float contact = 0; // strongest car-car impulse of the last step
        // Online battle: a remote player's car.
        bool remote = false, retired = false, have_pose = false, ghost = false, finished = false;
        double last_contact = -1e18; // race time the player's car last touched it
        double observed_finish = -1;  // when we saw it complete the race distance
        RemoteCar::Pose pose;
    };

    const track::Track& track_;
    FieldConfig cfg_;
    sim::TrackWorld walls_;
    sim::World world_;
    PlayerVehicle player_;
    std::shared_ptr<ai::AiCourse> course_;
    ai::RubberBand rubber_;
    std::vector<Opp> opps_;
    std::vector<int> pit_entries_;
    std::vector<CarCrossing> crossings_;
    std::vector<RemoteHit> remote_hits_;
    std::vector<std::string> labels_;
    std::vector<ai::CarSnapshot> snaps_;
    std::unique_ptr<track::LapTimer> player_timer_;
    int me_ = -1, me2_ = -1;
    float brake2_ = 0;
    int laps_ = 0;
    double finish_player_ = -1, finish2_ = -1;
    std::vector<int> order_; // the race order (ranks_ahead), kept from tick to tick
    std::unique_ptr<track::LapTimer> timer2_;
    sim::ControlInputs in2_;
    sim::VehicleState prev2_;
    float impact2_ = 0, contact2_ = 0;
    double race_time_ = 0;
};

void PlayerVehicle::reset(const TrackPose& p) { f_.reset_player(p); }
void PlayerVehicle::place(const TrackPose& p) { f_.reset_player(p, 0, true); }
void PlayerVehicle::set_dirt_course(bool dirt) { f_.set_dirt_course(dirt); }
void PlayerVehicle::reset_moving(const TrackPose& p, float v) { f_.reset_player(p, v); }
void PlayerVehicle::step(const sim::ControlInputs& in, double dt) { f_.step(in, dt); }
const sim::VehicleState& PlayerVehicle::state() const { return f_.player_vehicle().state(); }
const sim::VehicleSetup& PlayerVehicle::setup() const { return f_.player_vehicle().setup(); }
const sim::Vehicle* PlayerVehicle::sim_vehicle() const { return &f_.player_vehicle(); }
void PlayerVehicle::begin_pit_stop(const sim::Quat& box) { f_.player_pit_stop(box); }
void PlayerVehicle::set_engine_condition(const sim::EngineCondition& c) { f_.player_engine_condition(c); }
sim::EngineCondition PlayerVehicle::engine_condition() const { return f_.player_vehicle().engine_condition(); }
void PlayerVehicle::set_reverse_request(bool on) { f_.player_reverse(on); }
void PlayerVehicle::set_driver_kind(int kind) { f_.player_driver_kind(kind); }
void PlayerVehicle::set_scripted_aids(const sim::ScriptedAids& a) { f_.player_scripted_aids(a); }
float PlayerVehicle::boost() const { return f_.player_vehicle().telemetry().boost; }
float PlayerVehicle::wall_impulse() const { return f_.player_vehicle().telemetry().wall_impulse; }

} // namespace

std::unique_ptr<RaceField> RaceField::create(const RaceTrack& track, const data::CarDatabase& db,
                                             const FieldConfig& cfg, std::string* error) {
    const track::Track* t = track.runtime();
    if (!t) {
        if (error) *error = "opponents need the track runtime";
        return nullptr;
    }
    auto f = std::make_unique<Field>(track, *t, cfg);
    if (!f->build(db, error)) return nullptr;
    return f;
}

} // namespace gt3r::race
