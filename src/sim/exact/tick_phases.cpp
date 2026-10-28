// The whole phases whose calls are ported one by one (phases 7 and 12), and the race step's vehicle phases; see tick.h.

#include "sim/exact/tick.h"

#include "sim/exact/pit_lane.h"

namespace gt3r::sim::exact {

void dynamics(Car& c, const Statics& s, const GearSettings& g) {
    brakes_and_clutch(c);
    rolling_resistance(c, s);
    tyre_curves(c);
    tyre_long_force(c, s);
    tyre_slip_force(c);
    forward_step(c);
    wheel_net_torque(c);
    torque_vectoring(c);
    axle_step(c);
    wheel_speeds(c);
    engine_speed(c);
    gear_step(c, g);
    const bool air = airborne(c);
    lateral_yaw(c, air);
    stop_hold(c, air);
}

ChassisOutputs chassis(Car& c, const ChassisInputs& in) {
    ChassisOutputs out;
    hub_offsets_xy(c);
    speed_ramp(c);
    out.counts = ground_contact(c, *in.course, *in.bumps, in.mark_flagged);
    if (in.course_out) course_out(c, *in.course_out, out.counts);
    terrain_factor(c);
    suspension(c);
    attitude(c, in.box_heading);
    bump_nudge(c);
    pad_rumble(c);
    steer_force(c, *in.feel);
    tyre_effects(c, *in.effects, in.course_kind);
    phase12_stub(c);
    engine_sound(c, in.sound_scales);
    effects_reset(c, in.x70, in.ghost_switch);
    view_follow(c);
    hub_offsets(c);
    body_axes(c);
    ground_frame(c);
    speed_bookkeeping(c, in.condition);
    wheel_spin(c);
    out.effects = tyre_effect_requests(c);
    return out;
}

void tick_until_collision(std::span<Car> cars, std::span<const CarInputs> in, const RaceInputs& race, AiPlan& plan) {
    const size_t n = cars.size();
    auto run = [&](size_t i) { return !in[i].posed; };
    for (size_t i = 0; i < n; ++i)
        if (run(i)) save_previous(cars[i]);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) prep(cars[i], race.prep);
    const u8 course_kind = course_category(race.aids.course_category);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) driver_input(cars[i], race.pad, in[i].pad, course_kind, in[i].ai, plan);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) wheel_steer(cars[i]);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) slip_ratios(cars[i], race.statics);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) aids(cars[i], race.aids);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) dynamics(cars[i], race.statics, race.gears);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) brake_heat(cars[i], race.brake_heat);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) clear_feedback(cars[i]);
}

void tick_after_collision(std::span<Car> cars, std::span<const CarInputs> in, const RaceInputs& race, Proximity* k,
                          Vu0Random& rng, const EmitHook& emit) {
    const size_t n = cars.size();
    auto run = [&](size_t i) { return !in[i].posed; };
    if (k) proximity(cars, *k);
    ChassisInputs ch;
    ch.course = race.course, ch.bumps = &race.bumps, ch.feel = &race.feel, ch.effects = &race.effects;
    ch.course_kind = race.aids.course_category, ch.sound_scales = race.sound_scales, ch.condition = race.prep.condition;
    ch.course_out = race.course_out.on ? &race.course_out : nullptr;
    for (size_t i = 0; i < n; ++i) {
        if (!run(i)) continue;
        ch.mark_flagged = in[i].mark_flagged, ch.box_heading = in[i].box_heading;
        ch.x70 = in[i].x70, ch.ghost_switch = in[i].ghost_switch;
        emit(int(i), chassis(cars[i], ch), rng);
    }
    for (size_t i = 0; i < n; ++i)
        if (run(i)) tire_wear(cars[i], race.prep.wear);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) wear_flags(cars[i], race.prep.wear, in[i].pit);
    for (size_t i = 0; i < n; ++i)
        if (run(i)) flaps(cars[i], rng);
}

void tick(std::span<Car> cars, std::span<const CarInputs> in, const RaceInputs& race, AiPlan& plan,
          const std::function<void(std::span<Car>)>& collide, Proximity* k, Vu0Random& rng, const EmitHook& emit, FlowStep* flow) {
    tick_until_collision(cars, in, race, plan);
    collide(cars);
    tick_after_collision(cars, in, race, k, rng, emit);
    if (flow) tick_flow(cars, *flow, plan);
}

} // namespace gt3r::sim::exact
