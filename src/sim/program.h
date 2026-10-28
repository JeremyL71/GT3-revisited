#pragma once

// The game program versions the exact core knows (sim/exact/program_data.h tells them apart) and how a race behaves.
// Where a ported routine differs between the releases, the port branches on the race's Behaviour: PAL runs the race step
// 50 times a second where the NTSC programs run it 60 times (its step length and every tick count differ: the rate), and
// PAL and NTSC-J also change some routines' rules (PAL's rubber band cap, NTSC-J's collision commit, ...). The differences
// are listed with the routines that have them.

#include "base/bytes.h"

#include <array>

namespace gt3r::sim {

enum class Program : u8 { NtscU, Pal, NtscJ };
constexpr int kPrograms = 3;
// "NTSC-U", "PAL", "NTSC-J".
const char* program_name(Program p);

// The routines whose rules (not their rate) differ between the releases, by their NTSC-U address. Each is followed as the
// release the race's rules name has it, or as the disc's program has it where ProgramData recognised another release's
// variant of that one routine (a build of its own: Behaviour::variant).
enum class Rule : u8 {
    RubberBand,      // the conductor's time-scale hook: PAL caps every race mode's time scale at 1.3
    BattleCap,       // the i.LINK battle's time-scale hook: its own cap (not in PAL's; NTSC-J has no such conductor)
    Oversteer,       // the AI's oversteer indicator: PAL's is 0 below 25 km/h
    PadRumble,       // the pad vibration: PAL leaves a car the AI drives at 0
    SteerForce,      // the steering-wheel force: PAL leaves a car the AI drives at 0x80
    LicenceStop,     // the licence's stop: PAL's speed 1.5 km/h
    LapRecord,       // a new record: PAL's needs a lower time
    SpeedEvent,      // the machine test's top speed: PAL's nothing for a ghost
    RecordsLocked,   // a car's times kept out of the records: PAL's callers test a ghost first
    LapTime,         // a lap time's events and records (PAL: a ghost's; NTSC-J: no general time event, locks)
    SectorTime,      // a sector time's (likewise)
    SectorDiff,      // a sector's difference to the best: NTSC-J's also with the race-flow byte
    LapDiff,         // a lap's difference (likewise)
    AngleWrap,       // the angle wrap: NTSC-J's a turn at a time
    RunUp,           // a player's run-up start: NTSC-J's has no shift lock-out
    BattleStart,     // the split-screen battle's start: NTSC-J's one battle conductor, two cars only
    CatchupAnswer,   // the battle's catch-up answer: NTSC-J's setting 2 for any set word
    WearAnswer,      // the battle's wear answer: NTSC-J's the word itself as the setting
    DrawMode,        // the car's draw mode: NTSC-J's has no race info + 0x70 test
    LapStartRestore, // the replay's lap-start restore: NTSC-J's does not rebuild the collision body
    CollisionCommit, // the collision commit: NTSC-J's no heading wrap, no yaw-rate clamp
    WallFriction,    // the wall friction: NTSC-J's without the zero-length tests
    PushSegment,     // the wall push-out: NTSC-J's takes the root of a zero length too
    LapUsable,       // the free run's lap handler: NTSC-J's lap check reads the player's device's own recording
    Count
};
constexpr int kRules = int(Rule::Count);
// "rubber band", ... (for the messages).
const char* rule_name(Rule r);

// How a race runs on the exact core: whose rules apply (the release whose own changes to the routines are followed) and the
// step rate. Races run at 60 steps a second with the rules of the disc's release (online races: NTSC-U's); a release's own
// recorded streams (PAL's demos) play at its own rate, PAL's 50 steps a second.
struct Behaviour {
    static constexpr u8 kFollow = 0xFF;
    Program rules = Program::NtscU;
    int rate = 60; // steps a second: 60, or 50 with PAL's rules (PAL's own release)
    // Per routine, the release whose variant the disc's program has where it is not `rules`' (ProgramData fills it for the
    // disc's own rules: a build whose routines are not all one release's); kFollow: `rules`'.
    std::array<u8, kRules> variant = follow_all();

    constexpr Program rule(Rule r) const {
        const u8 v = variant[size_t(r)];
        return v == kFollow ? rules : Program(v);
    }
    constexpr bool pal_rate() const { return rate == 50; }
    constexpr bool follows_rules() const {
        for (u8 v : variant)
            if (v != kFollow) return false;
        return true;
    }
    static constexpr std::array<u8, kRules> follow_all() {
        std::array<u8, kRules> a{};
        for (u8& v : a) v = kFollow;
        return a;
    }
    friend constexpr bool operator==(const Behaviour&, const Behaviour&) = default;
};
// How often the caller steps the race: 60 a second, PAL's own release 50.
constexpr int steps_per_second(const Behaviour& b) { return b.rate; }
// A release's whole own behaviour (its rules at its rate: PAL 50 steps a second).
constexpr Behaviour release_behaviour(Program p) { return {p, p == Program::Pal ? 50 : 60}; }

} // namespace gt3r::sim
