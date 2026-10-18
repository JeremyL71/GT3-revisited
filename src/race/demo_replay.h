#pragma once

// The game's own replay saves: the disc's demo runs (demo/<event>: the licence
// tests' and arcade time attacks' demonstrations, docs/dev/RACE.md "Demo runs").
// They hold controller input (re-simulated on playback), not car states.
//
// Save envelope (0x40 bytes): u32 header size, u32 body length, u16 version, u16 kind
// (4 = replay), u32 CRC-32 of the stored body, label; every body byte is inverted.
// Body: "GT2K" replay header (version 3, 0x191C bytes; +0x78 u32 finish time ms, car
// records of 0x280 bytes from +0x518 (starting with the car's equipment record, the
// memory card's 0x1D0-byte car layout: car, parts, settings) with +0x1F8 transmission
// 0 AT / 1 MT, +0x1918
// recorder block count), then recorder blocks: u32 mode (0 = one input stream),
// u32 frame count T, u32 byte length (with these 8 bytes, 4-byte aligned), and a
// bitstream (MSB first) of change records: gamma(mask + 1), one delta per changed
// field, gamma(run). Delta: 3-bit class k, sign bit, (2k+1)-bit value added to
// {0, 2, 10, 42, 170, 682, 2730, 10922}[k]. Every frame the steering is first
// extrapolated to 2x - previous (when |x| >= 16, clamped to +-4096). Playback
// delivers T - 1 frames, one per simulation step from the first step of the race
// (countdown and run-up included). Pad record: flags, kind (2 analog, 3 digital
// steering rate), s8 shift (+1 up / -1 down), s16 steer (+-4096), |steer|, throttle
// and brake 0..4096, two spare axes.

#include "base/bytes.h"
#include "sim/sim_types.h"

#include <optional>
#include <string>
#include <vector>

namespace gt3r::race {

struct DemoFrame {
    s16 steer = 0;     // +-4096 = full lock
    s16 throttle = 0;  // 0..4096
    s16 brake = 0;     // 0..4096
    s8 shift = 0;      // +1 up, -1 down
    u8 kind = 2;
    u8 flags = 31;     // pad record flags (bit 0: analog steering)
};

struct DemoReplay {
    u32 time_ms = 0;        // the run's recorded time
    bool automatic = true;  // car record transmission
    Bytes car_record;       // the car record's first 0x1D0 bytes: a CarSaveEquipments (car, parts, settings)
    std::vector<DemoFrame> frames;

    // nullopt (with *error) when the data is not a version-3 replay with an input stream.
    static std::optional<DemoReplay> parse(ByteSpan file, std::string* error = nullptr);
    // The simulation's inputs for frame i; past the end an all-zero frame (spec §4.4).
    sim::ControlInputs inputs(size_t i) const;
    // The pad record kind of frame i (2 analog, 3 digital steering; 3 past the end).
    int kind(size_t i) const { return i < frames.size() ? frames[i].kind : 3; }
    size_t size() const { return frames.size(); }
};

} // namespace gt3r::race
