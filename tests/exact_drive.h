#pragma once

// Driving a car down the machine tests' straight on the game's own physics (the exact core), for the tests of what reads a
// car's state and telemetry (sounds, mod cars): event M1000's course, a standing start, the automatic gearbox. Needs the
// disc (its program and course).

#include "disc/core_image.h"
#include "disc/disc.h"
#include "game/data/car_database.h"
#include "race/backend.h"
#include "race/exact_field.h"
#include "sim/exact/program_data.h"
#include "sim/exact/race_inputs.h"
#include "vfs/vfs.h"

#include <cstdio>
#include <memory>
#include <string>

namespace gt3r::test {

struct StraightRun {
    std::unique_ptr<Vfs> vfs;
    std::unique_ptr<race::RaceTrack> track;
    std::unique_ptr<race::RaceField> field; // null when the race cannot be set up
    // One step with these inputs (the frame the pad would give: race::exact_inputs_frame).
    void step(const sim::ControlInputs& in) { field->player().step(in, 1.0 / field->tick_hz()); }
};

inline StraightRun straight_run(Disc& disc, const data::CarDatabase& db, const std::string& car) {
    StraightRun r;
    std::optional<Bytes> core = disc.volume().read_program();
    const data::Event* ev = db.event("M1000");
    if (!core || !ev) return r;
    auto data = std::make_shared<const sim::exact::ProgramData>(sim::exact::ProgramData::load(CoreImage::decode(*core)));
    r.vfs = std::make_unique<Vfs>(&disc.volume());
    std::optional<Bytes> file = r.vfs->read(course_file_path(*r.vfs, ev->course));
    if (!file) return r;
    r.track = race::make_track(*r.vfs, *file, ev->course);
    race::FieldConfig cfg;
    cfg.event = "M1000";
    cfg.max_opponents = 0;
    cfg.exact.data = data;
    cfg.exact.course_file = *file;
    cfg.exact.course = ev->course;
    cfg.exact.kind = sim::exact::Conductor::MachineTest;
    cfg.exact.player.car = {car};
    std::string err;
    r.field = race::make_exact_field(*r.track, db, cfg, &err);
    if (!r.field) std::printf("    %s: %s\n", car.c_str(), err.c_str());
    return r;
}

} // namespace gt3r::test
