#pragma once

// An in-memory data::CarDatabase for disc-free tests: an archive of empty tables (the
// container layouts are docs/formats/database.md's) filled with INVENTED car records. Every
// value in the records is a round number chosen here; none comes from a disc.

#include "formats/db/gt_db.h"
#include "game/data/car_database.h"

#include <cstring>
#include <memory>
#include <optional>
#include <string>

namespace gt3r::test::cardb {

inline void put16(Bytes& b, size_t o, u16 v) { b[o] = u8(v), b[o + 1] = u8(v >> 8); }
inline void put32(Bytes& b, size_t o, u32 v) { put16(b, o, u16(v)), put16(b, o + 2, u16(v >> 16)); }
inline void put64(Bytes& b, size_t o, u64 v) { put32(b, o, u32(v)), put32(b, o + 4, u32(v >> 32)); }

// GTAR of 36 empty GTDT tables (record sizes large enough for every record decoded here), and
// empty STDB string tables.
inline Bytes empty_paramdb() {
    const u32 n = u32(data::TableId::Count);
    Bytes out(16 + 4 * size_t(n + 1));
    std::memcpy(out.data(), "GTAR", 4);
    put32(out, 4, n);
    put32(out, 8, u32(out.size())); // data offset
    put32(out, 12, 7);
    for (u32 i = 0; i <= n; ++i) put32(out, 16 + 4 * size_t(i), 16 * i);
    for (u32 i = 0; i < n; ++i) {
        Bytes t(16);
        std::memcpy(t.data(), "GTDT", 4);
        put16(t, 6, u16(i));
        put16(t, 10, i == u32(data::TableId::Car) ? 0x110 : 0x200); // record size; no records
        put32(t, 12, 16);
        out.insert(out.end(), t.begin(), t.end());
    }
    return out;
}

inline Bytes empty_stdb() {
    Bytes b(16);
    std::memcpy(b.data(), "STDB", 4);
    put32(b, 12, 16);
    return b;
}

// A database without records (nullptr if it does not load). Fill it with add_car / put_record,
// then rebuild().
inline std::unique_ptr<data::CarDatabase> empty_database() {
    auto db = std::make_unique<data::CarDatabase>();
    const Bytes paramdb = empty_paramdb(), stdb = empty_stdb();
    bool ok = db->load([&](const std::string& path) -> std::optional<Bytes> {
        if (path == "database/paramdb_us.db") return paramdb;
        if (path == "database/paramstr_us.db" || path == "database/paramunistr_us.db") return stdb;
        return std::nullopt;
    });
    return ok ? std::move(db) : nullptr;
}

// Car `label`: chassis, engine and suspension records (everything else takes build_car_spec's
// defaults), and with a `model` a body record naming it. With the suspension block it is a
// spec-path car, as every disc car is. `torque` is the engine's torque word at every axis
// point, `rev_limit` its rev limit (x100 rpm).
inline void add_car(data::CarDatabase& db, const std::string& label, u16 torque, u8 rev_limit = 75,
                    const std::string& model = {}) {
    const std::string ch = "ch_" + label, en = "en_" + label, su = "su_" + label, rm = "rm_" + label;
    for (const std::string& l : {label, ch, en, su, rm}) db.add_label(l);
    Bytes c(0x40);
    put64(c, 0, db::id_hash(ch));
    put64(c, 8, db::id_hash(label));
    c[0x10] = 55;            // front weight %
    put16(c, 0x14, 4400);    // length mm
    put16(c, 0x16, 1300);    // height mm
    put16(c, 0x18, 2600);    // wheelbase mm
    put16(c, 0x1A, 1180);    // weight kg (+120 kg in the spec)
    c[0x1C] = 80;            // yaw inertia / 100
    c[0x1D] = 200;
    db.put_record(data::TableId::Chassis, c);
    Bytes e(0x60);
    put64(e, 0, db::id_hash(en));
    put64(e, 8, db::id_hash(label));
    const u8 axis[4] = {10, 30, 50, 70}; // x100 rpm
    for (int i = 0; i < 4; ++i) {
        put16(e, size_t(0x18 + 2 * i), torque);
        e[size_t(0x47 + i)] = axis[i];
    }
    e[0x44] = 80;        // idle x10 rpm
    e[0x45] = rev_limit; // rev limit x100
    e[0x46] = 70;        // upshift x100
    e[0x57] = 4;         // axis points
    db.put_record(data::TableId::Engine, e);
    // Suspension (stock settings and the block bytes build_car_spec converts; one-step setting
    // ranges, so each setting reads its minimum byte): springs 40 / 45 x 0.1 kgf/mm at a 100 %
    // motion ratio, ride height 120 mm, dampers through (0.1 m/s, 12 x 49 N) and (0.5 m/s, 49 x 49 N)
    // in bump, (0.1, 16 x 49) and (0.5, 61 x 49) in rebound, anti-roll bytes 60 / 40, progressive
    // byte 10, unsprung 40 x 0.25 kg, bump damping 20 x 98, knee gap 30 mm, bump room 80 mm,
    // droop 100 mm.
    Bytes s(0x80);
    put64(s, 0, db::id_hash(su));
    put64(s, 8, db::id_hash(label));
    s[0x1D] = s[0x1E] = 80;   // bump room
    s[0x1F] = s[0x20] = 100;  // droop
    s[0x23] = 40, s[0x26] = 45; // springs
    s[0x27] = s[0x28] = 100;  // motion ratios
    s[0x29] = s[0x2A] = 10;   // progressive gain
    for (size_t o : {0x2B, 0x32, 0x39, 0x40, 0x47, 0x4B}) s[o] = 1; // damper / anti-roll setting steps
    s[0x2C] = s[0x3A] = 12, s[0x2F] = s[0x3D] = 49; // bump forces (x 49 N)
    s[0x33] = s[0x41] = 16, s[0x36] = s[0x44] = 61; // rebound forces
    s[0x48] = 60, s[0x4C] = 40; // anti-roll
    put16(s, 0x54, 120), put16(s, 0x5A, 120); // ride heights (mm)
    s[0x5C] = s[0x5D] = 40;   // unsprung
    for (size_t o : {0x70, 0x72, 0x74, 0x76}) s[o] = 10; // damper speeds (x 0.01 m/s)
    for (size_t o : {0x71, 0x73, 0x75, 0x77}) s[o] = 50;
    s[0x78] = s[0x79] = 30;   // knee gap
    s[0x7A] = s[0x7B] = 20;   // bump damping
    db.put_record(data::TableId::Suspension, s);
    Bytes car(0x110);
    put64(car, 0, db::id_hash(label));
    put64(car, 8 + 8 * 3, db::id_hash(ch));  // part slot 3: chassis
    put64(car, 8 + 8 * 6, db::id_hash(en));  // part slot 6: engine
    put64(car, 8 + 8 * 19, db::id_hash(su)); // part slot 19: suspension
    if (!model.empty()) { // body (part slot 5): the model id, no weight, tread or inertia change
        db.add_label(model);
        Bytes b(0x48);
        put64(b, 0, db::id_hash(rm));
        put64(b, 8, db::id_hash(label));
        put64(b, 0x18, db::id_hash(model));
        put16(b, 0x30, 1000); // weight per mille
        b[0x38] = 100;        // inertia %
        db.put_record(data::TableId::RacingModify, b);
        put64(car, 8 + 8 * 5, db::id_hash(rm));
    }
    db.put_record(data::TableId::Car, car);
}

// Arcade car (table 35) `label` over base car `car`, with the base car's parts.
inline void add_arcade_car(data::CarDatabase& db, const std::string& label, const std::string& car) {
    db.add_label(label);
    Bytes v(0x100);
    put64(v, 0, db::id_hash(label));
    put64(v, 8, db::id_hash(car));
    if (const ByteSpan* r = db.record(data::TableId::Car, car))
        for (size_t i = 0; i < 27; ++i) std::memcpy(&v[16 + 8 * i], r->data() + 8 + 8 * i, 8);
    db.put_record(data::TableId::ArcadeCar, v);
}

} // namespace gt3r::test::cardb
