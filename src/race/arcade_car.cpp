#include "race/arcade_car.h"

#include "formats/db/gt_db.h"
#include "game/data/car_database.h"

namespace gt3r::race {

std::optional<ArcadeCar> arcade_car(const data::CarDatabase& db, const std::string& id, int tire_mode,
                                    const std::string& course, std::string* error) {
    for (const data::CarVariant& v : db.arcade_cars()) {
        if (v.label != id && v.label != "_" + id) continue;
        ArcadeCar out;
        out.label = v.label, out.car = v.car;
        std::array<u64, 27> parts = v.parts;
        int stage = tire_mode == 1 ? 11 : 10;
        for (const data::Course& c : db.courses())
            if (c.label == course) stage = c.surface == 2 ? 12 : c.surface == 1 ? stage + 13 : stage;
        out.stage = stage;
        const char sfx = char('a' + stage);
        for (size_t i = 0; i < data::kCarPartTables.size(); ++i) {
            int t = data::kCarPartTables[i];
            if (t != int(data::TableId::FrontTire) && t != int(data::TableId::RearTire)) continue;
            std::string label = std::string(t == int(data::TableId::FrontTire) ? "ft_" : "rt_") + v.car + "_" + sfx;
            if (db.record(data::TableId(t), label)) parts[i] = db::id_hash(label);
        }
        std::optional<sim::CarSpec> spec = sim::build_car_spec(db, v.car, parts, error);
        if (!spec) return std::nullopt;
        out.spec = std::move(*spec);
        return out;
    }
    if (error) *error = "not an arcade car: " + id;
    return std::nullopt;
}

int driver_param_set(const data::CarDatabase& db, const std::string& course) {
    for (const data::Course& c : db.courses())
        if (c.label == course) return c.surface == 1 || c.surface == 2 ? int(c.surface) : 0;
    return 0;
}

} // namespace gt3r::race
