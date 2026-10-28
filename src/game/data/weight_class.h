#pragma once

// The arcade class GT3 gives a car that is in no class list: Single Race puts a garage car
// on a road course in a class's race by its weight per power (docs/behaviour/gt_mode.md
// §16.1, `arcade_prepare`). Mod cars with `[arcade] class = auto` and the online lobby's
// cars without a list class are classed the same way.

namespace gt3r::data {

// By kg / PS: the spec's mass less 120 kg over the car's computed power. `cls`: 1 C, 2 B,
// 3 A, 4 S (game::arcade_class_tag); `next`: 1 = the class's next race ("+ 1", the
// heavier half of the class).
struct WeightClass {
    int cls = 1;
    int next = 1;
};
inline WeightClass class_by_weight(float kg_per_ps) {
    static constexpr struct {
        float below;
        int cls, next;
    } kTable[] = {{2.1f, 4, 0}, {3.0f, 4, 1}, {3.5f, 3, 0}, {5.0f, 3, 1}, {5.8f, 2, 0}, {6.3f, 2, 1}, {9.0f, 1, 0}};
    for (const auto& t : kTable)
        if (kg_per_ps < t.below) return {t.cls, t.next};
    return {1, 1};
}

} // namespace gt3r::data
