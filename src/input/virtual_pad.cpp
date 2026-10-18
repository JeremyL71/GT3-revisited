#include "input/virtual_pad.h"

#include "base/config.h"

namespace gt3r {

namespace {
constexpr std::string_view kButtonNames[kPadButtonCount] = {
    "Select", "L3", "R3", "Start", "Up", "Right", "Down", "Left",
    "L2", "R2", "L1", "R1", "Triangle", "Circle", "Cross", "Square",
};
constexpr std::string_view kAxisNames[kPadAxisCount] = {"LX", "LY", "RX", "RY"};
} // namespace

std::string_view pad_button_name(PadButton b) { return kButtonNames[static_cast<size_t>(b)]; }
std::string_view pad_axis_name(PadAxis a) { return kAxisNames[static_cast<size_t>(a)]; }

std::optional<PadButton> pad_button_from_name(std::string_view name) {
    std::string n = to_lower(name);
    for (size_t i = 0; i < kPadButtonCount; ++i)
        if (to_lower(kButtonNames[i]) == n) return static_cast<PadButton>(i);
    return std::nullopt;
}

std::optional<PadAxis> pad_axis_from_name(std::string_view name) {
    std::string n = to_lower(name);
    for (size_t i = 0; i < kPadAxisCount; ++i)
        if (to_lower(kAxisNames[i]) == n) return static_cast<PadAxis>(i);
    return std::nullopt;
}

} // namespace gt3r
