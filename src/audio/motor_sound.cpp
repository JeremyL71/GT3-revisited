#include "audio/motor_sound.h"

#include "formats/ps_adpcm.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace gt3r {

namespace {

constexpr double kRate = 44100.0; // game pitch 0x1000
constexpr double kPi = 3.14159265358979;
// Layers by base rpm; each covers a pitch range well inside the SPU's 4x limit.
struct LayerDef {
    u16 base;
    s16 lo, hi;
};
constexpr LayerDef kLayers[] = {{1500, 0, 2500}, {5000, 3500, 7000}, {12000, 9000, 25500}};
// Loop length: ~0.4 s in whole frames.
constexpr size_t kLoopFrames = 630;
constexpr size_t kLoop = kLoopFrames * PsAdpcmDecoder::kFrameSamples;

// Frame 0 silent, frame 1 starts the loop, the last frame ends it and repeats.
u8 layer_flags(size_t f, size_t n) {
    if (f == n - 1) return 0x03; // end | repeat
    if (f == 1) return 0x04;     // loop start
    return 0;
}

// One loop of a tone whose partials fit a whole number of cycles in kLoop
// samples (seamless), plus optional periodic noise.
std::vector<s16> render(double f0, const std::vector<std::pair<double, double>>& partials, double noise, u32 seed) {
    std::vector<double> buf(kLoop, 0.0);
    for (const auto& [mult, amp] : partials) {
        double cycles = std::max(1.0, std::round(f0 * mult * kLoop / kRate));
        if (cycles * kRate / kLoop > kRate * 0.2) continue; // would alias once pitched up
        double phase = 0.37 * mult;
        for (size_t i = 0; i < kLoop; ++i) buf[i] += amp * std::sin(2 * kPi * cycles * double(i) / kLoop + phase);
    }
    if (noise > 0) { // band-passed hiss: white noise through a two-pole resonator near 3 x f0
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> u(-1, 1);
        double fc = std::min(f0 * 3.0, 6000.0), r = 0.97;
        double c1 = 2 * r * std::cos(2 * kPi * fc / kRate), c2 = -r * r;
        double y1 = 0, y2 = 0;
        std::vector<double> n(kLoop);
        for (int pass = 0; pass < 2; ++pass) // run twice so the loop starts in steady state
            for (size_t i = 0; i < kLoop; ++i) {
                double y = u(rng) * (1 - r) + c1 * y1 + c2 * y2;
                y2 = y1, y1 = y;
                n[i] = y;
            }
        double peak = 1e-9;
        for (double v : n) peak = std::max(peak, std::fabs(v));
        for (size_t i = 0; i < kLoop; ++i) buf[i] += noise * n[i] / peak;
    }
    double peak = 1e-9;
    for (double v : buf) peak = std::max(peak, std::fabs(v));
    std::vector<s16> out(PsAdpcmDecoder::kFrameSamples + kLoop, 0); // leading silent frame
    for (size_t i = 0; i < kLoop; ++i) out[PsAdpcmDecoder::kFrameSamples + i] = s16(std::lround(buf[i] / peak * 26000.0));
    return out;
}

std::shared_ptr<const sound::EngineBank> make_bank(double order, const std::vector<std::pair<double, double>>& partials,
                                                   double noise, s16 volume, u32 seed) {
    auto bank = std::make_shared<sound::EngineBank>();
    auto data = std::make_shared<Bytes>();
    u32 seed_n = seed;
    for (const LayerDef& l : kLayers) {
        double f0 = order * 0.15 * l.base; // Hz at the layer's base rpm
        std::vector<s16> pcm = render(f0, partials, noise, seed_n++);
        Bytes enc = ps_adpcm_encode(pcm.data(), pcm.size(), layer_flags);
        sound::EngineLayer el;
        el.base_rpm = l.base;
        el.lo_rpm = l.lo;
        el.hi_rpm = l.hi;
        el.volume = volume;
        el.pitch = 0x1000;
        el.offset = u32(data->size());
        el.size = u32(enc.size());
        data->insert(data->end(), enc.begin(), enc.end());
        bank->layers.push_back(el);
    }
    bank->unknown_28 = bank->unknown_2c = 127;
    bank->adpcm = data;
    return bank;
}

} // namespace

MotorBanks make_motor_banks(const MotorSoundParams& p) {
    MotorBanks b;
    double order = std::clamp(double(p.whine_order), 0.25, 4.0);
    auto vol = [](double level, double base) { return s16(std::clamp(base * level, 0.0, 16383.0)); };
    b.whine = make_bank(order, {{1, 1.0}, {2, 0.35}, {3, 0.12}, {0.5, 0.2}}, 0.0, vol(p.whine_level, 9000), 1);
    b.inverter = make_bank(order * 2.0, {{1, 0.6}, {1.5, 0.25}, {4, 0.15}}, 0.5, vol(p.inverter_level, 6500), 7);
    return b;
}

} // namespace gt3r
