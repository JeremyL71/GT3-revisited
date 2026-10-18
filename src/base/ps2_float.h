#pragma once

// PS2 EE FPU single-precision add / multiply for code that must make the
// same float decisions as the original (random draws compared against
// weight sums). The EE FPU: results round towards zero,
// no denormals, overflow saturates, and the adder first drops the smaller
// operand's mantissa bits beyond one guard position. (sim/models.cpp keeps
// its own copy for the physics models.)

namespace gt3r::ps2 {

float mul(float a, float b);
float add(float a, float b);
// cvt.w.s: truncation towards zero, saturating.
int to_int(float f);

} // namespace gt3r::ps2
