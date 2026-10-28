#pragma once

// GT3's float math (sinf, cosf, sincosf, atanf, atan2f) in EE arithmetic (ee_float.h), bit for bit: the same reduction and
// polynomial in the same operation order, so a port that calls the routine the program calls gets the program's result.
// sincos is a separate routine with its own reduction: its results can differ from sin / cos in the last bit, so each call
// site uses the one the program uses.

#include "base/ee_float.h"

namespace gt3r::ee {

F sin(F x);                       // sinf
F cos(F x);                       // cosf
F sin_uncached(F x);              // the same without the memo of recent results (ee_math.cpp)
F cos_uncached(F x);
void sincos(F x, F& s, F& c);     // sincosf
F atan(F x);                      // atanf
// atan2f: note the program's shortcut: 0 whenever x + y == 0 (so also for y = -x, not only at the origin).
F atan2(F y, F x);

} // namespace gt3r::ee
