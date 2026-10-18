#pragma once

// Minimal self-registering test framework.
//
//   TEST(gs_roundtrip) { CHECK(a == b); }
//   TEST(car_model_loads) {
//       Disc* disc = test_disc();          // nullptr unless GT3R_DISC is set
//       if (!disc) SKIP("needs GT3R_DISC");
//       ...
//   }
//
// Tests that read game data must never embed or write game data; they load
// it from the user's disc at run time and skip in CI.

#include "disc/disc.h"

#include <cstdio>
#include <functional>
#include <vector>

namespace gt3r::test {

struct TestCase {
    const char* name;
    std::function<void()> fn;
};

std::vector<TestCase>& registry();
int& failures();
Disc* test_disc(); // opened once from $GT3R_DISC; nullptr when unset

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

struct Skip {
    const char* reason;
};

} // namespace gt3r::test

#define TEST(name)                                                                           \
    static void test_##name();                                                               \
    static ::gt3r::test::Registrar test_reg_##name(#name, test_##name);                      \
    static void test_##name()

#define CHECK(cond)                                                                          \
    do {                                                                                     \
        if (!(cond)) {                                                                       \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                    \
            ++::gt3r::test::failures();                                                      \
        }                                                                                    \
    } while (0)

#define SKIP(reason) throw ::gt3r::test::Skip{reason}
