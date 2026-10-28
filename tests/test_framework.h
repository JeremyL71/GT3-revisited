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
//
// Files a test writes go in its process's own folder (temp_path / temp_dir), never in the
// working directory or a fixed path, so several test runs can go at once.

#include "disc/disc.h"

#include <cstdio>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace gt3r::test {

struct TestCase {
    const char* name;
    std::function<void()> fn;
};

std::vector<TestCase>& registry();
int& failures();
// Opened once from $GT3R_DISC; nullptr when unset. The disc tests are written against the
// NTSC-U data set: test_disc() is nullptr for another region's disc (those tests skip), and
// tests that work on every region use any_region_disc().
Disc* test_disc();
Disc* any_region_disc();

// This run's own folder: <temp>/gt3r-tests-<pid>-<random>, made as the run starts and removed
// when it ends (kept, and its path printed, when GT3R_KEEP_TEST_TMP is set). The run also
// points TMPDIR / TMP / TEMP at it and the user data folder of the code under test
// (GT3R_DATA_DIR, game::user_data_dir: saves, replays, mods) at its "user-data" folder.
const std::filesystem::path& temp_root();
// <temp root>/<name> with nothing there (whatever an earlier test left is removed).
std::filesystem::path temp_path(const std::string& name);
// <temp root>/<name> as a new empty folder.
std::filesystem::path temp_dir(const std::string& name);

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
