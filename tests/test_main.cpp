// Test runner. Tests live in test_*.cpp and register with TEST().
// Set GT3R_DISC=/path/to/gt3.iso to also run tests that need game data.

#include "test_framework.h"

#include <cstdlib>
#include <cstring>
#include <memory>

namespace gt3r::test {

std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

int& failures() {
    static int n = 0;
    return n;
}

Disc* test_disc() {
    static std::unique_ptr<Disc> disc;
    static bool tried = false;
    if (!tried) {
        tried = true;
        if (const char* path = std::getenv("GT3R_DISC")) {
            try {
                disc = Disc::open(path);
            } catch (const std::exception& e) {
                std::printf("  cannot open GT3R_DISC: %s\n", e.what());
            }
        }
    }
    return disc.get();
}

} // namespace gt3r::test

int main(int argc, char** argv) {
    using namespace gt3r::test;
    const char* filter = argc > 1 ? argv[1] : nullptr; // substring filter
    int skipped = 0;
    for (const TestCase& t : registry()) {
        if (filter && !std::strstr(t.name, filter)) continue;
        int before = failures();
        try {
            t.fn();
        } catch (const Skip& s) {
            std::printf("skip %s (%s)\n", t.name, s.reason);
            ++skipped;
            continue;
        } catch (const std::exception& e) {
            std::printf("  FAIL %s: exception: %s\n", t.name, e.what());
            ++failures();
        }
        std::printf("%s %s\n", failures() == before ? "ok  " : "FAIL", t.name);
    }
    std::printf("%d failure(s), %d skipped\n", failures(), skipped);
    return failures() ? 1 : 0;
}
