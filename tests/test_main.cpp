// Test runner. Tests live in test_*.cpp and register with TEST().
// Set GT3R_DISC=/path/to/gt3.iso to also run tests that need game data.

#include "test_framework.h"

#include <csignal>
#include <cstdio>
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

namespace {

// The test running now, so a crash names it (a crashed process loses its buffered output).
const char* volatile g_current = nullptr;

extern "C" void on_crash(int sig) {
    std::signal(sig, SIG_DFL);
    const char* name = g_current ? g_current : "(between tests)";
    std::fputs("\nCRASH (signal) in test ", stderr);
    std::fputs(name, stderr);
    std::fputs("\n", stderr);
    std::fflush(stderr);
    std::raise(sig);
}

} // namespace

int main(int argc, char** argv) {
    using namespace gt3r::test;
    const char* filter = argc > 1 ? argv[1] : nullptr; // substring filter
    // A line per test even into a pipe (CI). MSVC has no line buffering (_IOLBF means full
    // buffering there, and a size of 0 aborts), so Windows writes unbuffered.
#ifdef _WIN32
    std::setvbuf(stdout, nullptr, _IONBF, 0);
#else
    std::setvbuf(stdout, nullptr, _IOLBF, BUFSIZ);
#endif
    for (int sig : {SIGSEGV, SIGABRT, SIGFPE, SIGILL}) std::signal(sig, on_crash);
    int skipped = 0;
    for (const TestCase& t : registry()) {
        if (filter && !std::strstr(t.name, filter)) continue;
        g_current = t.name;
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
