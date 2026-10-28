// Test runner. Tests live in test_*.cpp and register with TEST().
// Set GT3R_DISC=/path/to/gt3.iso to also run tests that need game data.
// Every run writes in its own folder (temp_root), so runs can go at once.

#include "test_framework.h"

#include <SDL3/SDL_stdinc.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <system_error>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

namespace fs = std::filesystem;

namespace {

fs::path g_root;
char g_root_text[1024]; // for the crash message (no allocation in a signal handler)

// Into the C runtime's environment (std::getenv) and SDL's copy of it (SDL_getenv).
void put_env(const char* name, const std::string& value) {
#ifdef _WIN32
    _putenv_s(name, value.c_str());
#else
    setenv(name, value.c_str(), 1);
#endif
    SDL_SetEnvironmentVariable(SDL_GetEnvironment(), name, value.c_str(), true);
}

// The run's folder in the system's temporary folder, a name no other run has; the variables that
// name temporary and user data folders point into it from here on (the code reads them when it
// needs a folder, so this holds for the whole run).
void make_temp_root() {
    if (!g_root.empty()) return;
    std::error_code ec;
    const fs::path base = fs::temp_directory_path(ec);
    if (ec) {
        std::fprintf(stderr, "no temporary folder: %s\n", ec.message().c_str());
        std::exit(2);
    }
#ifdef _WIN32
    const long pid = long(_getpid());
#else
    const long pid = long(getpid());
#endif
    std::random_device rd;
    std::mt19937 mix(rd() ^ unsigned(std::chrono::steady_clock::now().time_since_epoch().count()));
    for (int attempt = 0; attempt < 100 && g_root.empty(); ++attempt) {
        char name[64];
        std::snprintf(name, sizeof name, "gt3r-tests-%ld-%08x", pid, unsigned(mix()));
        if (fs::create_directory(base / name, ec) && !ec) g_root = base / name;
    }
    if (g_root.empty()) {
        std::fprintf(stderr, "cannot make a test folder in %s\n", base.string().c_str());
        std::exit(2);
    }
    std::snprintf(g_root_text, sizeof g_root_text, "%s", g_root.string().c_str());
    const std::string root = g_root.string();
    for (const char* v : {"TMPDIR", "TMP", "TEMP"}) put_env(v, root);
    fs::create_directory(g_root / "user-data", ec);
    put_env("GT3R_DATA_DIR", (g_root / "user-data").string());
}

// Removes the run's folder unless GT3R_KEEP_TEST_TMP is set.
void remove_temp_root() {
    if (g_root.empty()) return;
    if (const char* keep = std::getenv("GT3R_KEEP_TEST_TMP"); keep && *keep && std::strcmp(keep, "0") != 0) {
        std::printf("test files kept in %s\n", g_root_text);
        return;
    }
    std::error_code ec;
    fs::remove_all(g_root, ec);
    if (ec) std::printf("  cannot remove %s: %s\n", g_root_text, ec.message().c_str());
}

} // namespace

namespace gt3r::test {

const fs::path& temp_root() {
    make_temp_root();
    return g_root;
}

fs::path temp_path(const std::string& name) {
    fs::path p = temp_root() / name;
    std::error_code ec;
    fs::remove_all(p, ec);
    return p;
}

fs::path temp_dir(const std::string& name) {
    fs::path p = temp_path(name);
    fs::create_directories(p);
    return p;
}

std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

int& failures() {
    static int n = 0;
    return n;
}

Disc* test_disc() {
    Disc* d = any_region_disc();
    if (d && d->info().region != Region::America) {
        static bool told = false;
        if (!told) std::printf("  GT3R_DISC is a %s disc: the NTSC-U disc tests skip\n", region_name(d->info().region));
        told = true;
        return nullptr;
    }
    return d;
}

Disc* any_region_disc() {
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
    if (g_root_text[0]) {
        std::fputs("test files kept in ", stderr);
        std::fputs(g_root_text, stderr);
        std::fputs("\n", stderr);
    }
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
    make_temp_root();
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
    remove_temp_root();
    return failures() ? 1 : 0;
}
