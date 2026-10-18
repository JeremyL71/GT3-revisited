#include "base/log.h"

#include <SDL3/SDL_log.h>

#include <cstdio>

namespace gt3r {

static LogLevel g_level = LogLevel::Info;

void log_set_level(LogLevel level) { g_level = level; }

void log_message(LogLevel level, const char* fmt, ...) {
    if (level < g_level) return;
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // SDL_Log routes to logcat on Android and stderr/console elsewhere.
    SDL_LogPriority prio = SDL_LOG_PRIORITY_INFO;
    switch (level) {
    case LogLevel::Debug: prio = SDL_LOG_PRIORITY_DEBUG; break;
    case LogLevel::Info: prio = SDL_LOG_PRIORITY_INFO; break;
    case LogLevel::Warn: prio = SDL_LOG_PRIORITY_WARN; break;
    case LogLevel::Error: prio = SDL_LOG_PRIORITY_ERROR; break;
    }
    SDL_LogMessage(SDL_LOG_CATEGORY_APPLICATION, prio, "%s", buf);
}

} // namespace gt3r
