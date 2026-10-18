#pragma once

#include <cstdarg>

namespace gt3r {

enum class LogLevel { Debug, Info, Warn, Error };

void log_set_level(LogLevel level);
void log_message(LogLevel level, const char* fmt, ...)
#if defined(__MINGW32__) // MinGW's libstdc++ uses its C99 printf: %zu is fine
    __attribute__((format(gnu_printf, 2, 3)))
#elif defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

} // namespace gt3r

#define LOG_DEBUG(...) ::gt3r::log_message(::gt3r::LogLevel::Debug, __VA_ARGS__)
#define LOG_INFO(...) ::gt3r::log_message(::gt3r::LogLevel::Info, __VA_ARGS__)
#define LOG_WARN(...) ::gt3r::log_message(::gt3r::LogLevel::Warn, __VA_ARGS__)
#define LOG_ERROR(...) ::gt3r::log_message(::gt3r::LogLevel::Error, __VA_ARGS__)
