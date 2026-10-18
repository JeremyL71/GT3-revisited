#pragma once

// Remembered disc images, [game] in config.ini:
//
//   disc    = /path/to/current.iso   ; opened at startup
//   recent1 = /path/to/current.iso   ; most recently opened first
//   recent2 = D:\Games\GT3 (PAL).iso
//   ...                              ; up to recent5
//
// One key per path, so paths may contain spaces, commas or '='. Paths are
// stored absolute (UTF-8); URIs such as Android's content:// are kept as
// given. Pure logic (no SDL, no file system access) so it can be unit-tested
// on every platform with Windows- and POSIX-style paths alike.

#include "base/config.h"

#include <string>
#include <string_view>
#include <vector>

namespace gt3r::platform {

constexpr int kMaxRecentDiscs = 5;

// True for "/x", "C:\x", "C:/x", "\\server\share" and "scheme://..." paths.
bool is_absolute_disc_path(std::string_view path);

// Same file as far as a path comparison can tell: separators are unified,
// and Windows drive / UNC paths compare case-insensitively.
bool same_disc_path(std::string_view a, std::string_view b);

// [game] recent1..recent5 (empty values and duplicates dropped).
std::vector<std::string> load_recent_discs(const IniDocument& cfg);
// Rewrites [game] recent1..recentN; removes keys past the list's end.
void save_recent_discs(IniDocument& cfg, const std::vector<std::string>& list);

// `path` moved (or added) to the front, without duplicates, capped at kMaxRecentDiscs.
std::vector<std::string> push_recent_disc(std::vector<std::string> list, const std::string& path);
// `list` without `path`.
std::vector<std::string> remove_recent_disc(std::vector<std::string> list, std::string_view path);

// Where to look for a saved disc path, in order. Absolute paths and URIs are
// returned as they are. A relative path (old configs) is tried as given
// (i.e. relative to the working directory, resolved by the caller) and then
// relative to `base_dir` (the executable's directory, SDL_GetBasePath()).
std::vector<std::string> disc_path_candidates(const std::string& saved, const std::string& base_dir);

// File name part of a path ("GT3.iso" for "C:\Games\GT3.iso").
std::string disc_file_name(std::string_view path);

} // namespace gt3r::platform
