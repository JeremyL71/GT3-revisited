#pragma once

#include "disc/volume.h"

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gt3r {

class IniDocument;

// A mod is a directory under <user data>/mods/<id>/ containing:
//   mod.ini   [mod] name, version, author, description, priority
//   files/    files mirroring GT3.VOL paths; each overrides or adds a file
//   cars/     new car definitions (*.ini, see game/data/mod_cars.h)
// See docs/MODDING.md.
struct ModInfo {
    std::string id; // directory name
    std::string name;
    std::string version;
    std::string author;
    std::string description;
    int priority = 0; // higher wins
    bool enabled = true;
    std::string root; // absolute path to the mod directory
    std::vector<std::string> files; // VOL-relative paths provided
    std::vector<std::string> cars;  // host paths of cars/*.ini (game/data/mod_cars.h)
    std::vector<std::string> makers; // host paths of makers/*.ini
};

// Layered read-only filesystem: enabled mods (by priority) over GT3.VOL.
// All engine asset loads go through here, so every asset is moddable.
class Vfs {
public:
    explicit Vfs(Volume* base);

    // Scans `mods_dir` for mods; enable state comes from `[mods]` in `config`.
    void scan_mods(const std::string& mods_dir, const IniDocument* config);
    const std::vector<ModInfo>& mods() const { return mods_; }
    void set_mod_enabled(const std::string& id, bool enabled);

    bool exists(const std::string& path) const;
    std::optional<Bytes> read(const std::string& path) const;

    // Random-access stream over a file, for large assets (movies, music) that
    // should not be loaded whole. Disc entries stored uncompressed and mod
    // files are read in place; compressed disc entries are inflated once.
    struct Stream {
        u64 size = 0;
        std::function<void(u64 offset, void* out, size_t len)> read; // throws on error
    };
    std::optional<Stream> open_stream(const std::string& path) const;
    // "disc" or the providing mod's id; empty if missing.
    std::string source_of(const std::string& path) const;
    // Merged listing (mod-added files included). Dirs end with '/'.
    std::vector<std::string> list_dir(const std::string& dir) const;

    Volume* base() const { return base_; }

    // Virtual file listing the enabled mods' car definitions, one "<mod id>\t<host
    // path>" line each (ascending priority), so loaders that only get a reader
    // (game/data/mod_cars.h) still see mod cars.
    static constexpr const char* kModCarsList = "mods/.cars";
    // Virtual file: the disc's packed program (CORE.GT3, Volume::read_program), for loaders
    // that read a reference table from it. Not moddable.
    static constexpr const char* kProgramFile = "disc/.core.gt3";

private:
    void rebuild_index();
    Volume* base_;
    std::vector<ModInfo> mods_;
    std::map<std::string, std::string> overrides_; // vol path -> host path
    std::map<std::string, std::string> override_source_; // vol path -> mod id
};

std::string normalize_vfs_path(std::string_view path);

} // namespace gt3r
