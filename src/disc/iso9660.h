#pragma once

#include "base/file.h"

#include <string>
#include <vector>

namespace gt3r {

constexpr u64 kSectorSize = 2048;

// Read-only ISO9660 reader for PS2 DVD images (2048-byte user-data sectors).
// PS2 discs are ISO9660 + UDF bridge; the ISO9660 view is sufficient.
class Iso9660 {
public:
    struct Entry {
        std::string name; // without ";1" version suffix
        u32 lba = 0;
        u64 size = 0;
        bool is_dir = false;
    };

    explicit Iso9660(File& file);

    const std::string& volume_id() const { return volume_id_; }
    // Lists a directory. `path` uses '/' separators; "" is the root.
    std::vector<Entry> list(const std::string& path) const;
    // Finds a file by path (case-insensitive, as PS2 paths are upper case).
    std::optional<Entry> find(const std::string& path) const;

    Bytes read(const Entry& e) const;

private:
    std::vector<Entry> read_dir(u32 lba, u32 size) const;
    File& file_;
    std::string volume_id_;
    Entry root_;
};

} // namespace gt3r
