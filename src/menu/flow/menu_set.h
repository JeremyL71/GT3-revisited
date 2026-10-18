#pragma once

// A menu set: one .mbl flow file plus the per-language .pmb layouts its pages
// live in, loaded on demand like the original page manager does.
//
//   menu/<name>.mbl                      language independent flow
//   menu/<LANG>/<name><N>.pmb            layouts; N is the page's pmb number
//
// Every .mbl page names the pmb file it is drawn from (MblPage::unk40, the
// "pmb number"); a page with an import page (MblPage::str44, e.g. the GT-mode
// status bar "00cmn/status") draws that page, which lives in the import page's
// own pmb, together with its own. See docs/formats/menu.md.

#include "formats/menu/mbl.h"
#include "menu/menu_scene.h"

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace gt3r {
class Vfs;
}

namespace gt3r::menu::flow {

// Reads a file by VOL path; returns nullopt when missing. Lets tests feed
// synthetic files without a disc.
using FileReader = std::function<std::optional<Bytes>(const std::string& path)>;
FileReader vfs_reader(const Vfs& vfs);

// Semantics of the .mbl fields (see docs/formats/menu.md).
inline int pmb_number(const MblPage& p) { return s16(p.unk40); }
inline const std::string& import_page(const MblPage& p) { return p.str44; }
inline int item_kind(const MblItem& it) { return it.unk142 & 0x0F; }
inline bool item_is_multi(const MblItem& it) { return item_kind(it) == 4 && !it.children.empty(); }

class MenuSet;

// Content patches applied as a set loads: game code (mods) edits the flow and the
// layouts in memory, e.g. GT Mode adds mod cars and makers to the dealers. `read` is
// the set's file reader.
struct MenuPatches {
    std::function<void(const std::string& set, const FileReader& read, MblFile& flow)> flow;
    std::function<void(const std::string& set, int pmb, const FileReader& read, MenuScene& scene)> layout;
};
MenuPatches& menu_patches();

class MenuSet {
public:
    // `name` is the short name ("arcade" for menu/arcade.mbl), `lang` a folder
    // under menu/ ("US", "FR", ...). Throws FormatError if the .mbl is missing
    // or malformed; pmb files are loaded lazily.
    MenuSet(FileReader reader, std::string name, std::string lang);

    const std::string& name() const { return name_; }
    const std::string& lang() const { return lang_; }
    const MblFile& flow() const { return mbl_; }

    // Path of the layout file holding pages with pmb number `n`.
    std::string pmb_path(int n) const;

    struct PageRef {
        const MblPage* flow = nullptr; // nullptr if the page is not in the .mbl
        const MenuScene* scene = nullptr;
        int page = -1; // index in scene->file().pages
        explicit operator bool() const { return scene && page >= 0; }
        const Page& layout() const { return scene->file().pages[size_t(page)]; }
    };

    // Resolves a page by name: its .mbl entry and its layout. The layout is
    // looked up in the page's own pmb; pages that are not in the .mbl (import
    // pages are, in shipped data) fall back to pmb 1. Empty ref if missing.
    PageRef page(std::string_view name) const;
    // Loads (and caches) pmb number n; nullptr if it cannot be read/parsed.
    const MenuScene* scene(int n) const;

private:
    FileReader reader_;
    std::string name_, lang_;
    MblFile mbl_;
    mutable std::map<int, std::unique_ptr<MenuScene>> scenes_; // nullptr = failed
};

} // namespace gt3r::menu::flow
