#include "menu/flow/theater_page.h"

#include <algorithm>
#include <limits>

namespace gt3r::menu::flow {

namespace {

// The icon grid inside the page's icon view (568 x 230 at (36, 158)): 64 x 64 icons at a
// 70 px pitch, 7 px in from the top-left, the fourth row cut by the view's bottom
// (measured on the original's DEMONSTRATION page).
constexpr float kPitch = 70, kInset = 7;

const Element* find_any(const Page& p, const std::string& name) {
    for (const Layer& l : p.layers) {
        for (const Element& e : l.elements)
            if (e.name == name) return &e;
        for (const Element& e : l.widgets)
            if (e.name == name) return &e;
    }
    return nullptr;
}

TheaterBackend& backend_ref() {
    static TheaterBackend b;
    return b;
}

} // namespace

void set_theater_backend(TheaterBackend b) { backend_ref() = std::move(b); }
const TheaterBackend& theater_backend() { return backend_ref(); }

void TheaterPage::open(Mode m, std::vector<TheaterEntry> entries, int cursor) {
    *this = TheaterPage{};
    mode_ = m;
    entries_ = std::move(entries);
    cursor_ = std::clamp(cursor, 0, std::max(0, int(entries_.size()) - 1));
    top_ = std::max(0, cursor_ / kCols - (kRows - 1));
    prev_ = std::numeric_limits<MenuKeys>::max(); // keys held when the page opens do not count
}

int TheaterPage::tick(MenuKeys held, std::vector<std::string>& sounds) {
    const MenuKeys pressed = held & MenuKeys(~prev_);
    // Directions repeat like the menus (the press, 31 frames later, then every 4 frames).
    const MenuKeys dirs = key_bit(MenuKey::Up) | key_bit(MenuKey::Down) | key_bit(MenuKey::Left) | key_bit(MenuKey::Right);
    MenuKeys fired = pressed;
    if (held & dirs) {
        if (pressed & dirs) repeat_ = 0;
        else if (++repeat_ >= 31 && (repeat_ - 31) % 4 == 0) fired |= held & dirs;
    }
    prev_ = held;
    ++frames_;
    auto key = [&](MenuKey k) { return (fired & key_bit(k)) != 0; };
    if (confirm_) {
        if (key(MenuKey::Left) || key(MenuKey::Right)) {
            int c = key(MenuKey::Left) ? 0 : 1;
            if (c != choice_) choice_ = c, sounds.push_back("cursor");
        } else if (key(MenuKey::Cancel) || (key(MenuKey::Confirm) && choice_ == 1)) {
            sounds.push_back("cancel");
            confirm_ = false;
        } else if (key(MenuKey::Confirm)) {
            const TheaterBackend& b = theater_backend();
            bool ok = b.remove && b.remove(entries_[size_t(cursor_)]);
            sounds.push_back(ok ? "ok" : "disable");
            if (ok) {
                entries_.erase(entries_.begin() + cursor_);
                cursor_ = std::clamp(cursor_, 0, std::max(0, int(entries_.size()) - 1));
                top_ = std::clamp(top_, 0, std::max(0, (int(entries_.size()) + kCols - 1) / kCols - kRows));
            }
            confirm_ = false;
        }
        return 0;
    }
    const int n = int(entries_.size());
    int dc = key(MenuKey::Right) ? 1 : key(MenuKey::Left) ? -1 : 0;
    int dr = key(MenuKey::Down) ? 1 : key(MenuKey::Up) ? -1 : 0;
    if ((dc || dr) && n > 0) {
        int col = cursor_ % kCols + dc, row = cursor_ / kCols + dr;
        const int rows = (n + kCols - 1) / kCols;
        int c = cursor_;
        if (col >= 0 && col < kCols && row >= 0 && row < rows) c = std::min(row * kCols + col, n - 1);
        if (c != cursor_) {
            cursor_ = c;
            sounds.push_back("cursor");
            int r = cursor_ / kCols;
            if (r < top_) top_ = r;
            if (r >= top_ + kRows) top_ = r - kRows + 1;
        }
    } else if (key(MenuKey::Confirm)) {
        if (n == 0 || !entries_[size_t(cursor_)].playable) {
            sounds.push_back("disable");
        } else {
            sounds.push_back("ok");
            return 1;
        }
    } else if (key(MenuKey::Start)) {
        if (n > 0 && entries_[size_t(cursor_)].deletable) {
            sounds.push_back("ok");
            confirm_ = true;
            choice_ = 1; // CANCEL first: deleting is the deliberate choice
        } else {
            sounds.push_back("disable");
        }
    } else if (key(MenuKey::Cancel)) {
        sounds.push_back("cancel");
        return -1;
    }
    return 0;
}

void TheaterPage::draw(const MenuSet::PageRef& ref, const MenuSet::PageRef& confirm, std::vector<ElementDraw>& out) const {
    if (!ref) return;
    const Page& p = ref.layout();
    float z = 1000;
    auto put = [&](const MenuSet::PageRef& r, const Element* e, float dx = 0, float dy = 0, const SDL_FRect* box = nullptr,
                   const std::string* text = nullptr) -> ElementDraw* {
        if (!e) return nullptr;
        ElementDraw d;
        d.scene = r.scene;
        d.element = e;
        d.widget = e->widget_class.has_value();
        d.dx = dx, d.dy = dy;
        d.z = z += 1;
        if (box) d.has_box = true, d.box = *box;
        if (text) d.text = std::make_shared<const std::string>(*text);
        out.push_back(d);
        return &out.back();
    };
    auto el = [&](const std::string& n) { return find_any(p, n); };
    // Titles: the hidden title moved onto the title base, and the panel's title bar.
    const std::string title = mode_ == Mode::LoadReplay ? "Load replay" : mode_ == Mode::LoadTrack ? "Load track" : "";
    if (const Element* t = el("title_load_replay"))
        if (const Element* base = el("title_base")) put(ref, t, base->x - t->x, base->y - t->y, nullptr, title.empty() ? nullptr : &title);
    if (mode_ != Mode::Demonstration) {
        // The panel's title bar names the list (the layout's own bar says DEMONSTRATION).
        const std::string bar = "REPLAY";
        for (ElementDraw& d : out)
            if (d.element && d.element->name == "title_bar") d.text = std::make_shared<const std::string>(bar);
    }
    // The icon grid, clipped to the view (the row below the third shows its top).
    const Element* view = el("icon_view_slot1");
    SDL_FRect cur{};
    bool have_cur = false;
    if (view) {
        const SDL_FRect clip{view->x, view->y, view->w, view->h};
        for (int v = 0; v < kCols * (kRows + 1); ++v) {
            const int i = v + top_ * kCols;
            float x = view->x + kInset + float(v % kCols) * kPitch;
            float y = view->y + view->h - kInset - 64 - float(v / kCols) * kPitch;
            SDL_FRect box{x, y, 64, 64};
            const TheaterEntry* f = i < int(entries_.size()) ? &entries_[size_t(i)] : nullptr;
            const char* icon = !f                                    ? "icon_blank"
                               : f->icon == TheaterEntry::Icon::License ? "icon_license"
                               : f->icon == TheaterEntry::Icon::Best    ? "icon_best_replay"
                                                                        : "icon_replay";
            if (ElementDraw* d = put(ref, el(icon), 0, 0, &box)) d->has_clip = true, d->clip = clip;
            if (f && i == cursor_) cur = box, have_cur = true;
        }
        if (have_cur) {
            // The cursor: a white quad added over the focused icon, pulsing (the original's
            // DEMONSTRATION page shows the icon clearly whitened; level M).
            if (ElementDraw* d = put(ref, el("icon_cursor"), 0, 0, &cur)) {
                d->additive = true;
                d->alpha = 0.3f + 0.25f * (1.0f - float(frames_ % 31) / 30.0f);
            }
        }
    }
    // The focused file's info panel.
    if (const TheaterEntry* f = chosen()) {
        const bool best = f->icon != TheaterEntry::Icon::Replay;
        const std::string layer = best ? "info_best_replay" : "info_replay";
        const std::string prefix = best ? "b_pt_" : "r_pt_";
        for (const Layer& l : p.layers)
            if (l.name == layer) {
                for (const Element& e : l.elements) put(ref, &e);
                for (const Element& e : l.widgets) {
                    std::string t = e.name.rfind(prefix, 0) == 0 ? f->text(e.name.substr(prefix.size())) : std::string();
                    if (!t.empty()) put(ref, &e, 0, 0, nullptr, &t);
                }
            }
    }
    // Delete confirm: the file page's confirm window, "Delete OK?", OK / CANCEL.
    if (confirm_ && confirm) {
        const Page& cp = confirm.layout();
        auto cel = [&](const std::string& n) { return find_any(cp, n); };
        for (const Layer& l : cp.layers)
            if (l.name == "confirm") {
                for (const Element& e : l.elements) put(confirm, &e);
            }
        put(confirm, cel("confirm_text_delete"));
        put(confirm, cel(std::string("confirm_m_ok") + (choice_ == 0 ? "" : "_sel")));
        put(confirm, cel(std::string("confirm_m_cancel") + (choice_ == 1 ? "" : "_sel")));
        if (const Element* face = cel("confirm_icon_face"))
            if (chosen()) {
                SDL_FRect fb{face->x + (face->w - 64) * 0.5f, face->y + (face->h - 64) * 0.5f, 64, 64};
                put(ref, el("icon_replay"), 0, 0, &fb);
            }
    }
}

} // namespace gt3r::menu::flow
