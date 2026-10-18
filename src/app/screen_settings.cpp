// Settings: every user-facing config.ini option on tabbed pages (Display,
// Audio, Controls, Force feedback, Touch, Game disc, Online). Changes apply at once
// where the engine can (volume, vsync, MSAA, 3D resolution, bindings, force
// feedback, touch layout) and are saved to config.ini immediately. Only keys
// the engine reads are offered; see README "Settings" for the key list.

#include "app/app.h"
#include "app/screen_online.h"
#include "app/screen_registry.h"
#include "app/updates.h"

#include "base/file.h"
#include "menu/flow/menu_sound.h"
#include "net/session.h"
#include "platform/recent_discs.h"
#include "platform/render_scale.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace gt3r {

namespace {

enum Tab { kDisplay, kAudio, kControls, kFfb, kTouch, kDisc, kOnline, kTabCount };
const char* const kTabNames[kTabCount] = {"Display", "Audio", "Controls", "Force feedback", "Touch", "Game disc", "Online"};

struct Row {
    enum class Kind { Toggle, Choice, Slider, Action, Info, Heading, Preview };
    Kind kind = Kind::Info;
    std::string label, desc, value;
    bool on = false;       // Toggle
    float frac = 0;        // Slider position 0..1
    bool enabled = true;   // false: shown greyed, changes ignored
    bool dim = false;      // greyed but still adjustable (touch settings on desktop)
    bool restart = false;  // takes effect after a restart (and differs from the running state)
    std::function<void(int)> change;     // Left/Right
    std::function<void(float)> set_frac; // slider pointer position 0..1
    std::function<void()> activate;      // Cross / tap
    std::function<void()> remove;        // Square (recent discs)

    bool selectable() const { return kind != Kind::Info && kind != Kind::Heading && kind != Kind::Preview; }
};

std::string percent(float v) { return std::to_string(int(std::lround(v * 100))) + "%"; }

float snap(float v, float lo, float hi, float step) {
    v = std::clamp(v, lo, hi);
    return std::clamp(lo + std::round((v - lo) / step) * step, lo, hi);
}

bool disc_available(const std::string& path) {
    return path.find("://") != std::string::npos || host_path_exists(path);
}

class SettingsScreen : public Screen {
public:
    explicit SettingsScreen(int tab) : tab_(std::clamp(tab, 0, kTabCount - 1)) {
        // Screenshots / testing: GT3R_SETTINGS_TAB=<0..6> opens that page.
        if (const char* t = SDL_getenv("GT3R_SETTINGS_TAB"); t && *t) tab_ = std::clamp(std::atoi(t), 0, kTabCount - 1);
    }

    void on_event(App& app, const SDL_Event& e) override {
        build(app);
        if (e.type == SDL_EVENT_KEY_DOWN && e.key.scancode == SDL_SCANCODE_TAB && !e.key.repeat) {
            switch_tab(app, (e.key.mod & SDL_KMOD_SHIFT) ? -1 : 1);
            return;
        }
        if (e.type == SDL_EVENT_MOUSE_WHEEL) {
            float dy = e.wheel.y * (e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -1.0f : 1.0f);
            scroll_ -= dy * lay_.row_h;
            follow_sel_ = false;
            return;
        }
        switch (gesture_.feed(e)) {
        case Gesture::Result::Down: {
            const float x = gesture_.x(), y = gesture_.y();
            press_ = Press{};
            if (lay_.back.contains(x, y)) press_.back = true;
            else if (int t = tab_at(x, y); t >= 0) press_.tab = t;
            else if (lay_.list.contains(x, y)) {
                press_.in_list = true;
                press_.scroll = scroll_;
                int i = row_at(y);
                if (i >= 0 && rows_[i].selectable()) {
                    press_.row = i;
                    Row& r = rows_[i];
                    // Sliders follow the finger straight away.
                    if (r.kind == Row::Kind::Slider && r.enabled && r.set_frac && x >= lay_.val_x - 12) {
                        sel_ = i;
                        focus_tabs_ = false;
                        dragging_ = i;
                        r.set_frac((x - lay_.val_x) / lay_.slider_w);
                    }
                }
            }
            break;
        }
        case Gesture::Result::DragStart:
        case Gesture::Result::Drag:
            if (dragging_ >= 0 && dragging_ < int(rows_.size()) && rows_[dragging_].set_frac && rows_[dragging_].enabled)
                rows_[dragging_].set_frac((gesture_.x() - lay_.val_x) / lay_.slider_w);
            else if (press_.in_list) {
                scroll_ = press_.scroll - (gesture_.y() - gesture_.start_y());
                follow_sel_ = false;
                press_.row = -1;
            } else {
                press_ = Press{};
            }
            break;
        case Gesture::Result::Tap: {
            Press p = press_;
            press_ = Press{};
            bool was_slider = dragging_ >= 0;
            dragging_ = -1;
            if (p.back && lay_.back.contains(gesture_.x(), gesture_.y())) {
                leave(app);
                app.pop();
                return;
            }
            if (p.tab >= 0) {
                if (p.tab != tab_) switch_tab(app, p.tab - tab_);
                focus_tabs_ = false;
                return;
            }
            if (p.row < 0 || p.row >= int(rows_.size()) || was_slider) return;
            sel_ = p.row;
            focus_tabs_ = false;
            Row& r = rows_[p.row];
            if (!r.enabled) return;
            const float x = gesture_.start_x();
            const bool on_value = x >= lay_.val_x - 12;
            switch (r.kind) {
            case Row::Kind::Slider:
                if (on_value && r.set_frac) r.set_frac((x - lay_.val_x) / lay_.slider_w);
                break;
            case Row::Kind::Choice:
                if (on_value && r.change) r.change(x < lay_.val_x + lay_.val_w / 2 ? -1 : 1);
                else if (r.activate) r.activate();
                break;
            case Row::Kind::Toggle:
            case Row::Kind::Action:
                if (r.activate) r.activate();
                break;
            default: break;
            }
            break;
        }
        case Gesture::Result::Up:
            press_ = Press{};
            dragging_ = -1;
            break;
        case Gesture::Result::None: break;
        }
    }

    void update(App& app, double) override {
        build(app);
        const MenuInput& m = app.menu();
        if (m.pressed(PadButton::Triangle)) {
            leave(app);
            app.pop();
            return;
        }
        if (m.pressed(PadButton::L1)) switch_tab(app, -1);
        if (m.pressed(PadButton::R1)) switch_tab(app, 1);

        if (focus_tabs_) {
            if (int h = m.horizontal()) switch_tab(app, h);
            if (m.vertical() > 0 || m.pressed(PadButton::Cross)) {
                focus_tabs_ = false;
                sel_ = first_selectable();
                follow_sel_ = true;
            }
            return;
        }
        if (int v = m.vertical()) {
            move(v);
            follow_sel_ = true;
        }
        if (sel_ < 0 || sel_ >= int(rows_.size())) return;
        Row& r = rows_[sel_];
        if (!r.enabled) return;
        if (int h = m.horizontal(); h && r.change) r.change(h);
        if (m.pressed(PadButton::Cross) && r.activate) r.activate();
        if (m.pressed(PadButton::Square) && r.remove) r.remove();
    }

    void render(App& app, Ui& ui) override {
        compute_layout(ui);
        const UiRect& S = ui.safe();
        const bool touch = ui.touch();

        // Top bar: back button, title, keyboard hint.
        ui.fill(0, 0, ui.width(), lay_.bar_bottom, colors::kPanel);
        ui.button(lay_.back, "< Back", false, press_.back && gesture_.active());
        ui.text(lay_.back.right() + 12, std::floor(lay_.back.y + (lay_.back.h - Ui::kChar) / 2), "SETTINGS", colors::kAccent);
        if (!touch)
            ui.text_right(S.right() - 8, std::floor(lay_.back.y + (lay_.back.h - Ui::kChar) / 2), "L1/R1 or Tab: section",
                          colors::kDim);

        // Tab bar.
        for (int t = 0; t < kTabCount; ++t)
            ui.tab(lay_.tabs[t].x, lay_.tabs[t].y, lay_.tabs[t].h, tab_name(t), t == tab_, focus_tabs_ && t == tab_);
        ui.fill(0, lay_.tab_bottom, ui.width(), 1, colors::kOutline);

        // Rows (scrollable).
        const int n = int(rows_.size());
        const UiRect& L = lay_.list;
        const float content = n ? row_top(n) : 0;
        if (follow_sel_ && sel_ >= 0 && sel_ < n && !focus_tabs_) {
            float top = row_top(sel_), bot = top + row_h(sel_);
            // Keep a heading visible with its first row.
            if (sel_ > 0 && rows_[sel_ - 1].kind == Row::Kind::Heading) top = row_top(sel_ - 1);
            if (top < scroll_) scroll_ = top;
            if (bot > scroll_ + L.h) scroll_ = bot - L.h;
        }
        scroll_ = std::clamp(scroll_, 0.0f, std::max(0.0f, content - L.h));
        if (focus_tabs_) scroll_ = 0;
        SDL_Rect clip{int(L.x), int(L.y), int(std::ceil(L.w)), int(L.h)};
        SDL_SetRenderClipRect(ui.renderer(), &clip);
        for (int i = 0; i < n; ++i) {
            float y = L.y + row_top(i) - scroll_;
            if (y + row_h(i) < L.y || y > L.bottom()) continue;
            draw_row(app, ui, rows_[i], y, row_h(i), i == sel_ && !focus_tabs_, i == press_.row && gesture_.active());
        }
        SDL_SetRenderClipRect(ui.renderer(), nullptr);
        ui.scrollbar({L.x, L.y, S.right() - L.x - 1, L.h}, content, scroll_);

        // Description of the highlighted option.
        const float dy = lay_.desc_y;
        ui.fill(0, dy, ui.width(), ui.height() - dy, Color{22, 24, 31});
        ui.fill(0, dy, ui.width(), 1, colors::kOutline);
        std::string desc = focus_tabs_ ? "Left/Right: switch section    Down: options"
                         : sel_ >= 0 && sel_ < n ? rows_[sel_].desc
                                                 : "";
        std::vector<std::string> lines = Ui::wrap(desc, S.w - 16);
        if (lines.size() > 2) {
            for (size_t k = 2; k < lines.size(); ++k) lines[1] += " " + lines[k];
            lines.resize(2);
            lines[1] = Ui::clip(lines[1], S.w - 16);
        }
        for (size_t i = 0; i < lines.size(); ++i) ui.text(S.x + 8, dy + 4 + float(i) * 11, lines[i], colors::kText);
        if (!touch) ui.footer(footer_hint());
    }

    TouchLayout touch_layout() const override { return TouchLayout::None; } // tap and drag work directly
    bool adaptive_ui() const override { return true; }

private:
    // Layout, recomputed each frame from the Ui's size (pointer handling uses
    // the last frame's).
    struct Layout {
        UiRect back, list;
        UiRect tabs[kTabCount];
        float bar_bottom = 20, tab_bottom = 40, desc_y = 318;
        float row_h = 18, heading_h = 18, preview_h = 110;
        float label_x = 24, val_x = 330, val_w = 288, slider_w = 200;
        bool short_names = false;
    };
    struct Press {
        bool back = false, in_list = false;
        int tab = -1, row = -1;
        float scroll = 0;
    };

    const char* tab_name(int t) const { return lay_.short_names && t == kFfb ? "Feedback" : kTabNames[t]; }

    void compute_layout(const Ui& ui) {
        const UiRect& S = ui.safe();
        const bool touch = ui.touch();
        Layout& l = lay_;
        const float bar_h = touch ? 30.0f : 20.0f;
        l.bar_bottom = S.y + bar_h;
        l.back = {S.x + 6, S.y + (touch ? 3 : 2), touch ? 72.0f : 64.0f, bar_h - (touch ? 6 : 4)};
        const float th = touch ? 28.0f : 16.0f, ty = l.bar_bottom + 4;
        float total = 0;
        l.short_names = false;
        for (int pass = 0; pass < 2; ++pass) {
            total = 0;
            for (int t = 0; t < kTabCount; ++t) total += ui.tab_width(tab_name(t)) + 4;
            if (total <= S.w - 8) break;
            l.short_names = true;
        }
        float tx = S.x + 6;
        for (int t = 0; t < kTabCount; ++t) {
            float w = ui.tab_width(tab_name(t));
            l.tabs[t] = {tx, ty, w, th};
            tx += w + 4;
        }
        l.tab_bottom = ty + th + 2;
        const float desc_h = 26, footer_h = touch ? 0.0f : 16.0f;
        l.desc_y = ui.height() - footer_h - desc_h - (touch ? ui.height() - S.bottom() : 0);
        l.list = {S.x, l.tab_bottom + 3, S.w, l.desc_y - l.tab_bottom - 4};
        l.row_h = touch ? 32.0f : 18.0f;
        l.heading_h = touch ? 26.0f : 18.0f;
        l.preview_h = touch ? 104.0f : 96.0f;
        l.label_x = S.x + (touch ? 18 : 16);
        l.val_x = std::floor(S.x + S.w * (touch ? 0.5f : 0.5f));
        l.val_w = S.right() - 16 - l.val_x;
        l.slider_w = std::max(60.0f, l.val_w - (touch ? 64 : 88));
        if (touch) l.val_x += 8, l.slider_w -= 8, l.val_w -= 8; // room for the slider knob
    }

    float row_h(int i) const {
        switch (rows_[i].kind) {
        case Row::Kind::Heading: return lay_.heading_h;
        case Row::Kind::Preview: return lay_.preview_h;
        default: return lay_.row_h;
        }
    }
    float row_top(int i) const { // offset of row i from the top of the list
        float y = 0;
        for (int k = 0; k < i && k < int(rows_.size()); ++k) y += row_h(k);
        return y;
    }
    int row_at(float y) const {
        float off = y - lay_.list.y + scroll_, top = 0;
        for (int i = 0; i < int(rows_.size()); ++i) {
            if (off >= top && off < top + row_h(i)) return i;
            top += row_h(i);
        }
        return -1;
    }
    int tab_at(float x, float y) const {
        for (int t = 0; t < kTabCount; ++t) {
            UiRect r = lay_.tabs[t];
            r.y -= 2, r.h += 4; // a little slack for fingers
            if (r.contains(x, y)) return t;
        }
        return -1;
    }

    // --- row builders ------------------------------------------------------

    Row& add(Row::Kind k, std::string label, std::string desc) {
        Row r;
        r.kind = k;
        r.label = std::move(label);
        r.desc = std::move(desc);
        rows_.push_back(std::move(r));
        return rows_.back();
    }

    void heading(std::string label) { add(Row::Kind::Heading, std::move(label), ""); }

    Row& info(std::string label, std::string value, std::string desc = "") {
        Row& r = add(Row::Kind::Info, std::move(label), std::move(desc));
        r.value = std::move(value);
        return r;
    }

    Row& toggle(std::string label, std::string desc, bool on, std::function<void(bool)> set) {
        Row& r = add(Row::Kind::Toggle, std::move(label), std::move(desc));
        r.on = on;
        r.change = [set, on](int) { set(!on); };
        r.activate = [set, on] { set(!on); };
        return r;
    }

    // `names` are shown; `idx` is the current one. Clamped (not wrapping).
    Row& choice(std::string label, std::string desc, std::vector<std::string> names, int idx,
                std::function<void(int)> set) {
        Row& r = add(Row::Kind::Choice, std::move(label), std::move(desc));
        int n = int(names.size());
        r.value = idx >= 0 && idx < n ? names[idx] : "?";
        r.change = [set, idx, n](int d) {
            int to = std::clamp(idx + d, 0, n - 1);
            if (to != idx) set(to);
        };
        r.activate = [set, idx, n] { set((idx + 1) % n); }; // Cross cycles
        return r;
    }

    Row& slider(std::string label, std::string desc, float v, float lo, float hi, float step, std::string text,
                std::function<void(float)> set) {
        Row& r = add(Row::Kind::Slider, std::move(label), std::move(desc));
        r.frac = (v - lo) / (hi - lo);
        r.value = std::move(text);
        r.change = [=](int d) {
            float to = snap(v + d * step, lo, hi, step);
            if (to != v) set(to);
        };
        r.set_frac = [=](float f) {
            float to = snap(lo + std::clamp(f, 0.0f, 1.0f) * (hi - lo), lo, hi, step);
            if (to != v) set(to);
        };
        return r;
    }

    Row& action(std::string label, std::string desc, std::string value, std::function<void()> fn) {
        Row& r = add(Row::Kind::Action, std::move(label), std::move(desc));
        r.value = std::move(value);
        r.activate = std::move(fn);
        return r;
    }

    // --- pages ---------------------------------------------------------------

    void build(App& app) {
        rows_.clear();
        switch (tab_) {
        case kDisplay: build_display(app); break;
        case kAudio: build_audio(app); break;
        case kControls: build_controls(app); break;
        case kFfb: build_ffb(app); break;
        case kTouch: build_touch(app); break;
        case kDisc: build_disc(app); break;
        case kOnline: build_online(app); break;
        default: break;
        }
        if (sel_ < 0 || sel_ >= int(rows_.size()) || !rows_[sel_].selectable()) sel_ = first_selectable();
    }

    void build_display(App& app) {
        App* a = &app;
        IniDocument& cfg = app.config();
        if (!platform::is_mobile())
            toggle("Fullscreen", "Fill the whole screen; off: a resizable window. F11 also toggles.", app.fullscreen(),
                   [a](bool v) { a->set_fullscreen(v); });
        else
            info("Fullscreen", "Always on this device");

        bool vsync = cfg.get("video", "vsync").value_or("1") != "0";
        toggle("VSync", "Wait for the display refresh: no tearing. Off: lower input latency.", vsync, [a](bool v) {
            a->config().set("video", "vsync", v ? "1" : "0");
            SDL_SetRenderVSync(a->renderer(), v ? 1 : 0);
            a->save_config();
        });

        static const int kMsaa[] = {1, 2, 4, 8};
        int want = std::atoi(cfg.get("video", "msaa").value_or("4").c_str());
        int idx = 0;
        for (int i = 0; i < 4; ++i)
            if (want >= kMsaa[i]) idx = i;
        std::vector<std::string> names = {"Off", "2x", "4x", "8x"};
        if (app.gpu() && app.gpu()->msaa() != kMsaa[idx])
            names[idx] += " (device: " + (app.gpu()->msaa() > 1 ? std::to_string(app.gpu()->msaa()) + "x)" : std::string("off)"));
        Row& msaa = choice("Anti-aliasing (MSAA)", "Smooths jagged edges of 3D models. Higher costs more GPU time.", names, idx,
                           [a](int i) {
                               a->config().set("video", "msaa", std::to_string(kMsaa[i]));
                               if (a->gpu()) a->gpu()->set_msaa(kMsaa[i]);
                               a->save_config();
                           });
        msaa.enabled = app.gpu() != nullptr;
        if (!msaa.enabled) msaa.desc = "Needs the 3D renderer (turn it on and restart).";

        static const char* const kScales[] = {"auto", "0.5", "0.6", "0.7", "0.75", "0.8", "0.9", "1.0"};
        std::optional<float> rs = platform::parse_render_scale(cfg.get("video", "render_scale").value_or("auto"));
        int sidx = 0;
        if (rs) {
            float best = 9;
            for (int i = 1; i < 8; ++i)
                if (float d = std::fabs(std::strtof(kScales[i], nullptr) - *rs); d < best) best = d, sidx = i;
        }
        std::vector<std::string> snames = {"Auto", "50%", "60%", "70%", "75%", "80%", "90%", "100%"};
        if (app.gpu()) snames[0] = "Auto (" + percent(app.gpu()->render_scale()) + ")";
        Row& scale = choice("3D resolution", "Resolution of 3D views vs. the screen. Lower is faster on phones.", snames,
                            sidx, [a](int i) {
                                a->config().set("video", "render_scale", kScales[i]);
                                if (render::GpuContext* g = a->gpu()) {
                                    std::optional<float> v = platform::parse_render_scale(kScales[i]);
                                    int pw = 0, ph = 0;
                                    SDL_GetWindowSizeInPixels(a->window(), &pw, &ph);
                                    g->set_render_scale(v ? *v : platform::auto_render_scale(pw, ph, platform::is_mobile()));
                                }
                                a->save_config();
                            });
        scale.enabled = app.gpu() != nullptr;
        if (!scale.enabled) scale.desc = msaa.desc;

        bool gpu_cfg = cfg.get("video", "gpu").value_or("1") != "0";
        Row& gpu = toggle("3D renderer (SDL_GPU)", "Off: 2D-only fallback for systems without a working GPU driver.",
                          gpu_cfg, [a](bool v) {
                              a->config().set("video", "gpu", v ? "1" : "0");
                              a->save_config();
                          });
        gpu.restart = gpu_cfg != (app.gpu() != nullptr);
        if (gpu.restart) gpu.desc = "Takes effect after restarting GT3 Revisited.";
        if (app.gpu()) info("Graphics driver", app.gpu()->driver_name());
        info("2D renderer", SDL_GetRendererName(app.renderer()));
    }

    void build_audio(App& app) {
        App* a = &app;
        AudioEngine& au = app.audio();
        bool disc = app.vfs() != nullptr;
        slider("Music volume",
               disc ? "Soundtrack and menu music. Changing it plays a preview track." : "Soundtrack and menu music.",
               au.music_volume(), 0, 1, 0.05f, percent(au.music_volume()), [this, a](float v) {
                   a->audio().set_music_volume(v);
                   start_preview(*a);
                   a->save_config();
               });
        slider("Sound effects", "Engines, tyres, crashes and menu sounds.", au.sfx_volume(), 0, 1, 0.05f,
               percent(au.sfx_volume()), [this, a](float v) {
                   a->audio().set_sfx_volume(v);
                   play_se(*a, "cursor");
                   a->save_config();
               });
        toggle("Surround cue", "GT3's rear-speaker phase trick for Pro Logic decoders. Off for stereo.",
               au.voices().surround_cue(), [a](bool v) {
                   a->audio().voices().set_surround_cue(v);
                   a->config().set("audio", "surround_cue", v ? "on" : "off");
                   a->save_config();
               });
        if (!au.ok()) info("Audio device", "Not available", "No audio output device was found.");
    }

    void build_controls(App& app) {
        App* a = &app;
        InputManager& in = app.input();
        const auto& presets = preset_names();
        for (int p = 0; p < InputManager::kMaxPlayers; ++p) {
            heading(p == 0 ? "Player 1" : "Player 2 (split screen)");
            std::vector<std::string> names(presets.begin(), presets.end());
            int idx = 0;
            for (size_t i = 0; i < names.size(); ++i)
                if (names[i] == in.preset(p)) idx = int(i);
            if (in.is_custom(p)) {
                names.insert(names.begin(), "custom bindings");
                idx = 0;
            }
            bool custom = in.is_custom(p);
            choice("Device preset",
                   custom ? "Custom bindings are in use. Choosing a preset replaces them."
                          : "Menu buttons. racing / classic: DS2 layout (racing adds paddles). wheel, keyboard.",
                   names, idx, [a, p, custom](int i) {
                       const auto& pn = preset_names();
                       int at = custom ? i - 1 : i;
                       if (at < 0 || at >= int(pn.size())) return;
                       a->input().apply_preset(p, pn[at]);
                       a->save_config();
                   });

            {
                static const char* const kLayouts[] = {"modern", "gt3"};
                bool rcustom = in.is_race_custom(p);
                std::vector<std::string> rnames = {"Modern", "GT3 original"};
                int ridx = in.race_layout(p) == "gt3" ? 1 : 0;
                if (rcustom) {
                    rnames.insert(rnames.begin(), "Custom");
                    ridx = 0;
                }
                choice("Race controls",
                       rcustom ? "Custom race bindings are in use. Choosing a layout replaces them."
                               : "Modern: triggers gas/brake, Cross/Square shift, Triangle view. GT3: the original.",
                       rnames, ridx, [a, p, rcustom](int i) {
                           int at = rcustom ? i - 1 : i;
                           if (at < 0 || at > 1) return;
                           a->input().set_race_layout(p, kLayouts[at]);
                           a->save_config();
                       });
            }

            std::vector<std::string> devs = {"auto"};
            for (const std::string& d : in.device_names())
                if (std::find(devs.begin(), devs.end(), d) == devs.end()) devs.push_back(d);
            const std::string& filter = in.device_filter(p);
            int didx = 0;
            for (size_t i = 0; i < devs.size(); ++i)
                if (devs[i] == filter) didx = int(i);
            if (filter != "auto" && didx == 0) {
                devs.push_back(filter); // remembered device that isn't connected right now
                didx = int(devs.size()) - 1;
            }
            std::vector<std::string> shown = devs;
            shown[0] = "Auto: " + in.device_name(p);
            if (filter != "auto" && in.device_name(p) == (in.keyboard_enabled(p) ? "Keyboard" : "No device"))
                shown[didx] += " (not connected)";
            choice("Device", "Controller for this player. Auto picks the first free pad (or wheel).", shown, didx,
                   [a, p, devs](int i) {
                       a->input().set_device_filter(p, devs[i]);
                       a->save_config();
                   });
            toggle("Keyboard", "The keyboard also drives this player (alongside the device).", in.keyboard_enabled(p),
                   [a, p](bool v) {
                       a->input().set_keyboard(p, v);
                       a->save_config();
                   });
        }
        heading("All players");
        {
            bool gt3_views = to_lower(app.config().get("race", "views").value_or("all")) == "gt3";
            choice("Race views", "Change view cycles bumper, bumper without mirror, chase, far chase; GT3: bumper and chase.",
                   {"All four", "GT3 original"}, gt3_views ? 1 : 0, [a](int i) {
                       a->config().set("race", "views", i == 1 ? "gt3" : "all");
                       a->save_config();
                   });
            bool shake = to_lower(app.config().get("race", "camera_shake").value_or("on")) != "off";
            toggle("In-car camera shake", "GT3's in-car vibration: grows with tyre slip and speed.", shake, [a](bool v) {
                a->config().set("race", "camera_shake", v ? "on" : "off");
                a->save_config();
            });
        }
        action("Button mapping...", "Rebind menu buttons and race controls per player; check pressure, test rumble.", "Open",
               [a] { a->push(make_input_screen()); });
    }

    void build_ffb(App& app) {
        App* a = &app;
        ForceFeedbackConfig c = app.input().ffb_config();
        auto apply = [a](const ForceFeedbackConfig& n) {
            a->input().set_ffb_config(n);
            a->save_config();
        };
        toggle("Force feedback", "Master switch for trigger effects and wheel forces.", c.enabled, [apply, c](bool v) {
            ForceFeedbackConfig n = c;
            n.enabled = v;
            apply(n);
        });
        size_t base = rows_.size();
        toggle("Trigger effects", "DualSense brake resistance and ABS pulse; Xbox impulse triggers.", c.triggers,
                          [apply, c](bool v) {
                              ForceFeedbackConfig n = c;
                              n.triggers = v;
                              apply(n);
                          });
        slider("Trigger resistance", "How hard the DualSense brake trigger pushes back.", c.trigger_force, 0, 1,
                          0.05f, percent(c.trigger_force), [apply, c](float v) {
                              ForceFeedbackConfig n = c;
                              n.trigger_force = v;
                              apply(n);
                          });
        toggle("Wheel force feedback", "Steering force and centring spring on force-feedback wheels.", c.wheel,
                          [apply, c](bool v) {
                              ForceFeedbackConfig n = c;
                              n.wheel = v;
                              apply(n);
                          });
        slider("Wheel strength", "Overall wheel force (100% = default, up to 200%).", c.strength, 0, 2, 0.05f,
                          percent(c.strength), [apply, c](float v) {
                              ForceFeedbackConfig n = c;
                              n.strength = v;
                              apply(n);
                          });
        slider("Centring spring", "Self-centring spring of the wheel at speed.", c.spring, 0, 1, 0.05f,
                          percent(c.spring), [apply, c](float v) {
                              ForceFeedbackConfig n = c;
                              n.spring = v;
                              apply(n);
                          });
        toggle("Invert wheel force", "Flip the steering force if your wheel pulls the wrong way.", c.invert,
                          [apply, c](bool v) {
                              ForceFeedbackConfig n = c;
                              n.invert = v;
                              apply(n);
                          });
        // base+0 triggers, +1 resistance, +2 wheel, +3 strength, +4 spring, +5 invert.
        for (size_t i = base; i < base + 6; ++i) rows_[i].enabled = c.enabled;
        if (!c.triggers) rows_[base + 1].enabled = false;
        if (!c.wheel) rows_[base + 3].enabled = rows_[base + 4].enabled = rows_[base + 5].enabled = false;
        info("Device (player 1)", app.input().device_name(0),
             "Effects are felt while driving.");
    }

    void build_touch(App& app) {
        App* a = &app;
        TouchConfig c = app.touch().controls().config();
        auto apply = [a](const TouchConfig& n) {
            a->touch().controls().configure(n);
            a->save_config();
        };
        bool desktop = touch_hidden();
        if (desktop) info("On this computer", "Only used with a touchscreen");
        add(Row::Kind::Preview, "Preview", "");
        size_t first = rows_.size();
        choice("On-screen controls", "Auto: shown after a touch, hidden on pad or keyboard input.",
               {"Auto", "Always", "Off"}, int(c.mode), [apply, c](int i) {
                   TouchConfig n = c;
                   n.mode = TouchConfig::Mode(i);
                   apply(n);
               });
        choice("Steering", "Drag: finger on the left half. Tilt: turn the device.", {"Drag", "Tilt"}, int(c.steering),
               [apply, c](int i) {
                   TouchConfig n = c;
                   n.steering = TouchConfig::Steering(i);
                   apply(n);
               });
        choice("Pedals", "Position: higher = more pressure. Touch area: screen pressure. Full: 100%.",
               {"Position", "Touch area", "Full"}, int(c.pedals), [apply, c](int i) {
                   TouchConfig n = c;
                   n.pedals = TouchConfig::Pedals(i);
                   apply(n);
               });
        slider("Pedal minimum", "Pressure at the bottom of a pedal (position mode).", c.pedal_min, 0, 1, 0.05f,
               percent(c.pedal_min), [apply, c](float v) {
                   TouchConfig n = c;
                   n.pedal_min = v;
                   apply(n);
               });
        slider("Steering range", "Drag distance for full lock, as a share of the screen width.", c.steer_range, 0.03f,
               0.5f, 0.01f, percent(c.steer_range), [apply, c](float v) {
                   TouchConfig n = c;
                   n.steer_range = v;
                   apply(n);
               });
        slider("Tilt deadzone", "Tilt steering ignores this much turn around the centre.", c.tilt_deadzone, 0, 0.5f,
               0.01f, percent(c.tilt_deadzone), [apply, c](float v) {
                   TouchConfig n = c;
                   n.tilt_deadzone = v;
                   apply(n);
               });
        slider("Opacity", "How solid the on-screen controls are drawn.", c.opacity, 0.05f, 1, 0.05f, percent(c.opacity),
               [apply, c](float v) {
                   TouchConfig n = c;
                   n.opacity = v;
                   apply(n);
               });
        slider("Size", "Size of the on-screen controls.", c.size, 0.6f, 1.6f, 0.05f, percent(c.size),
               [apply, c](float v) {
                   TouchConfig n = c;
                   n.size = v;
                   apply(n);
               });
        toggle("Left-handed", "Mirror the layout: pedals on the left, steering on the right.", c.left_handed,
               [apply, c](bool v) {
                   TouchConfig n = c;
                   n.left_handed = v;
                   apply(n);
               });
        if (desktop)
            for (size_t i = first; i < rows_.size(); ++i) rows_[i].dim = true;
    }

    void build_disc(App& app) {
        App* a = &app;
        IniDocument& cfg = app.config();
        std::string current = app.disc() ? cfg.get("game", "disc").value_or("") : "";
        if (app.disc()) {
            const DiscInfo& d = app.disc()->info();
            info("Current disc", d.description);
            info("", d.serial + "  v" + d.version + "  " + d.video);
            info("File", platform::disc_file_name(current), "");
        } else {
            info("Current disc", "None");
        }
        action("Browse for a disc image...", "Pick a .iso made from your own Gran Turismo 3 A-Spec disc.", "",
               [a] { a->show_open_dialog(); });

        std::vector<std::string> recent = app.recent_discs();
        if (!recent.empty()) heading("Recent disc images");
        for (const std::string& p : recent) {
            bool is_current = !current.empty() && platform::same_disc_path(p, current);
            bool ok = disc_available(p);
            std::string where = is_current ? "In use" : ok ? p : "Missing: " + p;
            Row& r = action(platform::disc_file_name(p),
                            is_current ? "This disc is open. Square: remove from the list."
                            : ok       ? "Cross: open this disc (restarts the game). Square: remove."
                                       : "Not found (moved or drive not connected). Square: remove.",
                            where, [a, p, is_current] {
                                if (!is_current) a->request_disc(p);
                            });
            r.enabled = ok;
            r.dim = is_current;
            r.remove = [a, p] { a->forget_recent_disc(p); };
        }
        if (!app.last_error().empty() && !app.failed_disc_path().empty())
            info("Last error", app.last_error(), elide_middle(app.failed_disc_path(), 77));

        heading("Startup");
        bool hub = to_lower(cfg.get("app", "start").value_or("")) == "hub";
        choice("Start in", "Game: opening movie and title menu. Developer options: the tools list.",
               {"Game", "Developer options"}, hub ? 1 : 0, [a](int i) {
                   a->config().set("app", "start", i ? "hub" : "game");
                   a->save_config();
               });
        info("Settings file", app.data_dir() + "config.ini");
    }

    // The update check (app/updates.h) and the relay server it asks.
    void build_online(App& app) {
        App* a = &app;
        Updates& up = updates();
        heading("Updates");
        toggle("Check for updates at startup",
               "Sends one small message with this game's version and platform to the relay server when the game "
               "starts. Off: only when you choose Check now.",
               up.enabled(app), [a](bool v) { updates().set_enabled(*a, v); });
        action("Check now", "Asks the relay server for the latest release (a few seconds at most).", up.status_text(),
               [a] { updates().check_now(*a); });
        info("This version", net::this_game_version().to_string());
        info("Latest release", (up.latest().known() ? up.latest().number() : std::string("Unknown")) +
                                   (up.checked().empty() ? "" : "  (as of " + up.checked() + ")"));
        // Sessions through the relay server need it; offline play and Host / Join by address never do.
        info("Online play needs", up.min_online().known() ? "Version " + up.min_online().number() + " or later" : "Any version");
        net::UpdateLinks links = up.links();
        Row& dl = action("Download update",
                         up.prompt() == net::UpdatePrompt::None ? "This version is up to date (as far as the last check knows)."
                                                                : "Opens the update dialog: " + links.download,
                         up.prompt() == net::UpdatePrompt::None ? "Up to date" : "Version " + up.latest().number(),
                         [] { updates().open_dialog(); });
        dl.enabled = up.prompt() != net::UpdatePrompt::None;
        Row& notes = action("Release notes", links.notes.empty() ? "The relay server has not named a release page yet." : links.notes,
                            links.notes.empty() ? "Unknown" : "", [] { updates().open_notes(); });
        notes.enabled = !links.notes.empty();
        heading("Online battle");
        std::string server = app.config().get("online", "server").value_or(kDefaultServer);
        info("Relay server", server.empty() ? kDefaultServer : server,
             "Change it in the online lobby (Relay server). The update check asks the same server.");
    }

    // --- behaviour ---------------------------------------------------------

    static bool touch_hidden() { return !platform::is_mobile(); }

    int first_selectable() const {
        for (size_t i = 0; i < rows_.size(); ++i)
            if (rows_[i].selectable()) return int(i);
        return -1;
    }

    void move(int dir) {
        if (focus_tabs_) {
            if (dir > 0) {
                focus_tabs_ = false;
                sel_ = first_selectable();
            }
            return;
        }
        int n = int(rows_.size());
        for (int i = sel_ + dir; i >= 0 && i < n; i += dir)
            if (rows_[i].selectable()) {
                sel_ = i;
                return;
            }
        if (dir < 0) {
            focus_tabs_ = true; // up past the first option: the tab bar
            scroll_ = 0;
        }
    }

    void switch_tab(App& app, int d) {
        if (tab_ == kAudio) stop_preview(app);
        tab_ = (tab_ + d + kTabCount) % kTabCount;
        sel_ = -1;
        scroll_ = 0;
        follow_sel_ = true;
        dragging_ = -1;
        press_ = Press{};
        build(app);
    }

    void leave(App& app) { stop_preview(app); }

    void start_preview(App& app) {
        if (!app.vfs() || app.audio().music_playing()) return;
        AudioEngine& au = app.audio();
        if (au.play_sequence(*app.vfs(), std::string("gtmenu01")) || au.play_sequence(*app.vfs(), 0)) preview_ = true;
    }

    void stop_preview(App& app) {
        if (!preview_) return;
        app.audio().stop_music();
        preview_ = false;
    }

    void play_se(App& app, const char* name) {
        if (!app.vfs()) return;
        if (!se_tried_) {
            se_tried_ = true;
            sounds_.load(*app.vfs());
        }
        sounds_.play(app.audio().voices(), name);
    }

    std::string footer_hint() const {
        if (sel_ < 0 || sel_ >= int(rows_.size()) || focus_tabs_) return "Left/Right: section    Triangle/Esc: back";
        const Row& r = rows_[sel_];
        switch (r.kind) {
        case Row::Kind::Toggle: return "Cross or Left/Right: switch    Triangle/Esc: back";
        case Row::Kind::Choice: return "Left/Right: change    Triangle/Esc: back";
        case Row::Kind::Slider: return "Left/Right: adjust (or drag)    Triangle/Esc: back";
        default: return r.remove ? "Cross: open    Square: remove    Triangle/Esc: back" : "Cross: open    Triangle/Esc: back";
        }
    }

    void draw_row(App& app, Ui& ui, const Row& r, float y, float h, bool selected, bool pressed) {
        const UiRect& S = ui.safe();
        const bool touch = ui.touch();
        const float ty = std::floor(y + (h - Ui::kChar) / 2), cy = y + h / 2;
        const float x0 = S.x + 6, x1 = S.right() - 8;
        if (r.kind == Row::Kind::Heading) {
            float hy = std::floor(y + h - 12);
            ui.text(lay_.label_x - 8, hy, r.label, colors::kAccent);
            float lx = lay_.label_x - 8 + Ui::text_width(r.label) + 8;
            ui.fill(lx, hy + 4, x1 - lx, 1, colors::kOutline);
            return;
        }
        if (r.kind == Row::Kind::Preview) {
            ui.text(lay_.label_x, y + 8, r.label, colors::kDim);
            const float px = lay_.val_x - (touch ? 40 : 60);
            float ly = y + 22;
            for (const std::string& line :
                 Ui::wrap("Driving layout with these settings (opacity, size, left-handed).", px - lay_.label_x - 12))
                ui.text(lay_.label_x, ly, line, colors::kDim), ly += 11;
            const bool inside = y >= lay_.list.y - 0.5f && y + h <= lay_.list.bottom() + 0.5f;
            app.touch().draw_preview(ui, {px, y + 4, x1 - px, h - 8}, inside);
            return;
        }
        if (selected || pressed) {
            ui.fill_round(x0, y + 1, x1 - x0, h - 2, touch ? 6.0f : 3.0f, pressed ? Color{62, 70, 100} : colors::kHighlight);
            ui.fill_round(x0, y + 4, 3, h - 8, 1.5f, colors::kAccent);
        } else if (touch && r.selectable()) {
            ui.fill(x0 + 8, y + h - 1, x1 - x0 - 16, 1, Color{34, 37, 48}); // row separators
        }
        const bool grey = !r.enabled || r.dim;
        Color label = r.kind == Row::Kind::Info ? colors::kDim : grey ? colors::kDim : selected ? colors::kAccent : colors::kText;
        std::string lbl = Ui::elide(r.label, lay_.val_x - lay_.label_x - 12);
        ui.text(lay_.label_x, ty, lbl, label);
        if (r.restart && lay_.label_x + Ui::text_width(lbl) + 16 + Ui::text_width("RESTART") < lay_.val_x)
            ui.tag(lay_.label_x + Ui::text_width(lbl) + 8, ty, "RESTART", colors::kAccent);

        const float vx = lay_.val_x, vw = lay_.val_w;
        Color val = grey ? colors::kDim : colors::kText;
        switch (r.kind) {
        case Row::Kind::Toggle: ui.toggle_pill(vx, cy, r.on, !grey); break;
        case Row::Kind::Choice: {
            Color arrow = !grey ? (selected ? colors::kAccent : colors::kText) : colors::kOutline;
            if (touch) { // tap targets for the two halves
                const float bw = 26, bh = h - 8;
                ui.fill_round(vx, y + 4, bw, bh, 5, Color{40, 43, 54});
                ui.fill_round(vx + vw - bw, y + 4, bw, bh, 5, Color{40, 43, 54});
                ui.text_center_in(vx, bw, ty, "<", arrow);
                ui.text_center_in(vx + vw - bw, bw, ty, ">", arrow);
            } else {
                ui.text(vx, ty, "<", arrow);
                ui.text_right(vx + vw, ty, ">", arrow);
            }
            ui.text_center_in(vx, vw, ty, Ui::elide(r.value, vw - (touch ? 60 : 32)), val);
            break;
        }
        case Row::Kind::Slider:
            ui.slider(vx, cy, lay_.slider_w, r.frac, selected, !grey);
            ui.text_right(vx + vw, ty, r.value, val);
            break;
        case Row::Kind::Action:
            ui.text(vx, ty, Ui::elide(r.value, vw - 16), r.enabled ? colors::kDim : colors::kBad);
            if (r.enabled && !r.dim) ui.text_right(vx + vw, ty, ">", selected ? colors::kAccent : colors::kOutline);
            break;
        case Row::Kind::Info: ui.text(vx, ty, Ui::elide(r.value, vw), colors::kText); break;
        default: break;
        }
    }

    std::vector<Row> rows_;
    int tab_ = 0;
    int sel_ = -1;
    float scroll_ = 0;
    bool follow_sel_ = true; // keep the selection in view (keyboard / pad); off while scrolling by touch
    int dragging_ = -1;      // slider row being dragged
    bool focus_tabs_ = false;
    bool preview_ = false;
    bool se_tried_ = false;
    Gesture gesture_;
    Press press_;
    Layout lay_;
    menu::flow::MenuSounds sounds_;
};


} // namespace

std::unique_ptr<Screen> make_settings_screen(int tab) { return std::make_unique<SettingsScreen>(tab); }

GT3R_REGISTER_PLAYER_SCREEN_NO_DISC(settings, "settings", "Settings",
                             "Display, audio, controls, force feedback, touch and disc settings.", 3,
                             [] { return make_settings_screen(); });

} // namespace gt3r
