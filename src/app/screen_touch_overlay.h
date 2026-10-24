#pragma once

#include "input/touch_controls.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_rect.h>

namespace gt3r {

class IniDocument;
class InputManager;
class Ui;
struct UiRect;

// App-side glue for the on-screen touch controls (input/touch_controls.h):
// routes SDL touch events in, feeds the result into player 1's virtual pad
// and draws the controls on top of the current screen.
class TouchOverlay {
public:
    void load(const IniDocument& config);
    void save(IniDocument& config) const { controls_.config().save(config); }

    // `e` must already be in UI coordinates (SDL_ConvertEventToRenderCoordinates).
    // Returns true when the event belongs to the overlay and must not reach
    // the screen (touches on a control and the mouse events SDL synthesizes
    // from them).
    bool event(const SDL_Event& e);
    // Call once per frame before InputManager::update().
    void update(double dt, TouchLayout layout, InputManager& input);
    void render(Ui& ui);
    // Window area (window coordinates) clear of display cutouts and rounded
    // corners; the controls keep out of the rest. App sets it every frame.
    void set_safe_area(const SDL_FRect& r) { safe_window_ = r; }
    // Draws the driving layout with the current [touch] settings, scaled
    // into `box` (UI units of the current layout): the Settings preview.
    // The controls ignore the clip rect: pass `controls` = false while the
    // box is partly scrolled out of view.
    void draw_preview(Ui& ui, const UiRect& box, bool controls = true) const;
    // Part of the 640x360 game area not covered by the touch controls (UI
    // units); the whole area while they are hidden. HUDs inset into it.
    TouchRect safe_area() const { return controls_.safe_area(); }
    // The visible controls' drawn bounds (TouchControls::control_bounds).
    std::vector<TouchRect> control_bounds() const { return controls_.control_bounds(); }
    // A screen's own button moved up clear of the visible controls (TouchControls::clear_above).
    TouchRect clear_above(TouchRect r) const { return controls_.clear_above(r); }

    TouchControls& controls() { return controls_; }

private:
    TouchControls controls_;
    SDL_FRect safe_window_{0, 0, 0, 0};
    bool swallow_mouse_ = false;
};

} // namespace gt3r
