#pragma once

// The online lobby's car and course pickers (docs/dev/ONLINE.md "Pickers"), after
// GT3's arcade pickers with the disc's own art:
//
//   Car: a maker page (GT Mode's dealers: the makers' logos and flags by region, as
//   cd/cd_<region> shows them; the GT Mode garage first), then the maker's cars as the arcade car page shows them: the
//   strip of "<car>_mini" icons (ar/select_car), the car turning in the arcade
//   showroom (menu/showroom_single.fbi, the page's reflection map) and the paint
//   column with each paint's colour and name (carcolor.db / carcolor.sdb).
//   Course: the arcade course page: the course's still behind (stream/course.ipic),
//   its logo and map, the strip of "<course>_mini" icons.
//
// Entries without disc art (courses the arcade lists lack, mod cars outside arcade
// mode) are drawn as text.

#include "base/bytes.h"

#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_render.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gt3r {

class App;
class Ui;
struct Color;
namespace data {
class CarDatabase;
}

struct PickerCar {
    std::string label, name, model;
    int garage = -1; // GT Mode garage index (the car with its parts and settings)
    bool mod = false;
    std::string maker; // database maker ("NISSAN"); garage cars are listed under the garage
    int paint = 0;     // garage cars: their own paint
    std::string wheel; // garage cars: the fitted wheel ("" = stock)
};

struct PickerCourse {
    std::string label, name;
};

struct PaintInfo {
    u32 rgb = 0; // bytes R, G, B
    std::string name;
};

// The disc art the lobby and the pickers draw, uploaded on first use.
class PickerArt {
public:
    PickerArt();
    ~PickerArt();
    PickerArt(const PickerArt&) = delete;
    PickerArt& operator=(const PickerArt&) = delete;
    // Rectangles in UI units; the art is fitted (aspect kept) and centred. False when
    // there is no art for the name.
    bool car_icon(App& app, const std::string& car, const SDL_FRect& dst, float alpha = 1, float bright = 1);
    // The maker's logo from GT Mode's dealers (a tuner's from the tune shop).
    bool maker_logo(App& app, const std::string& maker, const SDL_FRect& dst, float alpha = 1);
    // The dealer region of a maker (0 Japan, 1 USA, 2 Germany, ... as GT Mode lists them;
    // -1 = none) and the region's flag.
    int maker_region(App& app, const std::string& maker);
    bool region_flag(App& app, int region, const SDL_FRect& dst, float alpha = 1);
    bool course_icon(App& app, const std::string& course, const SDL_FRect& dst, float alpha = 1);
    bool course_map(App& app, const std::string& course, const SDL_FRect& dst, float alpha = 1);
    bool course_logo(App& app, const std::string& course, const SDL_FRect& dst, float alpha = 1);
    // The course's still (640 x 448) filling `dst`, cropped to its aspect when `cover`.
    bool course_still(App& app, const std::string& course, const SDL_FRect& dst, bool cover, float alpha = 1);
    // The arcade car page's showroom reflection map (its "car" element image), or null.
    struct Image {
        u32 width = 0, height = 0;
        const std::vector<u32>* rgba = nullptr;
    };
    Image reflection_map(App& app);

private:
    struct Art;
    struct Ref;
    void load(App& app);
    bool draw_ref(App& app, const Ref* r, const SDL_FRect& dst, float alpha, float bright = 1);
    SDL_Texture* texture(App& app, int scene, int image);

    bool loaded_ = false;
    std::unique_ptr<Art> art_;
    std::map<std::pair<int, int>, SDL_Texture*> textures_;
    std::map<std::string, SDL_Texture*> stills_;
};

// A car turning in the arcade showroom, drawn into a rectangle of the UI.
class CarTurntable {
public:
    CarTurntable();
    ~CarTurntable();
    CarTurntable(const CarTurntable&) = delete;
    CarTurntable& operator=(const CarTurntable&) = delete;
    // Shows `car` (database label) with paint index `paint`; the model loads once the
    // choice has rested a few frames (as the original's showroom waits for its load).
    void draw(App& app, const SDL_FRect& dst, const data::CarDatabase& db, const PickerCar& car, int paint,
              PickerArt& art);

private:
    struct State;
    std::unique_ptr<State> s_;
};

class CarPicker {
public:
    CarPicker();
    ~CarPicker();
    enum class Result { None, Chosen, Cancelled };
    // `art`, `cars` and `db` must outlive the picker while it is open. `note(i, paint)` gives a
    // warning for a car (the lobby's rules), "" when fine.
    void open(App& app, PickerArt& art, const data::CarDatabase& db, const std::vector<PickerCar>& cars, int selected,
              int paint, std::function<std::string(int, int)> note);
    Result update(App& app);
    void render(App& app, Ui& ui);
    void show_makers() { in_group_ = false; } // the maker page instead of the car's own
    int car() const { return car_; }
    int paint() const { return paint_; }

    // The paints of a car: its own for a garage car, else its model's list (one unnamed
    // grey paint when unknown).
    std::vector<PaintInfo> paints(App& app, const PickerCar& c) const;

private:
    struct Group {
        std::string maker, title; // maker "" = the garage
        std::vector<int> cars;    // indices into cars_
        int region = -1;          // dealer region (PickerArt::maker_region)
    };
    void build_groups(App& app);
    void enter_group(int g, int pos);
    void render_makers(App& app, Ui& ui);
    void render_cars(App& app, Ui& ui);

    const data::CarDatabase* db_ = nullptr;
    const std::vector<PickerCar>* cars_ = nullptr;
    std::function<std::string(int, int)> note_;
    std::vector<Group> groups_;
    bool in_group_ = false;
    int group_ = 0, pos_ = 0; // focused group, focused car within it
    int car_ = 0, paint_ = 0;
    int maker_scroll_ = 0;
    float strip_x_ = 0;     // eased strip scroll (UI units)
    float paint_y_ = 0;     // eased paint column scroll (slots)
    int pulse_ = 0;         // frames since the last move (the focused icon's pulse)
    PickerArt* art_ = nullptr;
    CarTurntable turntable_;
};

class CoursePicker {
public:
    enum class Result { None, Chosen, Cancelled };
    // `length_m(label)`: the course length in metres (0 = unknown).
    void open(const std::vector<PickerCourse>& courses, int selected, std::function<u32(const std::string&)> length_m);
    Result update(App& app);
    void render(App& app, Ui& ui, PickerArt& art);
    int course() const { return sel_; }

private:
    const std::vector<PickerCourse>* courses_ = nullptr;
    std::function<u32(const std::string&)> length_m_;
    int sel_ = 0, prev_ = -1;
    float strip_x_ = 0;
    int pulse_ = 0, fade_ = 0; // frames since the last move
};

// Plays a named menu sound effect (sound/se.inf: "cursor", "ok", "cancel", "disable")
// when the menu SE option is on.
void picker_sound(App& app, const char* name);

// A paint chip: the colour filled, outlined.
void draw_paint_chip(Ui& ui, float x, float y, float w, float h, u32 rgb, bool focused);

} // namespace gt3r
