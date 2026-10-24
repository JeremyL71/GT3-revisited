#pragma once

// Builds renderable models for a course file: scenery GTM1 blocks merged into
// one scene::Model, plus helper geometry for debugging (road surface, racing
// lines drawn as flat ribbons).

#include "formats/course/ai_lines.h"
#include "formats/course/course_file.h"
#include "formats/course/road.h"
#include "formats/model/gtm1.h"
#include "scene/model.h"

namespace gt3r::course {

struct CourseModelOptions {
    bool scenery = true;     // slot 1
    bool scenery2 = false;   // slot 5 (secondary set; overlaps slot 1)
    bool backdrop = true;    // slot 22
    bool sky = false;        // slot 21: the camera-centred sky dome of the race view
    bool sky_low = false;    // slot 24: a smaller, low-resolution dome the race draws only in a
                             // secondary view pass, never in the main view
    bool sky_object = false; // slot 23: the small dark sky dome drawn only into the car reflection map
    // Slot 27, the licence course line: a driving line on the road (13 licence course files,
    // 12 used by B / A tests), drawn with every view of those courses. See
    // course_line_model() for the race's own copy.
    bool course_line = true;
    // Wet-road reflection pass (route5 / route11 families, the L_circle tests): slot 9 is the
    // pre-mirrored reflection model drawn into a small offscreen buffer, slot 13 the alpha-only
    // mask that weights it on the frame. Not part of the visible scenery.
    bool road_reflection = false;      // slot 9
    bool road_reflection_mask = false; // slot 13
    bool reflections = false; // chunk kind 4 passes: road sheen, masked by the base pass alpha (gtm1::Options::road_sheen)
    bool frame_alpha = false; // keep the alpha-only shadow masks (gtm1::Options::frame_alpha)
    bool textures = true;
    int lod = 0;
    // Debug overlays.
    bool road_mesh = false;   // GTRW surface, flat-shaded by surface type
    bool ai_lines = false;    // AI line 0 (and the centre line) as ribbons
};

struct Course {
    CourseFile file;
    Road road;
    AiLines ai;
};

// Parses the course container plus road and AI lines.
Course load_course(ByteSpan data);

// Merges the selected GTM1 blocks and overlays into one model. `clut_sets` (when given)
// receives the texture sets with CLUT animations, `first` indexing the merged model's textures.
// `animated` (when given) receives each block's nodes the +0x3C animators move, left out of the
// model (texture indices: the merged model's).
scene::Model build_course_model(ByteSpan data, const CourseModelOptions& opt = {}, const std::string& name = {},
                                std::vector<gtm1::ClutSet>* clut_sets = nullptr,
                                std::vector<gtm1::AnimatedNodes>* animated = nullptr);

// The licence course line (slot 27) on its own: a 0.4 m wide ribbon about 0.1 m above the
// road along the test's section, untextured, pre-lit, coloured per dash (blue, grey, red and
// blends between them: the demonstration's pedals). Its node 0 only sets state for node 1's
// draw, so the primitives come out double-sided, without depth writes and keeping the frame
// alpha (scene::kKeepAlpha). Empty when the course file has no line. docs/formats/course.md
// "Course line".
scene::Model course_line_model(ByteSpan data, const std::string& name = {});

// Debug geometry. Colours are RGBA8 (memory order R, G, B, A).
scene::Mesh make_road_mesh(const Road& road);
scene::Mesh make_ribbon(const std::vector<Vec3>& points, float width, u32 color, float lift = 0.15f,
                        const std::string& name = "line");

// Appends `src` to `dst`, re-indexing textures.
void append_model(scene::Model& dst, scene::Model&& src);

} // namespace gt3r::course
