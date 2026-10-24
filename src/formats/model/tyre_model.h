#pragma once

// The tyre GT3 builds around each wheel. The car file holds only its texture
// (section GTTR); the game makes the geometry from the tyre size:
//
// - a textured, lit ring: the tread, a 45-degree shoulder and the outer
//   sidewall, from the rim out to the tread, with the tyre material;
// - black, unlit faces closing the inner side (and the tread seen from inside);
// - at the highest detail, a translucent black band over the top of the tyre
//   (the wheel arch's shade), darkest at the inner edge.
//
// See docs/formats/car_model.md "Tyres".

#include "scene/model.h"

#include <vector>

namespace gt3r {

// The tyre's size (from the database tyre size) and its GTTR header fields.
struct TyreShape {
    float radius = 0.3f;   // R: outer radius (m)
    float width = 0.2f;    // W: section width (m)
    float sidewall = 0.1f; // H: sidewall height (m); the rim (and the sidewall's inner edge) is at R - 0.75 H
    u32 flags = 0;         // GTTR +0x14; bit 1: symmetric tyre (both sides textured, no band)
    float repeats = 2;     // GTTR +0x10: times the texture goes round the tread
    float shininess = 8;   // GTTR +0x18: the tyre material's shininess
};

// How much of the tyre is built: the menus' showroom tyre, or one of the race's levels of detail.
struct TyreDetail {
    int segments = 18;
    bool shoulder = true;    // the 45-degree shoulder between the sidewall and the tread
    bool inner_faces = true; // the black faces seen from inside the tyre
    bool band = true;        // the arch shade
    float outer_inset = 0;   // the outer sidewall's place inward of the rim face (m)
};
TyreDetail menu_tyre_detail();
TyreDetail race_tyre_detail(int lod); // 0..2 (higher levels use 2)

// The tyre of a left-hand wheel around its hub at the origin: axle along X, outer face towards
// -X (where the rim's face is, at -W/2). Returns the ring (textured lit pieces and the black
// faces) and, when built, the band. `right`: the wheel is a right-hand one (the caller turns it
// half a turn about Y), which runs the tread texture the other way. `phase`: where the texture
// starts round the tyre (0..1; the game picks it at random per tyre).
std::vector<scene::Mesh> build_tyre(const TyreShape& shape, const TyreDetail& detail, int texture, bool right,
                                    float phase);

// The tyre material: Ma 0.3, Md 0.15, Ms 0.9, no emission.
scene::Material tyre_material(float shininess);

} // namespace gt3r
