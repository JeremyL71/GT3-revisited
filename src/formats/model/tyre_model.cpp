#include "formats/model/tyre_model.h"

#include <algorithm>
#include <cmath>

namespace gt3r {

namespace {

constexpr float kPi = 3.14159265f;
constexpr u32 kBlack = 0xFF000000u; // RGBA8 memory order R, G, B, A: opaque black

// A point across the tyre's profile: x along the axle, r from it, the normal's (x, radial)
// parts, and the texture's T (0 = the image's top row).
struct Edge {
    float x, r, nx, nr, t;
};

class Builder {
public:
    explicit Builder(int segments) : n_(segments) {}

    // One piece of the ring between two edges (a strip round the tyre). The faces look the way
    // the normals point. s(i) is the texture's S at angle index i; `mirror` takes the angles
    // as N - i (the inner side of a symmetric tyre).
    template <class S>
    void strip(scene::Mesh& m, const Edge& a, const Edge& b, S&& s, bool mirror = false, u32 color = 0xFFFFFFFFu,
               bool flip = false) {
        const u32 base = u32(m.vertices.size());
        for (int i = 0; i <= n_; ++i) {
            const int k = mirror ? n_ - i : i;
            m.vertices.push_back(vertex(a, k, s(i), color, flip));
            m.vertices.push_back(vertex(b, k, s(i), color, flip));
        }
        for (int i = 0; i < n_; ++i) {
            const u32 a0 = base + 2 * u32(i), b0 = a0 + 1, a1 = a0 + 2, b1 = a0 + 3;
            triangle(m, a0, b0, a1);
            triangle(m, a1, b0, b1);
        }
    }

    // A flat disc at x, radius r, facing +X (or -X).
    void disc(scene::Mesh& m, float x, float r, bool towards_minus_x) {
        const u32 base = u32(m.vertices.size());
        const Edge e{x, r, towards_minus_x ? -1.0f : 1.0f, 0, 0};
        for (int i = 0; i < n_; ++i) m.vertices.push_back(vertex(e, i, 0, kBlack, false));
        for (int i = 1; i + 1 < n_; ++i) triangle(m, base, base + u32(i), base + u32(i) + 1);
    }

private:
    scene::Vertex vertex(const Edge& e, int k, float s, u32 color, bool flip) const {
        // Angle index k of N: (x, r sin, r cos); index N is exactly index 0.
        const float th = 2 * kPi * float(k % n_) / float(n_);
        const float sn = std::sin(th), cs = std::cos(th), f = flip ? -1.0f : 1.0f;
        scene::Vertex v;
        v.pos[0] = e.x, v.pos[1] = e.r * sn, v.pos[2] = e.r * cs;
        v.normal[0] = f * e.nx, v.normal[1] = f * e.nr * sn, v.normal[2] = f * e.nr * cs;
        v.uv[0] = s, v.uv[1] = e.t;
        v.color = color;
        return v;
    }

    // Wound so that it faces the way its vertices' normals point (counter-clockwise = front).
    static void triangle(scene::Mesh& m, u32 a, u32 b, u32 c) {
        const scene::Vertex &va = m.vertices[a], &vb = m.vertices[b], &vc = m.vertices[c];
        float e1[3], e2[3], n[3] = {}, g[3];
        for (int k = 0; k < 3; ++k) {
            e1[k] = vb.pos[k] - va.pos[k], e2[k] = vc.pos[k] - va.pos[k];
            n[k] = va.normal[k] + vb.normal[k] + vc.normal[k];
        }
        g[0] = e1[1] * e2[2] - e1[2] * e2[1];
        g[1] = e1[2] * e2[0] - e1[0] * e2[2];
        g[2] = e1[0] * e2[1] - e1[1] * e2[0];
        if (g[0] * n[0] + g[1] * n[1] + g[2] * n[2] < 0) std::swap(b, c);
        m.indices.insert(m.indices.end(), {a, b, c});
    }

    int n_;
};

void close_primitive(scene::Mesh& m, u32& first, int texture, u32 flags, const scene::Material& mat) {
    if (m.indices.size() == first) return;
    scene::Primitive p;
    p.first_index = first;
    p.index_count = u32(m.indices.size()) - first;
    p.texture = texture;
    p.flags = flags;
    p.material = mat;
    m.primitives.push_back(p);
    first = u32(m.indices.size());
}

} // namespace

TyreDetail menu_tyre_detail() {
    TyreDetail d;
    d.segments = 30;
    d.outer_inset = 0.001f;
    return d;
}

TyreDetail race_tyre_detail(int lod) {
    TyreDetail d;
    lod = std::clamp(lod, 0, 2);
    d.segments = lod == 0 ? 18 : lod == 1 ? 12 : 8;
    d.shoulder = lod < 2;
    d.inner_faces = lod < 2;
    d.band = lod == 0;
    d.outer_inset = -0.004f;
    return d;
}

scene::Material tyre_material(float shininess) {
    scene::Material m;
    for (int i = 0; i < 3; ++i) m.ambient[i] = 0.3f, m.diffuse[i] = 0.15f, m.specular[i] = 0.9f;
    m.shininess = shininess;
    m.valid = true;
    return m;
}

std::vector<scene::Mesh> build_tyre(const TyreShape& shape, const TyreDetail& detail, int texture, bool right,
                                    float phase) {
    const int n = std::max(3, detail.segments);
    const float R = shape.radius, W = shape.width;
    const float b = R - 0.75f * shape.sidewall; // the rim radius
    const bool symmetric = (shape.flags & 2) != 0;
    const float k = symmetric ? 0.3f : 0.25f;
    const float h = detail.shoulder ? k * (R - b) : 0.0f; // the shoulder's depth
    const float xo = -0.5f * W + detail.outer_inset, xi = 0.5f * W;
    const float rep = shape.repeats;
    // S round the tyre: the tread runs one way on left wheels and the other on right ones;
    // the sidewall lettering runs the same way on both.
    auto up = [&](int i) { return phase + float(i) * rep / float(n); };
    auto down = [&](int i) { return phase + rep - float(i) * rep / float(n); };
    auto tread_s = [&](int i) { return right ? up(i) : down(i); };
    // T without a shoulder: the tread's outer edge and the sidewall's outer edge meet there.
    const float t_edge = detail.shoulder ? 0.5f : 1.0f - 0.375f / (1.0f - k);
    const float t_side = detail.shoulder ? 0.625f : t_edge;
    const float c8 = 0.9239f, s8 = 0.3827f, r2 = 0.7071f; // cos / sin 22.5 degrees, 1 / sqrt 2

    Builder bld(n);
    scene::Mesh ring;
    ring.name = "tyre";
    u32 first = 0;
    // Textured, lit: tread, shoulder, outer sidewall (and with a symmetric tyre the inner ones).
    const Edge tread_a = detail.shoulder ? Edge{xo + h, R, -s8, c8, t_edge} : Edge{xo, R, -r2, r2, t_edge};
    const Edge tread_b = !symmetric ? Edge{xi, R, 0, 1, 0}
                         : detail.shoulder ? Edge{xi - h, R, s8, c8, 0}
                                           : Edge{xi, R, r2, r2, 0};
    bld.strip(ring, tread_a, tread_b, tread_s);
    const Edge side_rim{xo, b, -r2, -r2, 1.0f};
    const Edge side_out = detail.shoulder ? Edge{xo, R - h, -c8, s8, t_side} : Edge{xo, R, -r2, r2, t_side};
    if (detail.shoulder) bld.strip(ring, Edge{xo, R - h, -c8, s8, 0.625f}, Edge{xo + h, R, -s8, c8, 0.5f}, tread_s);
    bld.strip(ring, side_rim, side_out, down);
    auto inner = [&](Edge e) { return Edge{-e.x, e.r, -e.nx, e.nr, e.t}; };
    if (symmetric) {
        // The inner side mirrors the outer one about the tyre's middle (x = 0 here is -W/2 +
        // W/2): x' = xi - (x - xo) for the pieces, as the builder places them at x = W.
        auto at_inner = [&](Edge e) {
            Edge m = inner(e);
            m.x = xi - (e.x - xo);
            return m;
        };
        if (detail.shoulder)
            bld.strip(ring, at_inner(Edge{xo, R - h, -c8, s8, 0.625f}), at_inner(Edge{xo + h, R, -s8, c8, 0.5f}),
                      tread_s, true);
        bld.strip(ring, at_inner(side_rim), at_inner(side_out), down, true);
    }
    close_primitive(ring, first, texture, 0, tyre_material(shape.shininess));
    // Black, unlit: the disc closing the inner side and the faces seen from inside the tyre.
    auto zero = [](int) { return 0.0f; };
    bld.disc(ring, xi, symmetric ? b : R, false);
    if (detail.inner_faces) {
        bld.disc(ring, xi, symmetric ? b : R, true);
        bld.strip(ring, tread_a, tread_b, zero, false, kBlack, true);
        if (symmetric) {
            Edge a = inner(Edge{xo, R - h, -c8, s8, 0}), c = inner(Edge{xo + h, R, -s8, c8, 0});
            a.x = xi, c.x = xi - h;
            if (detail.shoulder) bld.strip(ring, a, c, zero, true, kBlack, true);
            Edge r0 = inner(side_rim), r1 = inner(side_out);
            r0.x = r1.x = xi;
            bld.strip(ring, r0, r1, zero, true, kBlack, true);
        }
    }
    close_primitive(ring, first, -1, scene::kUnlit, {});

    std::vector<scene::Mesh> out;
    out.push_back(std::move(ring));
    // The arch shade: half a ring over the top (angle index 0..N/2), 3 mm above the tread,
    // stretched along the car to sqrt(R^2 + W^2/4), from 2 cm inboard of the outer face
    // (transparent) to 2 cm inboard of the inner face. Its alpha there is
    // 256 - (i - s)^2 256 / s^2 (s = N/4 + 1, whole numbers), 0 at both ends; the GS blends it
    // as Cd (1 - As/128), clamped, so it is fully black wherever As passes 128: a third
    // vertex marks that point across the band.
    if (detail.band && !symmetric) {
        scene::Mesh band;
        band.name = "band";
        const float rb = std::sqrt(R * R + 0.25f * W * W);
        const float x0 = -0.5f * W + 0.02f, x1 = 0.5f * W + 0.02f;
        const int s = n / 4 + 1;
        for (int i = 0; i <= n / 2; ++i) {
            int a = 0;
            if (i > 0 && i < n / 2) a = std::clamp(256 - int(float((i - s) * (i - s)) * (256.0f / float(s * s))), 0, 255);
            const float th = 2 * kPi * float(i) / float(n);
            const float y = R * std::sin(th) + 0.003f, z = rb * std::cos(th);
            const float f = a > 128 ? 128.0f / float(a) : 1.0f; // where As reaches 128
            const u32 alpha = u32(std::lround(float(std::min(a, 128)) / 128.0f * 255.0f));
            const float xs[3] = {x0, x0 + f * (x1 - x0), x1};
            const u32 al[3] = {0, alpha, alpha};
            for (int c = 0; c < 3; ++c) {
                scene::Vertex v;
                v.pos[0] = xs[c], v.pos[1] = y, v.pos[2] = z;
                v.normal[1] = 1;
                v.color = al[c] << 24;
                band.vertices.push_back(v);
            }
        }
        for (u32 i = 0; i + 1 <= u32(n / 2); ++i)
            for (u32 c = 0; c < 2; ++c) {
                const u32 p = 3 * i + c, q = p + 3;
                band.indices.insert(band.indices.end(), {p, p + 1, q, q, p + 1, q + 1});
            }
        u32 bf = 0;
        close_primitive(band, bf, -1, scene::kAlphaBlend | scene::kUnlit | scene::kNoDepthWrite | scene::kDoubleSided,
                        {});
        out.push_back(std::move(band));
    }
    return out;
}

} // namespace gt3r
