#pragma once

// Minimal 3D math for the renderer: column-major 4x4 matrices (same layout
// as scene::Mesh::transform and GLSL/MSL uniforms), right-handed, Y up,
// clip-space depth 0..1 (SDL_GPU uses that on every backend).

#include <array>
#include <cmath>

namespace gt3r::render {

struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
};

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) {
    float l = length(v);
    return l > 0 ? v * (1.0f / l) : v;
}

// Column-major: m[col * 4 + row].
struct Mat4 {
    std::array<float, 16> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static Mat4 identity() { return {}; }
    static Mat4 from(const std::array<float, 16>& a) { return Mat4{a}; }
    float& at(int row, int col) { return m[col * 4 + row]; }
    float at(int row, int col) const { return m[col * 4 + row]; }

    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for (int c = 0; c < 4; ++c)
            for (int rr = 0; rr < 4; ++rr) {
                float s = 0;
                for (int k = 0; k < 4; ++k) s += at(rr, k) * o.at(k, c);
                r.at(rr, c) = s;
            }
        return r;
    }
    Vec3 transform_point(const Vec3& p) const {
        return {at(0, 0) * p.x + at(0, 1) * p.y + at(0, 2) * p.z + at(0, 3),
                at(1, 0) * p.x + at(1, 1) * p.y + at(1, 2) * p.z + at(1, 3),
                at(2, 0) * p.x + at(2, 1) * p.y + at(2, 2) * p.z + at(2, 3)};
    }
};

Mat4 translate(const Vec3& t);
Mat4 scale(const Vec3& s);
Mat4 rotate_x(float radians);
Mat4 rotate_y(float radians);
Mat4 rotate_z(float radians);
// Rotation matrix of a unit quaternion (w, x, y, z).
Mat4 rotate_quat(float w, float x, float y, float z);
// Right-handed perspective, depth mapped to 0..1. `fovy` in radians.
Mat4 perspective(float fovy, float aspect, float znear, float zfar);
Mat4 look_at(const Vec3& eye, const Vec3& target, const Vec3& up);
// General inverse; returns identity for singular matrices.
Mat4 inverse(const Mat4& a);
Mat4 transpose(const Mat4& a);

} // namespace gt3r::render
