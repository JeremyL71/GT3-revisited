#include "render/math3d.h"

#include <utility>

namespace gt3r::render {

Mat4 translate(const Vec3& t) {
    Mat4 r;
    r.at(0, 3) = t.x;
    r.at(1, 3) = t.y;
    r.at(2, 3) = t.z;
    return r;
}

Mat4 scale(const Vec3& s) {
    Mat4 r;
    r.at(0, 0) = s.x;
    r.at(1, 1) = s.y;
    r.at(2, 2) = s.z;
    return r;
}

Mat4 rotate_y(float a) {
    Mat4 r;
    float c = std::cos(a), s = std::sin(a);
    r.at(0, 0) = c;
    r.at(0, 2) = s;
    r.at(2, 0) = -s;
    r.at(2, 2) = c;
    return r;
}

Mat4 rotate_x(float a) {
    Mat4 r;
    float c = std::cos(a), s = std::sin(a);
    r.at(1, 1) = c;
    r.at(1, 2) = -s;
    r.at(2, 1) = s;
    r.at(2, 2) = c;
    return r;
}

Mat4 rotate_z(float a) {
    Mat4 r;
    float c = std::cos(a), s = std::sin(a);
    r.at(0, 0) = c;
    r.at(0, 1) = -s;
    r.at(1, 0) = s;
    r.at(1, 1) = c;
    return r;
}

Mat4 rotate_quat(float w, float x, float y, float z) {
    Mat4 r;
    r.at(0, 0) = 1 - 2 * (y * y + z * z);
    r.at(0, 1) = 2 * (x * y - w * z);
    r.at(0, 2) = 2 * (x * z + w * y);
    r.at(1, 0) = 2 * (x * y + w * z);
    r.at(1, 1) = 1 - 2 * (x * x + z * z);
    r.at(1, 2) = 2 * (y * z - w * x);
    r.at(2, 0) = 2 * (x * z - w * y);
    r.at(2, 1) = 2 * (y * z + w * x);
    r.at(2, 2) = 1 - 2 * (x * x + y * y);
    return r;
}

Mat4 perspective(float fovy, float aspect, float zn, float zf) {
    Mat4 r;
    float f = 1.0f / std::tan(fovy * 0.5f);
    r.m.fill(0);
    r.at(0, 0) = f / aspect;
    r.at(1, 1) = f;
    r.at(2, 2) = zn / (zf - zn); // reversed depth: z / w = 1 at -zn, 0 at -zf
    r.at(2, 3) = zn * zf / (zf - zn);
    r.at(3, 2) = -1;
    return r;
}

Mat4 frustum(float tl, float tr, float td, float tu, float zn, float zf) {
    Mat4 r;
    r.m.fill(0);
    r.at(0, 0) = 2.0f / (tr - tl);
    r.at(0, 2) = (tr + tl) / (tr - tl);
    r.at(1, 1) = 2.0f / (tu - td);
    r.at(1, 2) = (tu + td) / (tu - td);
    r.at(2, 2) = zn / (zf - zn); // reversed depth: z / w = 1 at -zn, 0 at -zf
    r.at(2, 3) = zn * zf / (zf - zn);
    r.at(3, 2) = -1;
    return r;
}

Mat4 look_at(const Vec3& eye, const Vec3& target, const Vec3& up) {
    Vec3 f = normalize(target - eye);
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);
    Mat4 r;
    r.at(0, 0) = s.x;
    r.at(0, 1) = s.y;
    r.at(0, 2) = s.z;
    r.at(1, 0) = u.x;
    r.at(1, 1) = u.y;
    r.at(1, 2) = u.z;
    r.at(2, 0) = -f.x;
    r.at(2, 1) = -f.y;
    r.at(2, 2) = -f.z;
    r.at(0, 3) = -dot(s, eye);
    r.at(1, 3) = -dot(u, eye);
    r.at(2, 3) = dot(f, eye);
    return r;
}

Mat4 transpose(const Mat4& a) {
    Mat4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) r.at(i, j) = a.at(j, i);
    return r;
}

Mat4 inverse(const Mat4& a) {
    // Gauss-Jordan elimination with partial pivoting.
    float w[4][8];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            w[i][j] = a.at(i, j);
            w[i][j + 4] = i == j ? 1.0f : 0.0f;
        }
    for (int c = 0; c < 4; ++c) {
        int p = c;
        for (int r = c + 1; r < 4; ++r)
            if (std::fabs(w[r][c]) > std::fabs(w[p][c])) p = r;
        if (std::fabs(w[p][c]) < 1e-12f) return Mat4::identity();
        if (p != c)
            for (int j = 0; j < 8; ++j) std::swap(w[p][j], w[c][j]);
        float inv = 1.0f / w[c][c];
        for (int j = 0; j < 8; ++j) w[c][j] *= inv;
        for (int r = 0; r < 4; ++r) {
            if (r == c) continue;
            float f = w[r][c];
            if (f != 0)
                for (int j = 0; j < 8; ++j) w[r][j] -= f * w[c][j];
        }
    }
    Mat4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) r.at(i, j) = w[i][j + 4];
    return r;
}

} // namespace gt3r::render
