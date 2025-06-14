//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include <algorithm>
#include <array>

Triangle::Triangle()
{
    v[0] << 0, 0, 0;
    v[1] << 0, 0, 0;
    v[2] << 0, 0, 0;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

void Triangle::setVertex(int ind, Vector3f ver)
{
    v[ind] = ver;
}
void Triangle::setNormal(int ind, Vector3f n)
{
    normal[ind] = n;
}
void Triangle::setColor(int ind, float r, float g, float b)
{
    if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) || (b > 255.)) {
        fprintf(stderr, "ERROR! Invalid color values");
        fflush(stderr);
        exit(-1);
    }

    color[ind] = Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
    return;
}
void Triangle::setTexCoord(int ind, float s, float t)
{
    tex_coords[ind] = Vector2f(s, t);
}

std::array<Vector4f, 3> Triangle::toVector4() const
{
    std::array<Eigen::Vector4f, 3> res;
    // 将v的所有元素经lambda表达式的变换后填充到res内
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) { return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f); });
    return res;
}

bool Triangle::isInside(float x, float y) const
{
    Vector3f p0 = Vector3f(v[0].x(), v[0].y(), 0);
    Vector3f p1 = Vector3f(v[1].x(), v[1].y(), 0);
    Vector3f p2 = Vector3f(v[2].x(), v[2].y(), 0);

    Vector3f p = Vector3f(x, y, 0);

    Vector3f v01 = p1 - p0;
    Vector3f v12 = p2 - p1;
    Vector3f v20 = p0 - p2;
    Vector3f v0p = p - p0;
    Vector3f v1p = p - p1;
    Vector3f v2p = p - p2;

    Vector3f n01 = v01.cross(v0p);
    Vector3f n12 = v12.cross(v1p);
    Vector3f n20 = v20.cross(v2p);

    if (n01.dot(n12) > 0 && n01.dot(n20) > 0) {
        return true;
    }
    return false;
}