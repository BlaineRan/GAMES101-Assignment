//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"

#include <math.h>

#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

rst::pos_buf_id
rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id
rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

rst::col_buf_id
rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool
insideTriangle(int x, int y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the
    // triangle represented by _v[0], _v[1], _v[2]
}

static std::tuple<float, float, float>
computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1, c2, c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer,
    ind_buf_id ind_buffer,
    col_buf_id col_buffer,
    Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind) {
        Triangle t;
        Eigen::Vector4f v[] = { mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f) };
        // Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        // Viewport transformation
        for (auto& vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

Eigen::Vector3f fadeColor(Eigen::Vector3f originalColor, float rate)
{
    Eigen::Vector3f res = (1 - rate) * originalColor + rate * Eigen::Vector3f(0, 0, 0);
    return res;
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();

    int min_x = static_cast<int>(std::min({ v[0].x(), v[1].x(), v[2].x() }));
    int max_x = static_cast<int>(std::ceil(std::max({ v[0].x(), v[1].x(), v[2].x() })));
    int min_y = static_cast<int>(std::min({ v[0].y(), v[1].y(), v[2].y() }));
    int max_y = static_cast<int>(std::ceil(std::max({ v[0].y(), v[1].y(), v[2].y() })));
    std::vector<std::pair<float, float>> diff = {
        { 0.25, 0.25 },
        { 0.25, 0.75 },
        { 0.75, 0.25 },
        { 0.75, 0.75 }
    };
    for (int i = min_x; i <= max_x; i++) {
        for (int j = min_y; j <= max_y; j++) {
            float count = 0;
            for (int k = 0; k < 4; k++) {
                float x = i + diff[k].first;
                float y = j + diff[k].second;
                if(t.isInside(x,y)) count++;
            }
            if(count == 0) continue;

            float x = i + 0.5;
            float y = j + 0.5;

            auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            int index = get_index(i, j);
            float nowZ = depth_buf[index];
            if (z_interpolated < nowZ) {
                depth_buf[index] = z_interpolated;

                float fadeRate =1 - (count/4.0f);
                Eigen::Vector3f newColor = fadeColor(t.getColor(),fadeRate);

                frame_buf[index] = newColor;
                this->set_pixel(Vector3f(i, j, 1), newColor);
            }
        }
    }
}
// Screen space rasterization
// void rst::rasterizer::rasterize_triangle(const Triangle& t)
// {
//     auto v = t.toVector4();

//     int min_x = static_cast<int>(std::min({ v[0].x(), v[1].x(), v[2].x() }));
//     int max_x = static_cast<int>(std::ceil(std::max({ v[0].x(), v[1].x(), v[2].x() })));
//     int min_y = static_cast<int>(std::min({ v[0].y(), v[1].y(), v[2].y() }));
//     int max_y = static_cast<int>(std::ceil(std::max({ v[0].y(), v[1].y(), v[2].y() })));
//     std::vector<std::pair<float, float>> diff = {
//         { 0.25, 0.25 },
//         { 0.25, 0.75 },
//         { 0.75, 0.25 },
//         { 0.75, 0.75 }
//     };
//     for (int i = min_x; i <= max_x; i++) {
//         for (int j = min_y; j <= max_y; j++) {
//             for (int k = 0; k < 4; k++) {
//                 float x = i + diff[k].first;
//                 float y = j + diff[k].second;
//                 int ind = sample_index(i, j, k);

//                 if (!t.isInside(x, y))
//                     continue;

//                 auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
//                 float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//                 float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//                 z_interpolated *= w_reciprocal;

//                 if (z_interpolated < sample_depth_buf[ind]) {
//                     sample_depth_buf[ind] = z_interpolated;
//                     sample_color_buf[ind] = t.getColor();
//                 }
//             }
//         }
//     }
//     resolve_to_framebuf();
// }

void rst::rasterizer::resolve_to_framebuf()
{
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            Eigen::Vector3f sum = { 0, 0, 0 };
            for (int s = 0; s < SAMPLE_CNT; ++s)
                sum += sample_color_buf[sample_index(x, y, s)];
            frame_buf[get_index(x, y)] = sum / SAMPLE_CNT;
        }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f { 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(),
            depth_buf.end(),
            std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h)
    : width(w)
    , height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    sample_depth_buf.assign(w * h * SAMPLE_CNT, std::numeric_limits<float>::infinity());
    sample_color_buf.assign(w * h * SAMPLE_CNT, {0,0,0});
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

int rst::rasterizer::sample_index(int x, int y, int s) // s ∈ [0,3]
{
    return (y * width + x) * SAMPLE_CNT + s;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point,
    const Eigen::Vector3f& color)
{
    // old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}
