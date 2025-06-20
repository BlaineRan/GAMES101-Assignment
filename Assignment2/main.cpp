
#include "Triangle.hpp"
#include "global.hpp"
#include "rasterizer.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    rotation_angle = (rotation_angle / 180) * MY_PI;
    Eigen::Matrix4f rotate;
    rotate << std::cos(rotation_angle), -std::sin(rotation_angle), 0, 0,
        std::sin(rotation_angle), std::cos(rotation_angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    model = model * rotate;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f m_proToOrth;
    m_proToOrth << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;
    float r, l, b, t;
    float eye_theta = (eye_fov / 180) * MY_PI;
    t = -zNear * std::tan(eye_theta / 2);
    // std::cout << "zNear is " << ((zNear > 0) ? "positive" : "negative") << std::endl;
    b = -t;
    r = t * aspect_ratio;
    l = -r;

    Eigen::Matrix4f m_orth,ortho_trans;
    m_orth << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;
    ortho_trans << 1, 0, 0, -(r + l) / 2.0f,
        0, 1, 0, -(t + b) / 2.0f,
        0, 0, 1, -(zNear + zFar) / 2.0f,
        0, 0, 0, 1;
    projection = m_orth * ortho_trans * m_proToOrth * projection;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2) {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Eigen::Vector3f> pos {
        { 2, 0, -2 },
        { 0, 2, -2 },
        { -2, 0, -2 },
        { 3.5, -1, -5 },
        { 2.5, 1.5, -5 },
        { -1, 0.5, -5 }
    };

    std::vector<Eigen::Vector3i> ind {
        { 0, 1, 2 },
        { 3, 4, 5 }
    };

    std::vector<Eigen::Vector3f> cols {
        { 217.0, 238.0, 185.0 },
        { 217.0, 238.0, 185.0 },
        { 217.0, 238.0, 185.0 },
        { 185.0, 217.0, 238.0 },
        { 185.0, 217.0, 238.0 },
        { 185.0, 217.0, 238.0 }
    };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on