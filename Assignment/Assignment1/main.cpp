#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float angle_red = rotation_angle * MY_PI / 180.0f;

    model << std::cos(angle_red), -std::sin(angle_red), 0, 0,
        std::sin(angle_red), std::cos(angle_red), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) 
{
    // 1. 角度转弧度
    float angle_rad = angle * MY_PI / 180.0f;

    // 2. 轴向量单位化
    Vector3f n = axis.normalized();

    // 3. 构建 3x3 旋转矩阵 R
    Eigen::Matrix3f R;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N; // 叉积矩阵
    N << 0, -n.z(), n.y(),
        n.z(), 0, -n.x(),
        -n.y(), n.x(), 0;

    R = std::cos(angle_rad) * I +
        (1 - std::cos(angle_rad)) * n * n.transpose() +
        std::sin(angle_rad) * N;

    // 4. 将 3x3 旋转矩阵扩展为 4x4 变换矩阵
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    model.block<3, 3>(0, 0) = R;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // 将垂直视场角转换为弧度
    float fov_rad = eye_fov * MY_PI / 180.0f;

    float t = std::tan(fov_rad / 2.0f) * std::abs(zNear);
    float r = t * aspect_ratio;

    // 构建透视-正交变换矩阵
    Eigen::Matrix4f persp_to_ortho;
    persp_to_ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    // 构建正交投影矩阵
    Eigen::Matrix4f ortho;
    ortho << 1.0f / r, 0, 0, 0,
        0, 1.0f / t, 0, 0,
        0, 0, 2.0f / (zNear - zFar), 0,
        0, 0, 0, 1;

    // 最终的投影矩阵是两者的乘积
    projection = ortho * persp_to_ortho;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(Eigen::Vector3f(0, 1, 0),angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
