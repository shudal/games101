#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {
    axis.normalize();

    Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();
    angle = MY_PI * (angle / 180);

    Eigen::Matrix3f tmpone = std::cos(angle) * Eigen::Matrix3f::Identity()
            + (1-cos(angle)) * axis * axis.transpose();

    Eigen::Matrix3f tmpthree;
    tmpthree<< 0,-1 * axis.z(), axis.y(),
                axis.z(),0,-1*axis.x(),
                -1*axis.y(),axis.x(),0;
    tmpthree = std::sin(angle) * tmpthree;

    tmpone += tmpthree;

    rot.block<3,3>(0,0) = tmpone;
    return rot;
}
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

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Vector3f axis(0,0,1);
    return get_rotation(axis,rotation_angle);
    rotation_angle = MY_PI * (rotation_angle/180);

    model << std::cos(rotation_angle), -1.0 * std::sin(rotation_angle), 0, 0,
            std::sin(rotation_angle), std::cos(rotation_angle), 0, 0,
            0,0,1,0,
            0,0,0,1;
     return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    eye_fov = MY_PI * (eye_fov / 180 );
    float l,r,t,b,n,f;
    t = std::abs(zNear) * std::tan(eye_fov / 2.0);
    b = -1.0 * t;
    r = t * aspect_ratio;
    l = -1.0 * t;
    n = zNear;
    f = zFar;

    Eigen::Matrix4f persp,p2o,ortho, tmp;
    p2o <<  n,0,0,0,
            0,n,0,0,
            0,0,n+f, -1.0*n*f,
            0,0,1,0;
    tmp << 1,0,0,-1.0*(r+l)/2,
           0,1,0,-1.0*(t+b)/2,
           0,0,1,-1.0*(n+f)/2,
           0,0,0,1;
    ortho<< 1.0/r,0,0,0,
            0,1.0/t,0,0,
            0,0,2.0/(n-f),0,
            0,0,0,1.0;
    ortho = ortho * tmp;
    persp = ortho * p2o;
    projection = persp;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

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
        else
            return 0;
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

        r.set_model(get_model_matrix(angle));
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
