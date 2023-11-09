#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

int main()
{

    int t_pixel_x = 200;
    int t_pixel_y = 200;

    double t_x_position = 2.1;
    double t_y_position = 4.1;
    double t_z_position = 1.1;

    Eigen::Matrix<double, 3, 3> camera_intrinsics;
    camera_intrinsics << 640.7934716413195, 0.0, 600.0322069895611, 0.0, 638.1580301012906, 373.0846095138255, 0.0, 0.0, 1.0;

    Eigen::Matrix<double, 4, 1> world_point;
    world_point << t_x_position, t_y_position, t_z_position, 1;

    Eigen::Matrix<double, 3, 1> pixel_coordinates;
    pixel_coordinates << t_pixel_x, t_pixel_y, 1;

    Eigen::Matrix<double, 3, 1> A;
    A = camera_intrinsics.inverse() * pixel_coordinates;
    std::cout << "Matrix A:" << std::endl;
    std::cout << A << std::endl;

    Eigen::Matrix<double, 1, 4> pseudoInvworldPoint = (world_point.transpose() * world_point).inverse() * world_point.transpose();

    Eigen::Matrix<double, 3, 4> B = A * pseudoInvworldPoint;

    std::cout << "Matrix B:" << std::endl;
    std::cout << B << std::endl;

    Eigen::Matrix<double, 3, 1> D = B * world_point;
    std::cout << "Matrix D:" << std::endl;
    std::cout << D << std::endl;

    const tf2::Matrix3x3 rotation_matrix_tf(B(0, 0), B(0, 1), B(0, 2), B(1, 0), B(1, 1), B(1, 2), B(2, 0), B(2, 1), B(2, 2));
    Eigen::Matrix<double, 3, 3> rotation_matrix;
    rotation_matrix << B(0, 0), B(0, 1), B(0, 2), B(1, 0), B(1, 1), B(1, 2), B(2, 0), B(2, 1), B(2, 2);
    std::cout << "Rotation Matrix:" << std::endl;
    std::cout << rotation_matrix << std::endl;

    Eigen::Matrix<double, 3, 1> translation_vector;
    translation_vector << B(0, 3), B(1, 3), B(2, 3);
    std::cout << "Translation Vector:" << std::endl;
    std::cout << translation_vector << std::endl;

    double roll, pitch, yaw;
    rotation_matrix_tf.getRPY(roll, pitch, yaw);

    double x, y, z;
    x = translation_vector(0);
    y = translation_vector(1);
    z = translation_vector(2);

    std::cout << "x: " << x << " y: " << y << " z: " << z << " yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << std::endl;

    return 0;
}
