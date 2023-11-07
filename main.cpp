#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix<double, 3, 1> A;
    A << 2, 4, 6; 

    Eigen::Matrix<double, 4, 1> C; 
    C << 1, 3, 5, 7; 


    Eigen::Matrix<double, 1, 4> pseudoInvC = (C.transpose() * C).inverse() * C.transpose();


    Eigen::Matrix<double, 3, 4> B = A * pseudoInvC;


    std::cout << "Matrix B:" << std::endl;
    std::cout << B << std::endl;

    Eigen::Matrix<double, 3, 1> D = B * C; 

    std::cout << "Matrix D:" << std::endl;
    std::cout << D << std::endl;

    return 0;
}
