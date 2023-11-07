cmake_minimum_required(VERSION 3.0)
project(test)  # Replace 'MatrixCalculation' with your preferred project name

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen package
find_package(Eigen3 REQUIRED)


# Add the executable
add_executable(test main.cpp)  # Replace 'main.cpp' with the name of your C++ file

# Link Eigen library
target_link_libraries(test Eigen3::Eigen)
