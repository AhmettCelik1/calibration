cmake_minimum_required(VERSION 3.0)
project(main)  # Replace 'MatrixCalculation' with your preferred project name

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen package
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_geometry_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
# Add the executable
add_executable(main main.cpp)  # Replace 'main.cpp' with the name of your C++ file

# Link Eigen library
target_link_libraries(main Eigen3::Eigen ${catkin_LIBRARIES})
