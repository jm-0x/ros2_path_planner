cmake_minimum_required(VERSION 3.5)
project(path_planner)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lfs_msgs REQUIRED)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Declare executable
add_executable(path_planner_node src/path_planner_node.cpp)

# Link libraries
ament_target_dependencies(path_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  lfs_msgs
  Eigen3
)

# Install targets
install(TARGETS
  path_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# ament package macro
ament_package()

