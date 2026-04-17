cmake_minimum_required(VERSION 3.8)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
    SYSTEM
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(traj_utils
    src/planning_visualization.cpp
    src/polynomial_traj.cpp
)
ament_target_dependencies(traj_utils rclcpp std_msgs visualization_msgs cv_bridge bspline_opt path_searching)

install(TARGETS traj_utils
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(traj_utils)
ament_export_dependencies(rclcpp std_msgs visualization_msgs cv_bridge bspline_opt path_searching)
ament_package()
