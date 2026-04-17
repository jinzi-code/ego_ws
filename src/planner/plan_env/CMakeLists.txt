cmake_minimum_required(VERSION 3.8)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
    SYSTEM
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env
    src/grid_map.cpp
    src/raycast.cpp
)
target_link_libraries(plan_env ${PCL_LIBRARIES})
ament_target_dependencies(plan_env rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs cv_bridge message_filters pcl_conversions)

install(TARGETS plan_env
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(plan_env)
ament_export_dependencies(rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs cv_bridge message_filters pcl_conversions)
ament_package()
