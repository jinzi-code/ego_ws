cmake_minimum_required(VERSION 3.8)
project(path_searching)

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
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
    SYSTEM
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(path_searching
    src/dyn_a_star.cpp
)
ament_target_dependencies(path_searching rclcpp std_msgs visualization_msgs cv_bridge plan_env)

install(TARGETS path_searching
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(path_searching)
ament_export_dependencies(rclcpp std_msgs visualization_msgs cv_bridge plan_env)
ament_package()
