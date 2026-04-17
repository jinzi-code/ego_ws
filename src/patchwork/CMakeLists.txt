cmake_minimum_required(VERSION 3.8)
project(patchwork)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED COMPONENTS filesystem)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Node.msg"
        "msg/GroundEstimate.msg"
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

include_directories(
        ${PCL_INCLUDE_DIRS}
        include
)

add_executable(ros_kitti nodes/ros_kitti.cpp)
target_link_libraries(ros_kitti ${PCL_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(ros_kitti rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge pcl_conversions laser_geometry)
rosidl_target_interfaces(ros_kitti ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_for_legoloam nodes/pub_for_legoloam.cpp)
target_link_libraries(pub_for_legoloam ${PCL_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(pub_for_legoloam rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge pcl_conversions laser_geometry)
rosidl_target_interfaces(pub_for_legoloam ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(offline_kitti nodes/offline_kitti.cpp)
target_link_libraries(offline_kitti ${PCL_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(offline_kitti rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge pcl_conversions laser_geometry)
rosidl_target_interfaces(offline_kitti ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(offline_own_data nodes/offline_own_data.cpp)
target_link_libraries(offline_own_data ${PCL_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(offline_own_data rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge pcl_conversions laser_geometry)
rosidl_target_interfaces(offline_own_data ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
        ros_kitti
        pub_for_legoloam
        offline_kitti
        offline_own_data
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
