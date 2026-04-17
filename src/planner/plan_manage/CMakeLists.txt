cmake_minimum_required(VERSION 3.8)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(mpc_controller REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/DataDisp.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

include_directories(
  include
  SYSTEM
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../qpOASES/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)
ament_target_dependencies(ego_planner_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs cv_bridge tf2 tf2_ros tf2_geometry_msgs plan_env path_searching bspline_opt traj_utils mpc_controller)
rosidl_target_interfaces(ego_planner_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(traj_server src/traj_server.cpp)
find_library(QPOASES_LIB qpOASES PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../qpOASES/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../install/qpOASES/lib
  /usr/lib
  /usr/local/lib
)
if(NOT QPOASES_LIB)
  message(FATAL_ERROR "qpOASES library not found for traj_server")
endif()
ament_target_dependencies(traj_server rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs cv_bridge tf2 tf2_ros tf2_geometry_msgs plan_env path_searching bspline_opt traj_utils mpc_controller)
target_link_libraries(traj_server ${QPOASES_LIB})
rosidl_target_interfaces(traj_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(cmd_vel_bridge_node src/cmd_vel_bridge_node.cpp)
ament_target_dependencies(cmd_vel_bridge_node rclcpp geometry_msgs)
rosidl_target_interfaces(cmd_vel_bridge_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(safety_stop_guard_node src/safety_stop_guard_node.cpp)
ament_target_dependencies(safety_stop_guard_node rclcpp geometry_msgs std_msgs)
rosidl_target_interfaces(safety_stop_guard_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  ego_planner_node
  traj_server
  cmd_vel_bridge_node
  safety_stop_guard_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()


