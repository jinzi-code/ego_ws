cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

message(status "IPOPT_CFLAGS: " ${IPOPT_CFLAGS})
message(status "IPOPT_CFLAGS_OTHER: " ${IPOPT_CFLAGS_OTHER})
set(CMAKE_CXX_FLAGS "-DHAVE_CSTDDEF -DHAVE_MPI_INITIALIZED")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../../qpOASES/include
  /usr/local/include
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(/usr/local/lib)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(mpc_controller
  src/MPC.cpp
  include/MPC.hpp
)
find_library(QPOASES_LIB qpOASES PATHS
  ${PROJECT_SOURCE_DIR}/../../qpOASES/lib
  ${PROJECT_SOURCE_DIR}/../../../install/qpOASES/lib
  /usr/local/lib
  /usr/lib
)
if(QPOASES_LIB)
  target_link_libraries(mpc_controller ${QPOASES_LIB})
else()
  message(FATAL_ERROR "qpOASES library not found for mpc_controller")
endif()
ament_target_dependencies(mpc_controller rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs)

option(BUILD_MPC_EXAMPLES "Build MPC example/test executables" OFF)
if(BUILD_MPC_EXAMPLES)
  if(EXISTS /usr/local/include/cppad/ipopt/solve.hpp)
    add_executable(solver app/example.cpp)
    target_link_libraries(solver ipopt)
    install(TARGETS solver DESTINATION lib/${PROJECT_NAME})
  endif()

  add_executable(qp_test app/qp_test.cpp)
  target_link_libraries(qp_test libqpOASES.so)

  add_executable(mpc_test
    app/mpc_control_test.cpp
    src/uniform_bspline.cpp
    src/MPC.cpp
  )
  target_link_libraries(mpc_test libqpOASES.so)
  ament_target_dependencies(mpc_test rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs)

  install(TARGETS qp_test mpc_test DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS
  mpc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(mpc_controller)
ament_export_dependencies(rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs)
ament_package()
