cmake_minimum_required(VERSION 2.8.3)
project(fwis_mpc_controller)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs tf2_ros tf2_geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OSQP_INCLUDE_DIRS}
  ${OSQPEIGEN_INCLUDE_DIRS}
)

add_executable(fwis_mpc_controller_node src/fwismpc.cpp src/fwis_mpc_controller_node.cpp)
target_link_libraries(fwis_mpc_controller_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  osqp::osqp
  OsqpEigen::OsqpEigen
  pthread
  dl
)
