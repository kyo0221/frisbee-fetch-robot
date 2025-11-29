cmake_minimum_required(VERSION 3.10)
project(frisbee_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(frisbee_detector_node
  src/frisbee_detector_node.cpp
  src/detector_core.cpp
)
ament_target_dependencies(
  frisbee_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  rcl_interfaces
  ament_index_cpp
)

add_executable(frisbee_chaser_node src/frisbee_chaser_node.cpp)
ament_target_dependencies(
  frisbee_chaser_node
  rclcpp
  geometry_msgs
  std_msgs
)

target_include_directories(frisbee_detector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(frisbee_chaser_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  frisbee_detector_node
  frisbee_chaser_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
  config
  resource
  DESTINATION share/${PROJECT_NAME})

ament_package()
