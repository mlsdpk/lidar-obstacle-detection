cmake_minimum_required(VERSION 3.5)
project(pointcloud_fusion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

# manually adding files for now
# later update it to SHARED LIB and components
add_executable(pointcloud_fusion_node 
  src/pointcloud_fusion_node.cpp
  src/pointcloud_fusion.cpp
  include/pointcloud_fusion/pointcloud_fusion.hpp
)

target_include_directories(pointcloud_fusion_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pointcloud_fusion_node 
  rclcpp
  message_filters
  sensor_msgs
)

install(TARGETS
  pointcloud_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
