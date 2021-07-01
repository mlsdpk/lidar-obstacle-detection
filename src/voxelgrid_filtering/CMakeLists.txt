cmake_minimum_required(VERSION 3.5)
project(voxelgrid_filtering)

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

# manually adding files for now
# later update it to SHARED LIB and components
add_executable(voxelgrid_filtering_node 
  src/voxelgrid_filtering_node.cpp
  src/voxelgrid_filtering.cpp
  include/voxelgrid_filtering/voxelgrid_filtering.hpp
)

target_include_directories(voxelgrid_filtering_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(voxelgrid_filtering_node 
  rclcpp
)

ament_package()
