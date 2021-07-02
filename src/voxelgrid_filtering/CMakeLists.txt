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
find_package(rclcpp_components REQUIRED)

# set library name
set(NODE_LIB voxelgrid_filtering_node)

# build node as library
add_library(${NODE_LIB} SHARED
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
  rclcpp_components
)

rclcpp_components_register_node(${NODE_LIB}
  PLUGIN "voxelgrid_filtering::VoxelGridFilterNode"
  EXECUTABLE ${NODE_LIB}_exe
)

# install library
install(TARGETS ${NODE_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executable
install(TARGETS ${NODE_LIB}_exe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
