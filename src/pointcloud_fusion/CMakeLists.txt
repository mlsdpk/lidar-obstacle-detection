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
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  message_filters
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set(NODE_LIB pointcloud_fusion_node)

add_library(${NODE_LIB} SHARED
  src/pointcloud_fusion.cpp
)

target_include_directories(pointcloud_fusion_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${NODE_LIB} 
  ${dependencies}
)

rclcpp_components_register_node(${NODE_LIB}
  PLUGIN "pointcloud_fusion::PointCloudFusionNode"
  EXECUTABLE ${NODE_LIB}_exe
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${NODE_LIB}
  EXPORT export_${NODE_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ${NODE_LIB}_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${NODE_LIB})
ament_export_dependencies(${dependencies})
ament_export_targets(export_${NODE_LIB} HAS_LIBRARY_TARGET)
ament_package()
