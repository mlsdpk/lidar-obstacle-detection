cmake_minimum_required(VERSION 3.5)
project(planar_segmentation)

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
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL 1.11.1 REQUIRED QUIET COMPONENTS common core features filters io segmentation surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if( PCL_FOUND )
  message(STATUS "PCL found at ${PCL_LIBRARIES}.")
endif()

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  PCL
  pcl_conversions
)

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
)

set(NODE_LIB planar_segmentation_node)

add_library(${NODE_LIB} SHARED
  src/planar_segmentation.cpp
)

target_include_directories(${NODE_LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${NODE_LIB} 
  ${dependencies}
)

target_link_libraries(${NODE_LIB} ${PCL_LIBRARIES})

rclcpp_components_register_node(${NODE_LIB}
  PLUGIN "planar_segmentation::PlanarSegmentationNode"
  EXECUTABLE ${NODE_LIB}_exe
)

target_link_libraries(${NODE_LIB}_exe ${PCL_LIBRARIES})

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
ament_export_dependencies(${dependencies} ${PCL_LIBRARIES})
ament_export_targets(export_${NODE_LIB} HAS_LIBRARY_TARGET)
ament_package()
