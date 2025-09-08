cmake_minimum_required(VERSION 3.8)
project(decompressed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pointcloud_compressor REQUIRED)

# Create library for pattern dictionary decompressor
add_library(pattern_dictionary_decompressor
  src/pattern_dictionary_decompressor.cpp
)
target_include_directories(pattern_dictionary_decompressor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pattern_dictionary_decompressor
  rclcpp
  std_msgs
  geometry_msgs
  pointcloud_compressor
)

# Create library for marker visualizer
add_library(marker_visualizer
  src/marker_visualizer.cpp
)
target_include_directories(marker_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(marker_visualizer
  pattern_dictionary_decompressor
)
ament_target_dependencies(marker_visualizer
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  pointcloud_compressor
)

# Create library for decompressed viewer node
add_library(decompressed_viewer_node_lib
  src/decompressed_viewer_node.cpp
)
target_include_directories(decompressed_viewer_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(decompressed_viewer_node_lib
  pattern_dictionary_decompressor
  marker_visualizer
)
ament_target_dependencies(decompressed_viewer_node_lib
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  pointcloud_compressor
)

# Create executable
add_executable(decompressed_viewer_node
  src/decompressed_viewer_main.cpp
)
target_link_libraries(decompressed_viewer_node
  decompressed_viewer_node_lib
)
ament_target_dependencies(decompressed_viewer_node
  rclcpp
)

# Install executables
install(TARGETS
  decompressed_viewer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  pattern_dictionary_decompressor
  marker_visualizer
  decompressed_viewer_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export libraries
ament_export_include_directories(include)
ament_export_libraries(
  pattern_dictionary_decompressor
  marker_visualizer
  decompressed_viewer_node_lib
)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  pointcloud_compressor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add Google Test
  find_package(ament_cmake_gtest REQUIRED)
  
  # Test for pattern dictionary decompressor
  ament_add_gtest(test_pattern_dictionary_decompressor
    test/test_pattern_dictionary_decompressor.cpp
  )
  target_link_libraries(test_pattern_dictionary_decompressor
    pattern_dictionary_decompressor
  )
  ament_target_dependencies(test_pattern_dictionary_decompressor
    rclcpp
    std_msgs
    geometry_msgs
    pointcloud_compressor
  )
  
  # Test for marker visualizer
  ament_add_gtest(test_marker_visualizer
    test/test_marker_visualizer.cpp
  )
  target_link_libraries(test_marker_visualizer
    marker_visualizer
    pattern_dictionary_decompressor
  )
  ament_target_dependencies(test_marker_visualizer
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    pointcloud_compressor
  )
endif()

ament_package()
