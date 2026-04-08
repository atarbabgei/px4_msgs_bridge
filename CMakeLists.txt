cmake_minimum_required(VERSION 3.8)
project(px4_msgs_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Shared library (composable node)
add_library(px4_bridge_lib SHARED
  src/px4_bridge.cpp
)

target_include_directories(px4_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

ament_target_dependencies(px4_bridge_lib
  rclcpp
  rclcpp_components
  px4_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

target_link_libraries(px4_bridge_lib Eigen3::Eigen)

# Register composable node
rclcpp_components_register_node(px4_bridge_lib
  PLUGIN "px4_bridge::Px4Bridge"
  EXECUTABLE px4_bridge_component
)

# Standalone executable
add_executable(bridge_node src/main.cpp)
target_link_libraries(bridge_node px4_bridge_lib)
ament_target_dependencies(bridge_node rclcpp)

# Install
install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS px4_bridge_lib
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
