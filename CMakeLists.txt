cmake_minimum_required(VERSION 3.8)
project(px4_msgs_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(include)

# Create converter library
add_library(converter
  src/converter.cpp
)

ament_target_dependencies(converter
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
)

# Create bridge node executable
add_executable(vehicle_pose_bridge
  src/bridge_node.cpp
)

target_link_libraries(vehicle_pose_bridge converter)

ament_target_dependencies(vehicle_pose_bridge
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
)

# Create VIO bridge node executable
add_executable(vio_bridge_node
  src/vio_bridge_node.cpp
)

ament_target_dependencies(vio_bridge_node
  rclcpp
  px4_msgs
  nav_msgs
)

# Install targets
install(TARGETS
  vehicle_pose_bridge
  vio_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  converter
  DESTINATION lib
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
