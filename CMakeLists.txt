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
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Create shared bridge library
add_library(px4_bridge_lib
  src/bridge_base.cpp
  src/converter_base.cpp
  src/bridge_manager.cpp
  src/px4_to_ros_converter.cpp
  src/ros_to_px4_converter.cpp
)

# Set include directories for the library
target_include_directories(px4_bridge_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(px4_bridge_lib
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# Create unified bridge node executable
add_executable(bridge_node
  src/main.cpp
)

target_link_libraries(bridge_node px4_bridge_lib)

ament_target_dependencies(bridge_node
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# Install targets
install(TARGETS
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  px4_bridge_lib
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
