cmake_minimum_required(VERSION 3.5)
project(drone)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)

include_directories(include)

add_library(drone_controller_nodes SHARED
            src/drone_controller_node.cpp
            src/odom_broadcaster_node.cpp
            src/takeoff_action_server.cpp
            src/land_action_server.cpp)
target_compile_definitions(drone_controller_nodes
  PRIVATE "DRONE_DLL")
ament_target_dependencies(drone_controller_nodes 
  rclcpp 
  rclcpp_components 
  rclcpp_action
  std_msgs 
  tf2 
  tf2_ros 
  tf2_msgs
  px4_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  drone_interfaces)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(drone_controller src/drone_controller.cpp)
target_link_libraries(drone_controller drone_controller_nodes)
ament_target_dependencies(drone_controller
  rclcpp tf2 tf2_ros tf2_msgs px4_msgs nav_msgs sensor_msgs geometry_msgs)

add_executable(odom_broadcaster src/odom_broadcaster.cpp)
target_link_libraries(odom_broadcaster drone_controller_nodes)
ament_target_dependencies(odom_broadcaster
  rclcpp tf2 tf2_ros tf2_msgs)

add_executable(takeoff_server src/takeoff_action.cpp)
target_link_libraries(takeoff_server drone_controller_nodes)
ament_target_dependencies(takeoff_server
  rclcpp rclcpp_action tf2 tf2_ros tf2_msgs px4_msgs geometry_msgs drone_interfaces )

add_executable(land_server src/land_action.cpp)
target_link_libraries(land_server drone_controller_nodes)
ament_target_dependencies(land_server
  rclcpp rclcpp_action tf2 tf2_ros tf2_msgs px4_msgs geometry_msgs drone_interfaces )

install(TARGETS
  drone_controller_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  drone_controller
  odom_broadcaster  
  takeoff_server
  land_server
  DESTINATION lib/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
