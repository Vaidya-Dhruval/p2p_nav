cmake_minimum_required(VERSION 3.8)
project(p2p_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(p2p_navigation_interfaces REQUIRED)

# Ensure include directories are properly set
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${nav2_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${p2p_navigation_interfaces_INCLUDE_DIRS}
)

# Add server executable
add_executable(p2p_nav_server src/p2p_nav_server.cpp)
target_link_libraries(p2p_nav_server ${rclcpp_LIBRARIES} ${rclcpp_action_LIBRARIES} ${nav2_msgs_LIBRARIES})
ament_target_dependencies(p2p_nav_server
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  nav_msgs
  action_msgs
  p2p_navigation_interfaces
)

# Add client executable
add_executable(p2p_nav_client src/p2p_nav_client.cpp)
target_link_libraries(p2p_nav_client ${rclcpp_LIBRARIES} ${rclcpp_action_LIBRARIES} ${nav2_msgs_LIBRARIES})
ament_target_dependencies(p2p_nav_client
  rclcpp
  rclcpp_action
  geometry_msgs
  p2p_navigation_interfaces
  nav2_msgs
)

# Install executables
install(TARGETS
  p2p_nav_server
  p2p_nav_client
  DESTINATION lib/${PROJECT_NAME}
)

# Create a component shared library for reuse in component containers
add_library(p2p_nav_server_component SHARED src/p2p_nav_server.cpp)
target_compile_definitions(p2p_nav_server_component PRIVATE "P2P_NAVIGATION_BUILDING_DLL")
ament_target_dependencies(p2p_nav_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  nav_msgs
  action_msgs
  p2p_navigation_interfaces
)
rclcpp_components_register_node(p2p_nav_server_component
  PLUGIN "P2PNavigationActionServer"
  EXECUTABLE p2p_nav_server_node
)

# Install components
install(TARGETS
  p2p_nav_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Create the config directory even if we don't have config files yet
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()