cmake_minimum_required(VERSION 3.8)
project(fitrobotcpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(fitrobot_interfaces REQUIRED)
# find_package(nlohmann_json REQUIRED)

# add_executable(button src/button.cpp) target_link_libraries(button
# pigpiod_if2) ament_target_dependencies(button rclcpp std_msgs)

include(FetchContent)
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  # GIT_TAG v3.7.3
  GIT_TAG v3.9.1 # or whatever the latest version is
)
set(JSON_BuildTests
    OFF
    CACHE INTERNAL ""
)
FetchContent_MakeAvailable(json)

# This section will create an executable for the node
add_executable(check_robot_status src/check_robot_status.cpp)
ament_target_dependencies(
  check_robot_status
  rclcpp
  tf2_ros
  std_srvs
  fitrobot_interfaces
  lifecycle_msgs
  nav2_msgs
)

add_executable(master_node src/master_node.cpp)
ament_target_dependencies(
  master_node
  rclcpp
  tf2
  tf2_ros
  std_msgs
  action_msgs
  nav2_msgs
  lifecycle_msgs
  std_srvs
  fitrobot_interfaces
)

add_executable(master_wpf src/master_node_wpf.cpp)
ament_target_dependencies(
  master_wpf
  rclcpp
  tf2
  tf2_ros
  std_msgs
  action_msgs
  nav2_msgs
  lifecycle_msgs
  std_srvs
  fitrobot_interfaces
  # nlohmann_json
)
target_link_libraries(master_wpf nlohmann_json::nlohmann_json)

# === legacy trial-and-error code ===
# add_library(basic_navigator_lib SHARED src/robot_navigator.cpp )
# ament_target_dependencies(basic_navigator_lib rclcpp std_srvs rclcpp_action
# nav2_msgs) add_executable(waypoint_follower src/waypoint_follower.cpp)
# ament_target_dependencies(waypoint_follower rclcpp tf2_ros std_srvs
# fitrobot_interfaces nav2_msgs)
#
# add_executable(parse_station src/parse_station.cpp)
# ament_target_dependencies(parse_station ament_index_cpp rclcpp nav2_msgs
# fitrobot_interfaces) target_link_libraries(parse_station
# nlohmann_json::nlohmann_json)

# add_executable(nav2_action_client src/nav2_action_client.cpp)
# ament_target_dependencies( nav2_action_client rclcpp tf2 tf2_ros
# tf2_geometry_msgs std_srvs nav2_msgs fitrobot_interfaces )

# Linking Library: Link this library with any executable that needs
# BasicNavigator. target_link_libraries(waypoint_follower basic_navigator_lib)

include_directories(
  include
  # ${fitrobot_interfaces_INCLUDE_DIRS}  # This line should typically be
  # unnecessary
)

install(DIRECTORY include/ DESTINATION include)
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS master_node check_robot_status master_wpf
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
