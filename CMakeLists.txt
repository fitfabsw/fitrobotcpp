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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fitrobot_interfaces REQUIRED)

# This section will create an executable for the node
add_executable(check_tf_service src/check_tf_service.cpp)
add_executable(check_robot_status src/check_robot_status.cpp)
ament_target_dependencies(check_tf_service rclcpp tf2_ros std_srvs fitrobot_interfaces)
ament_target_dependencies(check_robot_status rclcpp tf2_ros std_srvs fitrobot_interfaces)


# include_directories(
#   include
#   ${fitrobot_interfaces_INCLUDE_DIRS}
# )

include_directories(
  include
  ${fitrobot_interfaces_INCLUDE_DIRS}  # This line should typically be unnecessary
)

install(TARGETS
  check_tf_service check_robot_status
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
