cmake_minimum_required(VERSION 3.8)
project(robinlab_go2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

add_executable(cruise_teleop src/cruise_teleop_node.cpp)
target_include_directories(cruise_teleop PRIVATE include)
ament_target_dependencies(cruise_teleop rclcpp geometry_msgs unitree_go)

# --- 3) /cmd_vel -> Unitree Sport -----------------------------------------
# Compílalo igual que ya compilabas uni_sport.cpp: ros2_sport_client.h/.cpp vienen de los ejemplos
# de unitree_ros2. Ajusta SPORT_CLIENT_DIR a donde los tengas.

set(SPORT_CLIENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_common)
add_executable(cmd_vel_bridge
  src/cmd_vel_bridge_node.cpp
  ${SPORT_CLIENT_DIR}/common/ros2_sport_client.cpp)
target_include_directories(cmd_vel_bridge PRIVATE include ${SPORT_CLIENT_DIR})
ament_target_dependencies(cmd_vel_bridge rclcpp geometry_msgs unitree_api)

install(TARGETS cruise_teleop cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
