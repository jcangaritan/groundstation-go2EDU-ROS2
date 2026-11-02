cmake_minimum_required(VERSION 3.8)
project(go2_description)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(joint_bridge src/joint_bridge.cpp)
ament_target_dependencies(joint_bridge rclcpp sensor_msgs unitree_go)

install(TARGETS joint_bridge
	DESTINATION lib/${PROJECT_NAME})
ament_package()
