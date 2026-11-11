cmake_minimum_required(VERSION 3.8)
project(robinlab_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

add_executable(convert_bag src/convert_bag.cpp)
ament_target_dependencies(convert_bag rclcpp sensor_msgs livox_ros_driver2 rosbag2_cpp rosbag2_storage)

install(TARGETS convert_bag
	DESTINATION lib/${PROJECT_NAME})

add_executable(gravity_alignment src/gravity_alignment.cpp)
target_include_directories(gravity_alignment PRIVATE include)
target_link_libraries(gravity_alignment Eigen3::Eigen Sophus::Sophus)
ament_target_dependencies(gravity_alignment rclcpp sensor_msgs geometry_msgs tf2_ros)

install(TARGETS gravity_alignment
	DESTINATION lib/${PROJECT_NAME})
ament_package()
