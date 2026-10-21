cmake_minimum_required(VERSION 3.8)
project(laika_media)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ⚡[DEP] FIND LINUX DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rmw_cyclonedds_cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# ⚡[DEP] FIND ROS2 DEPENDENCIES
set(ROS_DEP
	rclcpp
	sensor_msgs
	nav_msgs
	tf2
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
	pcl_ros
	pcl_conversions
	livox_ros_driver2
	unitree_api
	unitree_go
	std_srvs)

foreach(DEP ${ROS_DEP})
	find_package(${DEP} REQUIRED)
endforeach()

# 🔄 [NODES] SET CPP_VISION NODES
set(CPP_CV
	go2_cam)

# ▶ [NODES] COMPILE AND INSTALL VISION NODES
foreach(NODE_CV ${CPP_CV})
	add_executable(${NODE_CV} src/vision/${NODE_CV}.cpp)
	target_include_directories(${NODE_CV} PRIVATE ${OpenCV_INCLUDE_DIRS})
	target_link_libraries(${NODE_CV} ${OpenCV_LIBS})
	ament_target_dependencies(${NODE_CV} rclcpp sensor_msgs cv_bridge)
	install(TARGETS ${NODE_CV}
		DESTINATION lib/${PROJECT_NAME})
endforeach()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ✅ [AMENT] PACKAGE AMENT 
ament_package()


