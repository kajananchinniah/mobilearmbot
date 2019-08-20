cmake_minimum_required(VERSION 2.8.3)
project(mobilearmbot_demos)


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	sensor_msgs
	moveit_core
	moveit_ros_planning_interface
	moveit_ros_move_group)

catkin_package(
	CATKIN_DEPENDS
	   roscpp
	   sensor_msgs
	   geometry_msgs
	   moveit_core
	   moveit_ros_planning_interface
	   moveit_ros_move_group
	)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(MobileArmBotDrive src/MobileArmBotDrive_node.cpp src/MobileArmBotDrive.cpp)
target_link_libraries(MobileArmBotDrive ${catkin_LIBRARIES})

add_executable(MobileArmBotArm src/MobileArmBotArm_node.cpp src/MobileArmBotArm.cpp)
target_link_libraries(MobileArmBotArm ${catkin_LIBRARIES})
