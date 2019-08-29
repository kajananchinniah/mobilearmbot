cmake_minimum_required(VERSION 2.8.3)
project(mobilearmbot_teleop)

#Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	)

catkin_package(
	CATKIN_DEPENDS
	   roscpp
	   geometry_msgs
	)

find_package(Curses REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mobilearmbot_teleop_key src/teleop.cpp)
target_link_libraries(mobilearmbot_teleop_key ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

