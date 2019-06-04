cmake_minimum_required(VERSION 2.8.3)
project(tams_tracker_launch)

find_package(catkin REQUIRED COMPONENTS
	tams_tracker_node
	${MSG_DEPS}
)

catkin_package(
	CATKIN_DEPENDS
		tams_tracker_node
		${MSG_DEPS}
)
