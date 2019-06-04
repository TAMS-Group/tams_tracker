cmake_minimum_required(VERSION 2.8.3)
project(tams_tracker_msgs)

set(MSG_DEPS
	geometry_msgs
	std_msgs
)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	${MSG_DEPS}
)

add_message_files(FILES
	MarkerArrayStamped.msg
	Marker.msg
	PeakArrayStamped.msg
	Peak.msg
	RayArrayStamped.msg
	Ray.msg
	RigidArrayStamped.msg
	Rigid.msg
	RigidStamped.msg
)

generate_messages(
	DEPENDENCIES
	${MSG_DEPS}
)

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
