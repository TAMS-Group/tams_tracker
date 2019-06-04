cmake_minimum_required(VERSION 2.9.3)
project(tams_tracker_imu)

set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  sensor_msgs
  eigen_conversions
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf
  tf_conversions
  sensor_msgs
  eigen_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tams_tracker_imu src/tams_tracker_imu.cpp)
add_dependencies(tams_tracker_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tams_tracker_imu ${catkin_LIBRARIES} ${Boost_LIBRARIES})
