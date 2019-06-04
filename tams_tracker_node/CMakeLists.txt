cmake_minimum_required(VERSION 2.9.3)
project(tams_tracker_node)

set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

#add_compile_options($<$<CONFIG:Release>:-O3>)
#add_compile_options($<$<CONFIG:Release>:-O0>)
#add_compile_options($<$<CONFIG:Release>:-ffast-math>)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
  roscpp
  std_srvs
  visualization_msgs
  interactive_markers
  image_transport
  sensor_msgs
  cv_bridge
  tams_tracker_msgs
  eigen_conversions
  rosbag
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf
  tf_conversions
  sensor_msgs
  cv_bridge
  actionlib_msgs
  image_transport
  tams_tracker_msgs
  eigen_conversions
  rosbag
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS program_options)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_DONT_VECTORIZE")

add_executable(tams_tracker_node src/tracker.cpp)
add_dependencies(tams_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tams_tracker_node yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(tams_tracker_calibrate src/calibrate.cpp)
add_dependencies(tams_tracker_calibrate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tams_tracker_calibrate yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(tams_tracker_record src/record.cpp)
add_dependencies(tams_tracker_record ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tams_tracker_record yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(tams_tracker_reconstruct src/reconstruct.cpp)
add_dependencies(tams_tracker_reconstruct ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tams_tracker_reconstruct yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(tams_tracker_align_imu src/align_imu.cpp)
add_dependencies(tams_tracker_align_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tams_tracker_align_imu yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(tams_tracker_publish_angles src/publish_angles.cpp)
add_dependencies(tams_tracker_publish_angles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tams_tracker_publish_angles yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(tams_tracker_filter src/filter.cpp)
add_dependencies(tams_tracker_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tams_tracker_filter yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
