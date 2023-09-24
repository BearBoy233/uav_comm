cmake_minimum_required(VERSION 3.10.2)
project(commnet)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
# message_generation
  nav_msgs
  geometry_msgs
  mavros_msgs
  serial
  tf2_eigen
  
  cluster_msgs
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  nav_msgs
#  geometry_msgs
#  mavros_msgs
#  cluster_msgs
#)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES commnet
#  DEPENDS system_lib
  CATKIN_DEPENDS
	roscpp 
	message_runtime
	geometry_msgs
	serial
	std_msgs
	mavros_msgs

  cluster_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ros_udp_subtopic
add_executable(ros_udp_subtopic  src/ros_udp_subtopic.cpp)
add_dependencies(ros_udp_subtopic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_udp_subtopic  ${catkin_LIBRARIES} )

# ros_udp_pubtopic
add_executable(ros_udp_pubtopic  src/ros_udp_pubtopic.cpp)
add_dependencies(ros_udp_pubtopic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_udp_pubtopic  ${catkin_LIBRARIES} )

