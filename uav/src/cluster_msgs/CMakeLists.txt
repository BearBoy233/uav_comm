# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.10.2)
project(cluster_msgs)

find_package(catkin REQUIRED COMPONENTS geographic_msgs geometry_msgs message_generation sensor_msgs std_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES

  global_gps_double.msg


  heart_beat.msg
  uav_loc_pos_enu_yaw.msg
  uav_gps_pos.msg    
)

add_service_files(
  DIRECTORY srv
  FILES
)

# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES geographic_msgs geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geographic_msgs geometry_msgs message_runtime sensor_msgs std_msgs
)

# install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")
