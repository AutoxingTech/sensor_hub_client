cmake_minimum_required(VERSION 3.1)
project(sensor_hub_client)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
  tf
  cln_msgs
)

add_message_files(DIRECTORY msgs FILES MockRobotState.msg MockRobotControl.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

catkin_install_python(PROGRAMS client.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
