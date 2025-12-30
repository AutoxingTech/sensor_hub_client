cmake_minimum_required(VERSION 3.1)
project(sensor_hub_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  message_generation
  tf
  ax_msgs
)

add_message_files(DIRECTORY msgs FILES TcpRobotState.msg TcpRobotControl.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs ax_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})


file(GLOB SRC_FILES
    src/main.cpp
    src/crc.cpp
    src/tcp_pack.cpp
    src/tcp_stream.cpp
    src/sensor_hub_client.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
