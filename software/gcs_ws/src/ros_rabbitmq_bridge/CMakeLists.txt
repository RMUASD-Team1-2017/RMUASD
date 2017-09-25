cmake_minimum_required(VERSION 2.8.3)
project(ros_rabbitmq_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
	sensor_msgs
  message_generation
)



catkin_python_setup()

add_message_files(
  FILES
  userinfo.msg
  mission_request.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
	sensor_msgs
)



## Generate messages, services and actions. Remember to put them into msg, srv, or action folder.
## Generate messages in the 'msg' folder


## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  mission_request.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )

## Generate added messages and services with any dependencies listed here

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aed_gcs_logic
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(health_check_node src/health_check_node.cpp)
#target_link_libraries(ros_rabbitmq_bridge
#    ${catkin_LIBRARIES}
#)
#add_dependencies(health_check_node ${PROJECT_NAME}_generate_messages_cpp)

catkin_install_python(PROGRAMS scripts/rabbitmq_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
