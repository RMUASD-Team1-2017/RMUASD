cmake_minimum_required(VERSION 2.8.3)
project(aed_gcs_logic)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

find_package (GeographicLib 1.34 REQUIRED)

catkin_python_setup()
## Generate messages, services and actions. Remember to put them into msg, srv, or action folder.
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  geo_coord.msg
  waypoints.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  mission_request.srv
  AbortRequest.srv
  OnboardStatus.srv
  HealthCheckService.srv
  SendMission.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aed_gcs_logic
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories (${GeographicLib_INCLUDE_DIRS})
add_definitions (${GeographicLib_DEFINITIONS})

add_executable(health_check_node src/health_check_node.cpp)
target_link_libraries(health_check_node
    ${catkin_LIBRARIES}
)
add_dependencies(health_check_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(path_planner_node src/path_planner_node.cpp src/path_planner.cpp)
target_link_libraries(path_planner_node
    ${catkin_LIBRARIES}
    ${GeographicLib_LIBRARIES}
)
add_dependencies(path_planner_node ${PROJECT_NAME}_generate_messages_cpp)

catkin_install_python(PROGRAMS src/risk_assessment.py src/path_planner_dummy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(control_logic_node src/control_logic_node.cpp src/drone_logic.cpp)
target_link_libraries(control_logic_node ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
add_dependencies(control_logic_node ${PROJECT_NAME}_generate_messages_cpp)


add_executable(control_logic_node_test src/controllogic.cpp)
target_link_libraries(control_logic_node_test ${catkin_LIBRARIES})
add_dependencies(control_logic_node_test ${PROJECT_NAME}_generate_messages_cpp)






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
