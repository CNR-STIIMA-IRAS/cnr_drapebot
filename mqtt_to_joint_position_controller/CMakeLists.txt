cmake_minimum_required(VERSION 3.0.2)
project(mqtt_to_joint_position_controller)


set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
	  position_controllers
	  roscpp
	  pluginlib
	  controller_interface
	  drapebot_mqtt_client
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mqtt_to_joint_position_controller
  CATKIN_DEPENDS position_controllers roscpp controller_interface drapebot_mqtt_client
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(WIN32)
  include_directories("C:\\Program Files\\mosquitto\\devel\\")
endif()

add_library(${PROJECT_NAME} src/mqtt_to_joint_position_controller.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES mqtt_to_position_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
