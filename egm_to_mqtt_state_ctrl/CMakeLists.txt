cmake_minimum_required(VERSION 3.0.2)
project(egm_to_mqtt_state_ctrl)

find_package(catkin REQUIRED COMPONENTS

    abb_egm_hardware_interface
    abb_egm_msgs
    abb_robot_cpp_utilities
    controller_interface
    pluginlib
    realtime_tools
    roscpp
	  drapebot_mqtt_client
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES egm_to_mqtt_state_ctrl
  CATKIN_DEPENDS roscpp controller_interface drapebot_mqtt_client
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(WIN32)
  include_directories("C:\\Program Files\\mosquitto\\devel\\")
endif()


add_library(${PROJECT_NAME} src/egm_to_mqtt_state_ctrl.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
