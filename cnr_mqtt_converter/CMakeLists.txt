cmake_minimum_required(VERSION 3.0.2)
project(cnr_mqtt_converter)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  actionlib
  control_msgs
  configuration_msgs
  cnr_mqtt_client
  rosdyn_core
  tf_conversions
  )

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto REQUIRED)
#pkg_check_modules(Mosquittopp IMPORTED_TARGET libmosquittopp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cnr_mqtt_client rosdyn_core
  )


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
mqtt src/mqtt_converter_client.cpp 
)

#add_library(
#test_mqtt src/test_mqtt_client.cpp
#)

#add_executable(main src/main.cpp)
#target_link_libraries(main test_mqtt ${catkin_LIBRARIES} mosquittopp mosquitto)
#add_dependencies(main  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable(mqtt_converter src/mqtt_converter_node.cpp)
target_link_libraries(mqtt_converter mqtt ${catkin_LIBRARIES} mosquitto jsoncpp)

#target_link_libraries (${PROJECT_NAME} ${catkin_LIBRARIES} PkgConfig::Mosquittopp PkgConfig::Mosquitto jsoncpp)

add_dependencies(mqtt_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
