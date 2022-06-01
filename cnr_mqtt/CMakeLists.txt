cmake_minimum_required(VERSION 3.0.2)
project(cnr_mqtt)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  actionlib
  control_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  )


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
mqtt src/mqtt_client.cpp
)

#add_library(
#test_mqtt src/test_mqtt_client.cpp
#)

#add_executable(main src/main.cpp)
#target_link_libraries(main test_mqtt ${catkin_LIBRARIES} mosquittopp mosquitto)
#add_dependencies(main  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable(mqtt_converter src/mqtt_converter.cpp)
target_link_libraries(mqtt_converter mqtt ${catkin_LIBRARIES} mosquittopp mosquitto jsoncpp)
add_dependencies(mqtt_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
