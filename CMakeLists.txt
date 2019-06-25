cmake_minimum_required(VERSION 2.8.3)
project(ranged_finder)

#target_link_libraries(vl53l0x ${catkin_LIBRARIES})

add_subdirectory(external/stm_vl530lx_lib EXCLUDE_FROM_ALL)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  RangedFinder.msg
)

add_service_files(
  FILES
  getSensorData.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
 INCLUDE_DIRS inc
 CATKIN_DEPENDS roscpp
)

include_directories(inc/ ${catkin_INCLUDE_DIRS} )

add_executable(
measure3sensors_node 
src/measure3sensors.cpp 
src/jetsonGPIO.c 
)

add_executable(
measure1sensor_node 
src/measure1sensor.cpp 
src/jetsonGPIO.c 
)

target_link_libraries(
  measure3sensors_node
  vl53l0x
  ${catkin_LIBRARIES}
)

target_link_libraries(
  measure1sensor_node 
  vl53l0x
  ${catkin_LIBRARIES}
)
