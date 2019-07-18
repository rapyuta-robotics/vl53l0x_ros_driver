cmake_minimum_required(VERSION 2.8.3)
project(vl53l0x_driver)

add_subdirectory(external/stm_vl530lx_lib EXCLUDE_FROM_ALL)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  message_generation
)

add_message_files(FILES
  vl53l0x.msg
)

add_service_files(FILES
  getSensorData.srv
)

generate_messages(DEPENDENCIES
   std_msgs
)

catkin_package(INCLUDE_DIRS inc
  CATKIN_DEPENDS roscpp
)

include_directories(inc/ ${catkin_INCLUDE_DIRS} )

add_executable(measure_proximity_node
  src/vl53l0x_driver.cpp
  src/mcp23017.c
)

target_link_libraries(measure_proximity_node
  vl53l0x
  soc
  ${catkin_LIBRARIES}
)
