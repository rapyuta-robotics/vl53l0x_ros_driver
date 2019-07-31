cmake_minimum_required(VERSION 2.8.3)
project(vl53l0x_driver)

add_subdirectory(external/stm_vl530lx_lib EXCLUDE_FROM_ALL)

add_compile_options(-std=c++11)

set(PKG_DEPS
  roscpp
  std_msgs
  message_generation
  rr_hw_interface
)


find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS}
)

add_message_files(FILES
  vl53l0x.msg
)

generate_messages(DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include external/stm_vl530lx_lib/inc
  CATKIN_DEPENDS ${PKG_DEPS}
)

include_directories(
  include
  external/stm_vl530lx_lib/inc
  ${catkin_INCLUDE_DIRS} )

add_executable(measure_proximity_node
  src/vl53l0x_driver.cpp
)

target_link_libraries(measure_proximity_node
  vl53l0x
  soc
  ${catkin_LIBRARIES}
)
