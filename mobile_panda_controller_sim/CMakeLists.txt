cmake_minimum_required(VERSION 3.0.2)
project(mobile_panda_controller_sim)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
controller_interface
dynamic_reconfigure
franka_hw
hardware_interface
pluginlib
#realtime_tool
geometry_msgs
realtime_tools
tf
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

catkin_python_setup()


catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES mobile_panda_controller_sim
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  DEPENDS Franka
  
  #  DEPENDS system_lib
  )
  
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(utils
   src/utils.cpp
)

add_library(mobile_panda_controller_sim
  src/twist_controller_sim.cpp
  src/path_controller_sim.cpp
)

add_dependencies(mobile_panda_controller_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

catkin_install_python(PROGRAMS scripts/cmd_vel_mux.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(mobile_panda_controller_sim PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  utils
)

target_include_directories(mobile_panda_controller_sim SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(mobile_panda_controller_sim PUBLIC
  include
)

## Installation
install(TARGETS mobile_panda_controller_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES mobile_panda_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#catkin_install_python(
  #PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py
  #DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(mobile_panda_controller_sim FILES ${SOURCES} ${HEADERS})
  add_tidy_target(mobile_panda_controller_sim
    FILES ${SOURCES}
    DEPENDS mobile_panda_controller_sim
  )
endif()

