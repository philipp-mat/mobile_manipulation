cmake_minimum_required(VERSION 3.0.2)
project(mobile_panda_simulation)

find_package(catkin REQUIRED
    roscpp
    geometry_msgs
    tf2_ros
	tf
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        geometry_msgs
        tf2_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(mobile_panda_simulation
    src/mobile_panda_simulation.cpp
)
target_link_libraries(mobile_panda_simulation
    ${catkin_LIBRARIES}
)
