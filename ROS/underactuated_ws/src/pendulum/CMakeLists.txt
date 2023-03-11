cmake_minimum_required(VERSION 3.0.2)
project(pendulum)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

set(root ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${root}
)

# Nodes
add_executable(controller nodes/controller/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(sim nodes/sim/simulator.cpp)
target_link_libraries(sim ${catkin_LIBRARIES})

add_executable(hal nodes/hal/hal.cpp ${root}/core/serial.cpp)
target_link_libraries(hal ${catkin_LIBRARIES})