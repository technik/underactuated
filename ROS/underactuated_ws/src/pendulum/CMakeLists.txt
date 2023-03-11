cmake_minimum_required(VERSION 3.0.2)
project(pendulum)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../
)

# Nodes
add_executable(controller nodes/controller/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(sim nodes/sim/simulator.cpp)
target_link_libraries(sim ${catkin_LIBRARIES})

add_executable(sim_rk4 nodes/sim/simulator_rk4.cpp)
target_link_libraries(sim_rk4 ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS nodes/sim/pendulum_viz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)