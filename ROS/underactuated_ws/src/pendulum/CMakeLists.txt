cmake_minimum_required(VERSION 3.0.2)
project(pendulum)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})


add_library(${PROJECT_NAME} src/gazeboSim.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})