cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package(
)
catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
