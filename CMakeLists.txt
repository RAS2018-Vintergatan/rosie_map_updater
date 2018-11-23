cmake_minimum_required(VERSION 2.8.3)
project(rosie_map_updater)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
phidgets
nav_msgs
geometry_msgs
sensor_msgs
tf
laser_geometry
pcl_ros
visualization_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets visualization_msgs geometry_msgs sensor_msgs nav_msgs tf message_filters laser_geometry pcl_ros
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_map_updater src/rosie_map_updater.cpp)
target_link_libraries(rosie_map_updater ${catkin_LIBRARIES})
add_dependencies(rosie_map_updater ${catkin_EXPORTED_TARGETS})
