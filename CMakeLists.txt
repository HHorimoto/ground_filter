cmake_minimum_required(VERSION 3.0.2)
project(ground_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Ground Filter
add_executable(ground_filter_node src/ground_filter_node.cpp)
target_link_libraries(ground_filter_node 
  ${catkin_LIBRARIES}
)
