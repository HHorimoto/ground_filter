cmake_minimum_required(VERSION 3.0.2)
project(ground_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  rostest
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Ground Filter
add_executable(${PROJECT_NAME} src/ground_filter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
