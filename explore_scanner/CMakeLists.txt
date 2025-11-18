cmake_minimum_required(VERSION 3.0.2)
project(explore_scanner)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_ros
  cover_planner
  bspline_opt
  traj_utils
  plan_env
  ego_planner
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs pcl_ros cover_planner bspline_opt traj_utils plan_env ego_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Executable for state manager
add_executable(explore_scanner_node
  src/explore_scanner_node.cpp
  src/explore_scanner_fsm.cpp
  src/explore_scanner_manager.cpp
)

target_link_libraries(explore_scanner_node
  ${catkin_LIBRARIES}
  polygon_drawer        
  grid_cover_planner 
)

add_library(explore_scanner
  src/explore_scanner_fsm.cpp
  src/explore_scanner_manager.cpp
)

target_link_libraries(explore_scanner
  ${catkin_LIBRARIES}
)

