cmake_minimum_required(VERSION 3.0.2)
project(cover_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_cover_planner
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# polygon_drawer 库
add_library(polygon_drawer
  src/polygon_drawer.cpp
)
target_link_libraries(polygon_drawer ${catkin_LIBRARIES})

# grid_cover_planner 库
add_library(grid_cover_planner
  src/grid_cover_planner.cpp
)
target_link_libraries(grid_cover_planner ${catkin_LIBRARIES})

# 可执行文件
add_executable(grid_cover_planner_node src/grid_cover_planner_node.cpp)
target_link_libraries(grid_cover_planner_node grid_cover_planner polygon_drawer ${catkin_LIBRARIES})

install(TARGETS grid_cover_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)