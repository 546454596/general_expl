cmake_minimum_required(VERSION 3.0.2)
project(rviz_polygon_plugin)

# Ensure we use C++14 or later for compatibility
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rviz
  geometry_msgs
  visualization_msgs
  pluginlib
  roscpp
)

# Qt5 setup
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)

# This setting causes Qt's "signals" and "slots" keywords to be defined.
add_definitions(-DQT_NO_KEYWORDS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rviz geometry_msgs visualization_msgs pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Make sure we can find the generated header files
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Source files
set(SOURCES
  src/polygon_draw_plugin.cpp
)

# Header files that need MOC processing
set(HEADERS
  include/polygon_draw_plugin.h
)

# Run Qt5 MOC on headers
qt5_wrap_cpp(MOC_FILES ${HEADERS})

# Create the plugin library
add_library(${PROJECT_NAME}
  ${SOURCES}
  ${MOC_FILES}
)

# Add compile definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE "QT_NO_KEYWORDS")

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

# Install rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)