cmake_minimum_required(VERSION 3.10)
project(rqt_mav_manager)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)
catkin_python_setup()

catkin_package()

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY resource DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/rqt_mav_manager DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
