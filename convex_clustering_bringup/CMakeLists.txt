cmake_minimum_required(VERSION 3.0.2)
project(convex_clustering_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)


catkin_package(CATKIN_DEPENDS
  rospy
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/convex_bringup.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
