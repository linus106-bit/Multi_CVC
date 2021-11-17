cmake_minimum_required(VERSION 3.0.2)
project(convex_clustering_code)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  genmsg
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  centroid.msg
  centroid_list.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  convex_clustering_code
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs genmsg message_runtime
  # DEPENDS system_lib eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include 
)

add_library(${PROJECT_NAME}  src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME} 
    ${catkin_LIBRARIES} )


add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME}_node 
		${PROJECT_NAME}
		${catkin_LIBRARIES} )

add_executable(${PROJECT_NAME}_merge src/${PROJECT_NAME}_merge.cpp)
add_dependencies(${PROJECT_NAME}_merge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME}_merge 
    ${PROJECT_NAME}
    ${catkin_LIBRARIES} )



