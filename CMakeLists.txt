cmake_minimum_required(VERSION 2.8.3)
project(pcd_to_linescan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  LIBRARIES pcd_to_linescan
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pcd_to_linescan src/pcd_to_linescan_node.cpp)
target_link_libraries(pcd_to_linescan ${catkin_LIBRARIES})

add_executable(pcd_to_linescan_node src/pcd_to_linescan_node.cpp)
target_link_libraries(pcd_to_linescan_node pcd_to_linescan ${catkin_LIBRARIES})

install(TARGETS pcd_to_linescan pcd_to_linescan_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
