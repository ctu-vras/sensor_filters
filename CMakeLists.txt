cmake_minimum_required(VERSION 3.0.2)
project(sensor_filters)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  filters
  nodelet
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS filters nodelet roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(types
  CompressedImage
  Image
  Imu
  Joy
  LaserScan
  MultiEchoLaserScan
  MagneticField
  NavSatFix
  PointCloud
  PointCloud2
  Range
  RelativeHumidity
  Temperature
)

foreach(type IN LISTS types)
  string(TOLOWER "${type}" type_lower)

  set(nodelet_name "${type_lower}_filter_chain_nodelet")
  set(node_name "${type_lower}_filter_chain")

  add_library(${nodelet_name} src/${type}FilterChainNodelet.cc)
  add_dependencies(${nodelet_name} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${nodelet_name} ${catkin_LIBRARIES})

  add_executable(${node_name} src/${type}FilterChainNode.cc)
  add_dependencies(${node_name} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node_name} ${catkin_LIBRARIES})

  install(TARGETS ${node_name}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(TARGETS ${nodelet_name}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
endforeach()

set(filters
  ChangeHeaderFilter
)

foreach(filter IN LISTS filters)
  add_library(${filter} src/filters/${filter}.cc)
  add_dependencies(${filter} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${filter} ${catkin_LIBRARIES})
  install(TARGETS ${filter}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
  install(FILES
    ${filter}.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)