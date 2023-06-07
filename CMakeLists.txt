# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(sensor_filters)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  filters
  image_transport
  nodelet
  point_cloud_transport
  roscpp
  sensor_msgs
)

#catkin_lint: ignore missing_export_lib
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

# Additional sources for the targets that use custom transport
target_sources(image_filter_chain_nodelet PRIVATE src/ImageFilterChainBase.cc)
target_sources(image_filter_chain PRIVATE src/ImageFilterChainBase.cc)
target_sources(pointcloud2_filter_chain_nodelet PRIVATE src/PointCloud2FilterChainBase.cc)
target_sources(pointcloud2_filter_chain PRIVATE src/PointCloud2FilterChainBase.cc)

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

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment and with empty rosdep cache,
  # so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_HOME})
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{ROS_HOME}")
  else()
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{HOME}/.ros")
  endif()
  #catkin_lint: ignore_once env_var
  if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
    roslint_custom(catkin_lint "-W2" .)
  endif()

  # Roslint C++ - checks formatting and some other rules for C++ files

  file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
  file(GLOB_RECURSE ROSLINT_SRC src/*.cc src/filters/*.cc src/filters/*.hh examples/*.cpp)
  #file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright,-build/namespaces")
  roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC})
  
  roslint_add_test()
endif()
