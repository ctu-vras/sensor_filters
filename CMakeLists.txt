# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.16.3)
project(sensor_filters)

set(CMAKE_CXX_STANDARD 17)

################
# Dependencies #
################

set(DEPENDENCIES
  Boost
  filters
  image_transport
  pluginlib
  point_cloud_transport
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rosidl_runtime_cpp
  sensor_msgs
)
set(BUILD_ONLY_DEPENDENCIES
  backward_ros
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

foreach(dependency IN ITEMS ${BUILD_ONLY_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

set(LIBRARY_TARGETS
  Boost::boost
  filters::filter_chain
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  rosidl_runtime_cpp::rosidl_runtime_cpp
  ${sensor_msgs_TARGETS}
)

##################
# Base libraries #
##################

# filter_chain_base
add_library(filter_chain_base SHARED src/FilterChainBase.cpp)
target_link_libraries(filter_chain_base PUBLIC ${LIBRARY_TARGETS})
target_include_directories(filter_chain_base PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(rclcpp_VERSION VERSION_LESS 20.0.0)
  target_compile_definitions(filter_chain_base PUBLIC MATCHED_EVENT_NOT_AVAILABLE)
endif()

# sensor_filters_node_helper
# cannot be SHARED, we don't want to export this library; OBJECT is similar to SHARED but not exported and has no .so
add_library(${PROJECT_NAME}_node_helper OBJECT src/NodeHelper.cpp)
target_link_libraries(${PROJECT_NAME}_node_helper PUBLIC rclcpp::rclcpp)
target_include_directories(${PROJECT_NAME}_node_helper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(${PROJECT_NAME}_node_helper PROPERTIES POSITION_INDEPENDENT_CODE ON)

# sensor_filters_image_filter_chain_base
add_library(${PROJECT_NAME}_image_filter_chain_base SHARED src/ImageFilterChainBase.cpp)
target_link_libraries(${PROJECT_NAME}_image_filter_chain_base
  PUBLIC ${LIBRARY_TARGETS} image_transport::image_transport
  PRIVATE ${PROJECT_NAME}_node_helper
)
target_include_directories(${PROJECT_NAME}_image_filter_chain_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(image_transport_VERSION VERSION_LESS 6.4.0)
  target_compile_definitions(${PROJECT_NAME}_image_filter_chain_base
    PUBLIC IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE)
endif()
if(image_transport_VERSION VERSION_LESS 3.2.0)
  target_compile_definitions(${PROJECT_NAME}_image_filter_chain_base PUBLIC IMAGE_TRANSPORT_PUB_OPTIONS_NOT_AVAILABLE)
endif()

# sensor_filters_pointcloud2_filter_chain_base
add_library(${PROJECT_NAME}_pointcloud2_filter_chain_base SHARED src/PointCloud2FilterChainBase.cpp)
target_link_libraries(${PROJECT_NAME}_pointcloud2_filter_chain_base
  PUBLIC ${LIBRARY_TARGETS} point_cloud_transport::point_cloud_transport
  PRIVATE ${PROJECT_NAME}_node_helper
)
target_include_directories(${PROJECT_NAME}_pointcloud2_filter_chain_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(point_cloud_transport_VERSION VERSION_LESS 5.2.0)
  target_compile_definitions(${PROJECT_NAME}_pointcloud2_filter_chain_base
    PUBLIC POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE)
endif()

##################################################################
# Macros for generating and adding individual filter chain nodes #
##################################################################

#
# Add a single generated filter chain node.
#
# :param TARGET_NAME: Name of the node's target.
# :type TARGET_NAME: string
# :param CLASS_NAME: Name of the C++ class with the node.
# :type CLASS_NAME: string
# :param FILENAME: Path to the generated node C++ file.
# :type FILENAME: string
# :param BASE_FILTER_LIB: Extra library to link (optional, set when you use a non-default filter chain base).
# :type BASE_FILTER_LIB: string
#
macro(add_filter_chain_node arg_TARGET_NAME arg_CLASS_NAME arg_FILENAME)
  set(options)
  set(oneValueArgs BASE_FILTER_LIB)
  set(multiValueArgs)
  cmake_parse_arguments(arg "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  set(TARGET_NAME "${arg_TARGET_NAME}")
  set(CLASS_NAME "${arg_CLASS_NAME}")
  set(FILENAME "${arg_FILENAME}")

  set(BASE_FILTER_LIB "${arg_BASE_FILTER_LIB}")

  add_library("${TARGET_NAME}_component" SHARED "${FILENAME}")
  target_link_libraries("${TARGET_NAME}_component" PRIVATE ${LIBRARY_TARGETS} filter_chain_base)
  if(BASE_FILTER_LIB)
    target_link_libraries("${TARGET_NAME}_component" PRIVATE ${BASE_FILTER_LIB})
  endif()
  target_include_directories(${TARGET_NAME}_component PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  rclcpp_components_register_node(
    "${TARGET_NAME}_component" PLUGIN "sensor_filters::${CLASS_NAME}" EXECUTABLE ${TARGET_NAME})

  install(TARGETS "${TARGET_NAME}_component"
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endmacro()

#
# Generate source code for a filter chain node and add it.
#
# :param NODE_NAME: Name of the node executable and its parameter namespace.
# :type NODE_NAME: string
# :param HEADER_NAME: Name of the header file for the message type.
# :type HEADER_NAME: string
# :param MESSAGE_TYPE: Name of the message type.
# :type MESSAGE_TYPE: string
# :param BASE_FILTER: Base filter class to use (optional, defaults to sensor_filters::FilterChainNodeBase).
# :type BASE_FILTER: string
# :param BASE_FILTER_LIB: Extra library to link (optional, set if you change BASE_FILTER).
# :type BASE_FILTER_LIB: string
# :param NO_LIFECYCLE: Whether to skip generating the lifecycle node (optional).
# :type NO_LIFECYCLE: boolean
#
macro(generate_filter_chain_source arg_NODE_NAME)
  set(options NO_LIFECYCLE)
  set(oneValueArgs HEADER_NAME MESSAGE_TYPE BASE_FILTER BASE_FILTER_LIB)
  set(multiValueArgs SOURCES)
  cmake_parse_arguments(arg "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  set(NODE_NAME "${arg_NODE_NAME}")

  set(BASE_FILTER "${arg_BASE_FILTER}")
  set(BASE_FILTER_LIB "${arg_BASE_FILTER_LIB}")
  set(HEADER_NAME "${arg_HEADER_NAME}")
  set(MESSAGE_TYPE "${arg_MESSAGE_TYPE}")
  set(NO_LIFECYCLE "${arg_NO_LIFECYCLE}")

  set(NODE_NAME "${NODE_NAME}_filter_chain")
  set(CLASS_NAME "${MESSAGE_TYPE}FilterChainNode")
  set(FILE_NAME "${MESSAGE_TYPE}FilterChain.cpp")
  set(OUTPUT_FILE "${PROJECT_BINARY_DIR}/${FILE_NAME}")

  set(MESSAGE_HEADER "sensor_msgs/msg/${HEADER_NAME}.hpp")

  if(BASE_FILTER)
    set(TEMPLATE_ARGS "sensor_msgs::msg::${MESSAGE_TYPE}, sensor_filters::${BASE_FILTER}")
    set(BASE_INCLUDE "#include <sensor_filters/${BASE_FILTER}.hpp>")
  else()
    set(TEMPLATE_ARGS "sensor_msgs::msg::${MESSAGE_TYPE}")
    set(BASE_INCLUDE "")
  endif()

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FilterChainNode.cpp.in ${OUTPUT_FILE} @ONLY)
  add_filter_chain_node(
    ${NODE_NAME}
    "${CLASS_NAME}"
    "${OUTPUT_FILE}"
    BASE_FILTER_LIB "${BASE_FILTER_LIB}"
  )

  if(NOT NO_LIFECYCLE)
    set(LIFECYCLE_NODE_NAME "${NODE_NAME}_lifecycle")
    set(LIFECYCLE_CLASS_NAME "Lifecycle${CLASS_NAME}")
    set(LIFECYCLE_FILE_NAME "Lifecycle${FILE_NAME}")
    set(LIFECYCLE_OUTPUT_FILE "${PROJECT_BINARY_DIR}/${LIFECYCLE_FILE_NAME}")

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/LifecycleFilterChainNode.cpp.in ${LIFECYCLE_OUTPUT_FILE} @ONLY)
    add_filter_chain_node(
      ${LIFECYCLE_NODE_NAME}
      "${LIFECYCLE_CLASS_NAME}"
      "${LIFECYCLE_OUTPUT_FILE}"
      BASE_FILTER_LIB "${BASE_FILTER_LIB}"
    )
  endif()
endmacro()

# Generate filter chains for all supported sensor_msgs types

generate_filter_chain_source("battery_state" HEADER_NAME "battery_state" MESSAGE_TYPE "BatteryState")
generate_filter_chain_source("camera_info" HEADER_NAME "camera_info" MESSAGE_TYPE "CameraInfo")
generate_filter_chain_source("compressed_image" HEADER_NAME "compressed_image" MESSAGE_TYPE "CompressedImage")
generate_filter_chain_source("fluid_pressure" HEADER_NAME "fluid_pressure" MESSAGE_TYPE "FluidPressure")
generate_filter_chain_source("illuminance" HEADER_NAME "illuminance" MESSAGE_TYPE "Illuminance")
generate_filter_chain_source("imu" HEADER_NAME "imu" MESSAGE_TYPE "Imu")
generate_filter_chain_source("joint_state" HEADER_NAME "joint_state" MESSAGE_TYPE "JointState")
generate_filter_chain_source("joy" HEADER_NAME "joy" MESSAGE_TYPE "Joy")
generate_filter_chain_source("joy_feedback_array" HEADER_NAME "joy_feedback_array" MESSAGE_TYPE "JoyFeedbackArray")
generate_filter_chain_source("laser_scan" HEADER_NAME "laser_scan" MESSAGE_TYPE "LaserScan")
generate_filter_chain_source("magnetic_field" HEADER_NAME "magnetic_field" MESSAGE_TYPE "MagneticField")
generate_filter_chain_source(
  "multi_dof_joint_state" HEADER_NAME "multi_dof_joint_state" MESSAGE_TYPE "MultiDOFJointState")
generate_filter_chain_source("multi_echo_scan" HEADER_NAME "multi_echo_laser_scan" MESSAGE_TYPE "MultiEchoLaserScan")
generate_filter_chain_source("navsat_fix" HEADER_NAME "nav_sat_fix" MESSAGE_TYPE "NavSatFix")
generate_filter_chain_source("pointcloud" HEADER_NAME "point_cloud" MESSAGE_TYPE "PointCloud")
generate_filter_chain_source("range" HEADER_NAME "range" MESSAGE_TYPE "Range")
generate_filter_chain_source("relative_humidity" HEADER_NAME "relative_humidity" MESSAGE_TYPE "RelativeHumidity")
generate_filter_chain_source("temperature" HEADER_NAME "temperature" MESSAGE_TYPE "Temperature")
generate_filter_chain_source("time_reference" HEADER_NAME "time_reference" MESSAGE_TYPE "TimeReference")
# special cases with transport-based pub/sub
generate_filter_chain_source("image" HEADER_NAME "image" MESSAGE_TYPE "Image"
  BASE_FILTER ImageFilterChainBase BASE_FILTER_LIB ${PROJECT_NAME}_image_filter_chain_base)
generate_filter_chain_source("pointcloud2" HEADER_NAME "point_cloud2" MESSAGE_TYPE "PointCloud2"
  BASE_FILTER PointCloud2FilterChainBase BASE_FILTER_LIB ${PROJECT_NAME}_pointcloud2_filter_chain_base)

install(
  TARGETS
    filter_chain_base
    ${PROJECT_NAME}_image_filter_chain_base
    ${PROJECT_NAME}_pointcloud2_filter_chain_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

##############################
# Register universal filters #
##############################

include(src/filters/CMakeLists.txt)

###########
# Install #
###########

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY cmake/ DESTINATION share/${PROJECT_NAME}/cmake FILES_MATCHING PATTERN "UniversalFilter*")

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp(SKIP_UNCRUSTIFY)

#    find_package(roslint REQUIRED)
#
#    # catkin_lint - checks validity of package.xml and CMakeLists.txt
#    # ROS buildfarm calls this without any environment and with empty rosdep cache,
#    # so we have problems reading the list of packages from env
#    # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
#    if (DEFINED ENV{ROS_HOME})
#        #catkin_lint: ignore_once env_var
#        set(ROS_HOME "$ENV{ROS_HOME}")
#    else ()
#        #catkin_lint: ignore_once env_var
#        set(ROS_HOME "$ENV{HOME}/.ros")
#    endif ()
#    #catkin_lint: ignore_once env_var
#    if (DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
#        roslint_custom(catkin_lint "-W2" .)
#    endif ()
#
#    # Roslint C++ - checks formatting and some other rules for C++ files
#
#    file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
#    file(GLOB_RECURSE ROSLINT_SRC src/*.cpp src/filters/*.cpp src/filters/*.hpp examples/*.cpp)
#    #file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)
#
#    set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
#    -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
#    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright,-build/namespaces")
#    roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC})
#
#    roslint_add_test()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
