# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(sensor_filters)

# TODO 2026-02-23 (solonovamax): clean all this up

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(DEPENDENCIES
        filters
        image_transport
        point_cloud_transport
        pluginlib
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

find_package(Boost CONFIG REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

foreach (dependency IN ITEMS ${BUILD_ONLY_DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

set(LIBRARY_TARGETS
        Boost::boost
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_components::component
        rclcpp_lifecycle::rclcpp_lifecycle
        rosidl_runtime_cpp::rosidl_runtime_cpp
        filters::filter_chain
        ${sensor_msgs_TARGETS}
)

function(add_filter_chain_node TARGET_NAME CLASS_NAME FILENAME)
    set(options)
    set(oneValueArgs BASE_FILTER_LIB)
    set(multiValueArgs)
    cmake_parse_arguments(PARSE_ARGV 0 arg
      "${options}" "${oneValueArgs}" "${multiValueArgs}"
    )

    set(BASE_FILTER_LIB "${arg_BASE_FILTER_LIB}")

    list(APPEND NODE_TARGETS ${TARGET_NAME})

    add_library("${TARGET_NAME}_component" SHARED "${FILENAME}")
    rclcpp_components_register_node("${TARGET_NAME}_component" PLUGIN "sensor_filters::${CLASS_NAME}" EXECUTABLE ${TARGET_NAME})
    target_link_libraries("${TARGET_NAME}_component" PRIVATE ${LIBRARY_TARGETS})
    if(BASE_FILTER_LIB)
        target_link_libraries("${TARGET_NAME}_component" PRIVATE ${BASE_FILTER_LIB})
    endif()
    target_include_directories(${TARGET_NAME}_component PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )

    install(TARGETS "${TARGET_NAME}_component"
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
    )
endfunction()

function(generate_filter_chain_source NODE_NAME)
    set(options NO_LIFECYCLE)
    set(oneValueArgs HEADER_NAME MESSAGE_TYPE BASE_FILTER BASE_FILTER_LIB)
    set(multiValueArgs SOURCES)
    cmake_parse_arguments(PARSE_ARGV 0 arg
            "${options}" "${oneValueArgs}" "${multiValueArgs}"
    )

    set(BASE_FILTER "${arg_BASE_FILTER}")
    set(BASE_FILTER_LIB "${arg_BASE_FILTER_LIB}")
    set(HEADER_NAME "${arg_HEADER_NAME}")
    set(MESSAGE_TYPE "${arg_MESSAGE_TYPE}")
    set(SOURCES "${arg_SOURCES}")
    set(NO_LIFECYCLE "${arg_NO_LIFECYCLE}")

    set(NODE_NAME "${NODE_NAME}_filter_chain")

    set(CLASS_NAME "${MESSAGE_TYPE}FilterChainNode")
    set(FILE_NAME "${MESSAGE_TYPE}FilterChain.cpp")

    set(OUTPUT_FILE "${PROJECT_BINARY_DIR}/${FILE_NAME}")

    add_custom_command(
            OUTPUT "${OUTPUT_FILE}"
            COMMAND ${CMAKE_COMMAND}
            -D "PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
            -D "MESSAGE_TYPE=${MESSAGE_TYPE}"
            -D "HEADER_NAME=${HEADER_NAME}"
            -D "BASE_FILTER=${BASE_FILTER}"
            -D "CLASS_NAME=${CLASS_NAME}"
            -D "NODE_NAME=${NODE_NAME}"
            -D "OUTPUT_FILE=${OUTPUT_FILE}"
            -D "LIFECYCLE=OFF"
            -P "${CMAKE_SOURCE_DIR}/cmake/GenerateFilterChainNode.cmake"
            DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/FilterChainNode.cpp.in"
            "${CMAKE_SOURCE_DIR}/cmake/GenerateFilterChainNode.cmake"
            VERBATIM
    )

    add_filter_chain_node(${NODE_NAME} "${CLASS_NAME}" "${OUTPUT_FILE}" BASE_FILTER_LIB "${BASE_FILTER_LIB}")

    if (SOURCES)
        target_sources(${NODE_NAME}_component PRIVATE ${SOURCES})
    endif ()

    if (NOT NO_LIFECYCLE)
        set(LIFECYCLE_NODE_NAME "${NODE_NAME}_lifecycle")

        set(LIFECYCLE_CLASS_NAME "Lifecycle${CLASS_NAME}")
        set(LIFECYCLE_FILE_NAME "Lifecycle${FILE_NAME}")

        set(LIFECYCLE_OUTPUT_FILE "${PROJECT_BINARY_DIR}/${LIFECYCLE_FILE_NAME}")

        add_custom_command(
                OUTPUT "${LIFECYCLE_OUTPUT_FILE}"
                COMMAND ${CMAKE_COMMAND}
                -D "PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
                -D "MESSAGE_TYPE=${MESSAGE_TYPE}"
                -D "HEADER_NAME=${HEADER_NAME}"
                -D "BASE_FILTER=${BASE_FILTER}"
                -D "CLASS_NAME=${LIFECYCLE_CLASS_NAME}"
                -D "NODE_NAME=${NODE_NAME}"  # lifecycle and normal nodes use the same name for interchangeability
                -D "OUTPUT_FILE=${LIFECYCLE_OUTPUT_FILE}"
                -D "LIFECYCLE=ON"
                -P "${CMAKE_SOURCE_DIR}/cmake/GenerateFilterChainNode.cmake"
                DEPENDS
                "${CMAKE_CURRENT_SOURCE_DIR}/cmake/LifecycleFilterChainNode.cpp.in"
                "${CMAKE_SOURCE_DIR}/cmake/GenerateFilterChainNode.cmake"
                VERBATIM
        )

        add_filter_chain_node(${LIFECYCLE_NODE_NAME} "${LIFECYCLE_CLASS_NAME}" "${LIFECYCLE_OUTPUT_FILE}" BASE_FILTER_LIB "${BASE_FILTER_LIB}")

        if (SOURCES)
            target_sources(${LIFECYCLE_NODE_NAME}_component PRIVATE ${SOURCES})
        endif ()
    endif ()
endfunction()

# cannot be SHARED, we don't want to export this library; OBJECT is similar to SHARED but not exported and has no .so
add_library(${PROJECT_NAME}_node_helper OBJECT src/NodeHelper.cpp)
target_link_libraries(${PROJECT_NAME}_node_helper PUBLIC rclcpp::rclcpp)
target_include_directories(${PROJECT_NAME}_node_helper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(${PROJECT_NAME}_node_helper PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}_image_filter_chain_base SHARED src/ImageFilterChainBase.cpp)
target_link_libraries(${PROJECT_NAME}_image_filter_chain_base PUBLIC ${LIBRARY_TARGETS} image_transport::image_transport)
target_link_libraries(${PROJECT_NAME}_image_filter_chain_base PRIVATE ${PROJECT_NAME}_node_helper)
target_include_directories(${PROJECT_NAME}_image_filter_chain_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(image_transport_VERSION VERSION_LESS 6.4.0)
    target_compile_definitions(${PROJECT_NAME}_image_filter_chain_base PUBLIC IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE)
endif()

add_library(${PROJECT_NAME}_pointcloud2_filter_chain_base SHARED src/PointCloud2FilterChainBase.cpp)
target_link_libraries(${PROJECT_NAME}_pointcloud2_filter_chain_base PUBLIC ${LIBRARY_TARGETS} point_cloud_transport::point_cloud_transport)
target_link_libraries(${PROJECT_NAME}_pointcloud2_filter_chain_base PRIVATE ${PROJECT_NAME}_node_helper)
target_include_directories(${PROJECT_NAME}_pointcloud2_filter_chain_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(point_cloud_transport_VERSION VERSION_LESS 5.2.0)
    target_compile_definitions(${PROJECT_NAME}_pointcloud2_filter_chain_base PUBLIC POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE)
endif()

generate_filter_chain_source("compressed_image" HEADER_NAME "compressed_image" MESSAGE_TYPE "CompressedImage")
generate_filter_chain_source("imu" HEADER_NAME "imu" MESSAGE_TYPE "Imu")
generate_filter_chain_source("joy" HEADER_NAME "joy" MESSAGE_TYPE "Joy")
generate_filter_chain_source("scan" HEADER_NAME "laser_scan" MESSAGE_TYPE "LaserScan")
generate_filter_chain_source("magnetic_field" HEADER_NAME "magnetic_field" MESSAGE_TYPE "MagneticField")
generate_filter_chain_source("multi_echo_scan" HEADER_NAME "multi_echo_laser_scan" MESSAGE_TYPE "MultiEchoLaserScan")
generate_filter_chain_source("navsat_fix" HEADER_NAME "nav_sat_fix" MESSAGE_TYPE "NavSatFix")
generate_filter_chain_source("pointcloud" HEADER_NAME "point_cloud" MESSAGE_TYPE "PointCloud")
generate_filter_chain_source("range" HEADER_NAME "range" MESSAGE_TYPE "Range")
generate_filter_chain_source("relative_humidity" HEADER_NAME "relative_humidity" MESSAGE_TYPE "RelativeHumidity")
generate_filter_chain_source("temperature" HEADER_NAME "temperature" MESSAGE_TYPE "Temperature")
# special cases with transport-based pub/sub
generate_filter_chain_source("image" HEADER_NAME "image" MESSAGE_TYPE "Image"
    BASE_FILTER ImageFilterChainBase BASE_FILTER_LIB ${PROJECT_NAME}_image_filter_chain_base)
generate_filter_chain_source("pointcloud2" HEADER_NAME "point_cloud2" MESSAGE_TYPE "PointCloud2"
    BASE_FILTER PointCloud2FilterChainBase BASE_FILTER_LIB ${PROJECT_NAME}_pointcloud2_filter_chain_base)

set(filters
        ChangeHeaderFilter
)

foreach (filter IN LISTS filters)
    list(APPEND FILTER_LIBRARIES ${filter})

    add_library(${filter} SHARED src/filters/${filter}.cpp)
    target_link_libraries(${filter} PRIVATE ${LIBRARY_TARGETS})
    target_include_directories(${filter} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    pluginlib_export_plugin_description_file(filters ${filter}.xml)
endforeach ()

install(TARGETS ${FILTER_LIBRARIES} ${PROJECT_NAME}_image_filter_chain_base ${PROJECT_NAME}_pointcloud2_filter_chain_base
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY include/
        DESTINATION include
)

#if (CATKIN_ENABLE_TESTING)
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
#endif ()

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
