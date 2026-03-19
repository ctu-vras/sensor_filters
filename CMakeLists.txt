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
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        filters
        sensor_msgs
        image_transport
        point_cloud_transport
        backward_ros
)

find_package(ament_cmake REQUIRED)

find_package(Boost CONFIG)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

set(LIBRARY_TARGETS
        ${boost_TARGETS}
        ${rclcpp_TARGETS}
        ${rclcpp_components_TARGETS}
        ${rclcpp_lifecycle_TARGETS}
        ${lifecycle_msgs_TARGETS}
        ${filters_TARGETS}
        ${sensor_msgs_TARGETS}
        ${image_transport_TARGETS}
        ${point_cloud_transport_TARGETS}
        ${backward_ros_TARGETS}
)

include_directories(include include/${PROJECT_NAME})

function(add_filter_chain_node TARGET_NAME CLASS_NAME FILENAME)
    list(APPEND NODE_TARGETS ${TARGET_NAME})

    add_library("${TARGET_NAME}_component" SHARED "${FILENAME}")
    rclcpp_components_register_node("${TARGET_NAME}_component" PLUGIN "sensor_filters::${CLASS_NAME}" EXECUTABLE ${TARGET_NAME})
    #add_executable(${TARGET_NAME} "${FILENAME}")
    target_link_libraries("${TARGET_NAME}_component" PUBLIC ${LIBRARY_TARGETS})

    #install(TARGETS ${TARGET_NAME}
    #        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    #)

    install(TARGETS "${TARGET_NAME}_component"
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
    )
endfunction()

function(generate_filter_chain_source NODE_NAME)
    set(options NO_LIFECYCLE)
    set(oneValueArgs HEADER_NAME MESSAGE_TYPE BASE_FILTER)
    set(multiValueArgs SOURCES)
    cmake_parse_arguments(PARSE_ARGV 0 arg
            "${options}" "${oneValueArgs}" "${multiValueArgs}"
    )

    set(BASE_FILTER "${arg_BASE_FILTER}")
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

    add_filter_chain_node(${NODE_NAME} "${CLASS_NAME}" "${OUTPUT_FILE}")

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
                -D "NODE_NAME=${LIFECYCLE_NODE_NAME}"
                -D "OUTPUT_FILE=${LIFECYCLE_OUTPUT_FILE}"
                -D "LIFECYCLE=ON"
                -P "${CMAKE_SOURCE_DIR}/cmake/GenerateFilterChainNode.cmake"
                DEPENDS
                "${CMAKE_CURRENT_SOURCE_DIR}/cmake/LifecycleFilterChainNode.cpp.in"
                "${CMAKE_SOURCE_DIR}/cmake/GenerateFilterChainNode.cmake"
                VERBATIM
        )

        add_filter_chain_node(${LIFECYCLE_NODE_NAME} "${LIFECYCLE_CLASS_NAME}" "${LIFECYCLE_OUTPUT_FILE}")

        if (SOURCES)
            target_sources(${LIFECYCLE_NODE_NAME}_component PRIVATE ${SOURCES})
        endif ()
    endif ()
endfunction()

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
add_filter_chain_node("image_filter_chain" "ImageFilterChainNode" src/ImageFilterChainNode.cpp)
add_filter_chain_node("pointcloud2_filter_chain" "PointCloud2FilterChainNode" src/PointCloud2FilterChainNode.cpp)

set(filters
        ChangeHeaderFilter
)

foreach (filter IN LISTS filters)
    list(APPEND FILTER_LIBRARIES ${filter})

    add_library(${filter} src/filters/${filter}.cpp)
    target_link_libraries(${filter} PUBLIC ${LIBRARY_TARGETS})
    pluginlib_export_plugin_description_file(filters ${filter}.xml)
endforeach ()

install(TARGETS ${FILTER_LIBRARIES}
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

ament_export_include_directories(include include/${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_export_libraries(${FILTER_LIBRARIES})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
