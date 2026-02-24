# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(sensor_filters)

# TODO 2026-02-23 (solonovamax): clean all this up

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(DEPENDENCIES robot_body_filter_msgs
        backward_ros
        filters
        image_transport
        point_cloud_transport
        rclcpp
        sensor_msgs
)

find_package(ament_cmake REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

include_directories(include include/${PROJECT_NAME})

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

foreach (type IN LISTS types)
    string(TOLOWER "${type}" type_lower)

    set(node_name "${type_lower}_filter_chain")

    list(APPEND NODE_TARGETS ${node_name})

    #    add_library(${node_name} src/${type}FilterChainNodelet.cpp)
    #    #target_link_libraries(${nodelet_name} ${DEPENDENCIES})
    #    ament_target_dependencies(${node_name} ${DEPENDENCIES})

    add_executable(${node_name} src/${type}FilterChainNode.cpp)
    #target_link_libraries(${node_name} ${DEPENDENCIES})
    ament_target_dependencies(${node_name} ${DEPENDENCIES})

    #    install(TARGETS ${node_name}
    #            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    #    )

    install(TARGETS ${node_name}
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION lib
    )
endforeach ()

# Additional sources for the targets that use custom transport
#target_sources(image_filter_chain_nodelet PRIVATE src/ImageFilterChainBase.cpp)
target_sources(image_filter_chain PRIVATE src/ImageFilterChainBase.cpp)
#target_sources(pointcloud2_filter_chain_nodelet PRIVATE src/PointCloud2FilterChainBase.cpp)
target_sources(pointcloud2_filter_chain PRIVATE src/PointCloud2FilterChainBase.cpp)

# TODO 2026-02-23 (solonovamax): finish this
#
#set(filters
#        ChangeHeaderFilter
#)
#
#foreach (filter IN LISTS filters)
#    add_library(${filter} src/filters/${filter}.cpp)
#    add_dependencies(${filter} ${catkin_EXPORTED_TARGETS})
#    target_link_libraries(${filter} ${catkin_LIBRARIES})
#    install(TARGETS ${filter}
#            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#    )
#    install(FILES
#            ${filter}.xml
#            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )
#endforeach ()

install(
        DIRECTORY include/
        DESTINATION include
)

#install(TARGETS ${NODE_TARGETS}
#        EXPORT export_${PROJECT_NAME}
#        RUNTIME DESTINATION bin
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#)

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

#ament_export_include_directories(include include/${PROJECT_NAME})
#ament_export_dependencies(${DEPENDENCIES})
#ament_export_libraries(${PROJECT_NAME} )
#ament_export_targets(
#  export_${PROJECT_NAME}
#)
ament_package()
