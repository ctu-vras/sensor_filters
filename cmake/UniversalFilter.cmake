# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Keep these lists in sync with include/sensor_filters/RegisterFilter.hpp

set(UNIVERSAL_SENSOR_FILTER_TYPES "")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "BatteryState")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "CameraInfo")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "CompressedImage")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "FluidPressure")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "Illuminance")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "Image")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "Imu")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "JointState")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "Joy")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "JoyFeedbackArray")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "LaserScan")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "MagneticField")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "MultiDOFJointState")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "MultiEchoLaserScan")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "NavSatFix")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "PointCloud")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "PointCloud2")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "Range")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "RelativeHumidity")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "Temperature")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES "TimeReference")

set(UNIVERSAL_SENSOR_FILTER_TYPES_NO_HEADER "")
list(APPEND UNIVERSAL_SENSOR_FILTER_TYPES_NO_HEADER "JoyFeedbackArray")

set(UNIVERSAL_SENSOR_FILTER_TYPES_ALL ${UNIVERSAL_SENSOR_FILTER_TYPES} ${UNIVERSAL_SENSOR_FILTER_TYPES_NO_HEADER})

#
# Use this macro to create and install the pluginlib XML file that registers all filters created by a call to
# REGISTER_UNIVERSAL_SENSOR_MSGS_FILTER() macro from include/sensor_filters/RegisterFilter.hpp .
#
# :param FILTER_NAME: Name of the filter.
# :type FILTER_NAME: string
# :param FILTER_DESCRIPTION: Textual description of the filter.
# :type FILTER_DESCRIPTION: string
# :param FILTER_TARGET: Name of the filter's CMake target. Defaults to ${FILTER_NAME}Filter
# :type FILTER_TARGET: string
# :param FILTER_PREFIX: Prefix of the filter (serves as C++ namespace and as prefix for the registered filters). Defaults to sensor_filters.
# :type FILTER_PREFIX: string
# :param MESSAGE_TYPES: The types of sensor_msgs messages to register the filter for.
# :type MESSAGE_TYPES: list of string
# :param MESSAGE_TYPE_PREFIX: C++ namespace prefix of the message types from MESSAGE_TYPES.
# :type MESSAGE_TYPES: string
#
# @public
#
macro(register_universal_filter_description_file arg_FILTER_NAME arg_FILTER_DESCRIPTION)
  set(options)
  set(oneValueArgs FILTER_TARGET FILTER_PREFIX MESSAGE_TYPE_PREFIX)
  set(multiValueArgs MESSAGE_TYPES)
  cmake_parse_arguments(arg "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(FILTER_NAME "${arg_FILTER_NAME}")
  set(FILTER_DESCRIPTION "${arg_FILTER_DESCRIPTION}")

  if(arg_FILTER_TARGET)
    set(FILTER_TARGET "${arg_FILTER_TARGET}")
  else()
    set(FILTER_TARGET "${FILTER_NAME}Filter")
  endif()
  
  if(arg_FILTER_PREFIX)
    set(FILTER_PREFIX "${arg_FILTER_PREFIX}")
  else()
    set(FILTER_PREFIX "sensor_filters")
  endif()
  
  if(arg_MESSAGE_TYPES)
    set(MESSAGE_TYPES "${arg_MESSAGE_TYPES}")
  else()
    set(MESSAGE_TYPES "${UNIVERSAL_SENSOR_FILTER_TYPES}")
  endif()

  if(arg_MESSAGE_TYPE_PREFIX)
    set(MESSAGE_TYPE_PREFIX "${arg_MESSAGE_TYPE_PREFIX}")
  else()
    set(MESSAGE_TYPE_PREFIX "sensor_msgs::msg::")
  endif()

  set(UNIVERSAL_FILTER_CLASSES "")
  foreach(MESSAGE_TYPE IN LISTS MESSAGE_TYPES)
    configure_file(cmake/UniversalFilterItem.xml.in "${CMAKE_CURRENT_BINARY_DIR}/${FILTER_NAME}/${MESSAGE_TYPE}.xml" @ONLY)
    file(READ "${CMAKE_CURRENT_BINARY_DIR}/${FILTER_NAME}/${MESSAGE_TYPE}.xml" universal_filter_item)
    set(UNIVERSAL_FILTER_CLASSES "${UNIVERSAL_FILTER_CLASSES}\n${universal_filter_item}")
  endforeach()
  configure_file(cmake/UniversalFilter.xml.in "${CMAKE_CURRENT_BINARY_DIR}/${FILTER_NAME}.xml")

  # pluginlib_export_plugin_description_file forces to use the file from CMAKE_CURRENT_SOURCE_DIR, but we have it in
  # CMAKE_CURRENT_BINARY_DIR, so we don't use the macro and instead use an adapted version of it
  # pluginlib_export_plugin_description_file(filters ${FILTER_NAME}.xml)

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${FILTER_NAME}.xml" DESTINATION share/${PROJECT_NAME})
  set(plugin_category filters)
  set(__PLUGINLIB_CATEGORY_CONTENT__${plugin_category}
    "${__PLUGINLIB_CATEGORY_CONTENT__${plugin_category}}share/${PROJECT_NAME}/${FILTER_NAME}.xml\n")
  list(APPEND __PLUGINLIB_PLUGIN_CATEGORIES ${plugin_category})  # duplicates are removes on use
endmacro()