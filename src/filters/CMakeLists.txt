# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Register universal filters

include(cmake/UniversalFilter.cmake)

macro(register_universal_filter FILTER_NAME FILTER_DESCRIPTION)
  set(FILTER_TARGET "${FILTER_NAME}Filter")

  add_library(${FILTER_TARGET} SHARED src/filters/${FILTER_NAME}.cpp)
  target_link_libraries(${FILTER_TARGET} PRIVATE ${LIBRARY_TARGETS})
  target_include_directories(${FILTER_TARGET} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  register_universal_filter_description_file("${FILTER_NAME}" "${FILTER_DESCRIPTION}" FILTER_TARGET "${FILTER_TARGET}")

  install(TARGETS ${FILTER_TARGET}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endmacro()

register_universal_filter(ChangeHeader "Change the header of a message.")
