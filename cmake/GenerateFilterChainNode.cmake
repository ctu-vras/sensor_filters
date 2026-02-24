set(MESSAGE_TYPE "sensor_msgs::msg::${MESSAGE_TYPE}")
set(MESSAGE_HEADER "sensor_msgs/msg/${HEADER_NAME}.hpp")

if (BASE_FILTER)
    set(TEMPLATE_ARGS "${MESSAGE_TYPE}, sensor_filters::${BASE_FILTER}")
    set(BASE_INCLUDE "#include <sensor_filters/${BASE_FILTER}.hpp>")
else ()
    set(TEMPLATE_ARGS "${MESSAGE_TYPE}")
    set(BASE_INCLUDE "")
endif ()

configure_file(${PROJECT_SOURCE_DIR}/cmake/FilterChainNode.cpp.in ${OUTPUT_FILE} @ONLY)
