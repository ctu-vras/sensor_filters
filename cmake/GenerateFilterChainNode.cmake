# ${MESSAGE_TYPE} = sensor_msgs::msg::CompressedImage
# ${NODE_NAME} = image_filter_chain

#string(REGEX REPLACE "(_|^)([a-zA-Z])" "\\U$1\\E" FILE_NAME ${NODE_NAME})
#string(APPEND FILE_NAME "FilterChainNode.cpp")
#string(REPLACE "::" "/" MESSAGE_HEADER ${MESSAGE_TYPE})
#string(APPEND MESSAGE_HEADER ".hpp")

set(MESSAGE_TYPE "sensor_msgs::msg::${MESSAGE_TYPE}")
set(MESSAGE_HEADER "sensor_msgs/msg/${HEADER_NAME}.hpp")

message(MESSAGE_TYPE="${MESSAGE_TYPE}")
message(HEADER_NAME="${HEADER_NAME}")
message(MESSAGE_HEADER="${MESSAGE_HEADER}")
if (BASE_FILTER)
    set(TEMPLATE_ARGS "${MESSAGE_TYPE}, sensor_filters::${BASE_FILTER}")
    # string(REPLACE "::" "/" "${BASE_FILTER}")
    set(BASE_INCLUDE "#include <sensor_filters/${BASE_FILTER}.hpp>")
else ()
    set(TEMPLATE_ARGS "${MESSAGE_TYPE}")
    set(BASE_INCLUDE "")
endif ()
message(NODE_NAME="${NODE_NAME}")
message(BASE_FILTER="${BASE_FILTER}")
message(TEMPLATE_ARGS="${TEMPLATE_ARGS}")

# Configure the header file
configure_file(${PROJECT_SOURCE_DIR}/cmake/FilterChainNode.cpp.in ${OUTPUT_FILE} @ONLY)

file(READ ${OUTPUT_FILE} CONTENTS)
message(CONTENTS=${CONTENTS})
