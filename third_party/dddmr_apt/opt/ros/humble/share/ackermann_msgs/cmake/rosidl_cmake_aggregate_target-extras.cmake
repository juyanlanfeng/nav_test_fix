# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target ackermann_msgs::ackermann_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${ackermann_msgs_TARGETS}.
if(ackermann_msgs_TARGETS AND NOT TARGET ackermann_msgs::ackermann_msgs)
  add_library(ackermann_msgs::ackermann_msgs INTERFACE IMPORTED)
  set_target_properties(ackermann_msgs::ackermann_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ackermann_msgs_TARGETS}")
endif()
