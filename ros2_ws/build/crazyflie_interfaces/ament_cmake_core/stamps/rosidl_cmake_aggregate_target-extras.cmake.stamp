# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target crazyflie_interfaces::crazyflie_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${crazyflie_interfaces_TARGETS}.
if(crazyflie_interfaces_TARGETS AND NOT TARGET crazyflie_interfaces::crazyflie_interfaces)
  add_library(crazyflie_interfaces::crazyflie_interfaces INTERFACE IMPORTED)
  set_target_properties(crazyflie_interfaces::crazyflie_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${crazyflie_interfaces_TARGETS}")
endif()
