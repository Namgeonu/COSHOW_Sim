// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:srv/Arm.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_interfaces/srv/arm.h"


#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__STRUCT_H_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Arm in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__Arm_Request
{
  bool arm;
} crazyflie_interfaces__srv__Arm_Request;

// Struct for a sequence of crazyflie_interfaces__srv__Arm_Request.
typedef struct crazyflie_interfaces__srv__Arm_Request__Sequence
{
  crazyflie_interfaces__srv__Arm_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__Arm_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Arm in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__Arm_Response
{
  uint8_t structure_needs_at_least_one_member;
} crazyflie_interfaces__srv__Arm_Response;

// Struct for a sequence of crazyflie_interfaces__srv__Arm_Response.
typedef struct crazyflie_interfaces__srv__Arm_Response__Sequence
{
  crazyflie_interfaces__srv__Arm_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__Arm_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  crazyflie_interfaces__srv__Arm_Event__request__MAX_SIZE = 1
};
// response
enum
{
  crazyflie_interfaces__srv__Arm_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Arm in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__Arm_Event
{
  service_msgs__msg__ServiceEventInfo info;
  crazyflie_interfaces__srv__Arm_Request__Sequence request;
  crazyflie_interfaces__srv__Arm_Response__Sequence response;
} crazyflie_interfaces__srv__Arm_Event;

// Struct for a sequence of crazyflie_interfaces__srv__Arm_Event.
typedef struct crazyflie_interfaces__srv__Arm_Event__Sequence
{
  crazyflie_interfaces__srv__Arm_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__Arm_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__STRUCT_H_
