// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__STRUCT_H_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'slow_sd'
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__struct.h"

// Struct defined in msg/MdRobotUserParam in the package md_interfaces.
typedef struct md_interfaces__msg__MdRobotUserParam
{
  uint16_t param_bit_select;
  md_interfaces__msg__MdRobotSlowStartStop slow_sd;
  double max_linear_speed_usr;
  double max_angular_speed_usr;
  uint8_t brake_type;
} md_interfaces__msg__MdRobotUserParam;

// Struct for a sequence of md_interfaces__msg__MdRobotUserParam.
typedef struct md_interfaces__msg__MdRobotUserParam__Sequence
{
  md_interfaces__msg__MdRobotUserParam * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} md_interfaces__msg__MdRobotUserParam__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__STRUCT_H_
