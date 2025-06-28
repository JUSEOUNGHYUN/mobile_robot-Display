// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from md_interfaces:msg/MdRobotSlowStartStop.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__STRUCT_H_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MdRobotSlowStartStop in the package md_interfaces.
typedef struct md_interfaces__msg__MdRobotSlowStartStop
{
  uint16_t slowstart;
  uint16_t slowstop;
} md_interfaces__msg__MdRobotSlowStartStop;

// Struct for a sequence of md_interfaces__msg__MdRobotSlowStartStop.
typedef struct md_interfaces__msg__MdRobotSlowStartStop__Sequence
{
  md_interfaces__msg__MdRobotSlowStartStop * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} md_interfaces__msg__MdRobotSlowStartStop__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__STRUCT_H_
