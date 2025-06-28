// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from md_interfaces:msg/MdRobotMsg2.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__STRUCT_H_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MdRobotMsg2 in the package md_interfaces.
typedef struct md_interfaces__msg__MdRobotMsg2
{
  double interval_time;
  int32_t x_pos;
  int32_t y_pos;
  int16_t angule;
  uint8_t us_1;
  uint8_t us_2;
  uint8_t us_3;
  uint8_t us_4;
  uint8_t platform_state;
  int16_t linear_velocity;
  int16_t angular_velocity;
  float input_voltage;
} md_interfaces__msg__MdRobotMsg2;

// Struct for a sequence of md_interfaces__msg__MdRobotMsg2.
typedef struct md_interfaces__msg__MdRobotMsg2__Sequence
{
  md_interfaces__msg__MdRobotMsg2 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} md_interfaces__msg__MdRobotMsg2__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__STRUCT_H_
