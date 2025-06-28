// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from md_interfaces:msg/MdMotorTicksAndVelo.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__STRUCT_H_
#define MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MdMotorTicksAndVelo in the package md_interfaces.
typedef struct md_interfaces__msg__MdMotorTicksAndVelo
{
  int32_t left_ticks;
  int16_t left_rpm;
  int32_t right_ticks;
  int16_t right_rpm;
} md_interfaces__msg__MdMotorTicksAndVelo;

// Struct for a sequence of md_interfaces__msg__MdMotorTicksAndVelo.
typedef struct md_interfaces__msg__MdMotorTicksAndVelo__Sequence
{
  md_interfaces__msg__MdMotorTicksAndVelo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} md_interfaces__msg__MdMotorTicksAndVelo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__STRUCT_H_
