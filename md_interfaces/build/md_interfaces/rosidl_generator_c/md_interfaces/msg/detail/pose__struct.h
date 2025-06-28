// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from md_interfaces:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__POSE__STRUCT_H_
#define MD_INTERFACES__MSG__DETAIL__POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Pose in the package md_interfaces.
typedef struct md_interfaces__msg__Pose
{
  float x;
  float y;
  float theta;
  float linear_velocity;
  float angular_velocity;
  uint8_t us_1;
  uint8_t us_2;
  uint8_t us_3;
  uint8_t us_4;
  uint8_t platform_state;
  uint8_t left_motor_state;
  uint8_t right_motor_state;
} md_interfaces__msg__Pose;

// Struct for a sequence of md_interfaces__msg__Pose.
typedef struct md_interfaces__msg__Pose__Sequence
{
  md_interfaces__msg__Pose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} md_interfaces__msg__Pose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MD_INTERFACES__MSG__DETAIL__POSE__STRUCT_H_
