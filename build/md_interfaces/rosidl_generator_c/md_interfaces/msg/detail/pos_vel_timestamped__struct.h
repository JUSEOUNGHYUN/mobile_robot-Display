// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from md_interfaces:msg/PosVelTimestamped.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__STRUCT_H_
#define MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in msg/PosVelTimestamped in the package md_interfaces.
typedef struct md_interfaces__msg__PosVelTimestamped
{
  int32_t motor_pos1;
  int16_t motor_vel1;
  int32_t motor_pos2;
  int32_t motor_vel2;
  builtin_interfaces__msg__Time stamp;
} md_interfaces__msg__PosVelTimestamped;

// Struct for a sequence of md_interfaces__msg__PosVelTimestamped.
typedef struct md_interfaces__msg__PosVelTimestamped__Sequence
{
  md_interfaces__msg__PosVelTimestamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} md_interfaces__msg__PosVelTimestamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__STRUCT_H_
