// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from md_interfaces:msg/PosVelTimestamped.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "md_interfaces/msg/detail/pos_vel_timestamped__rosidl_typesupport_introspection_c.h"
#include "md_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "md_interfaces/msg/detail/pos_vel_timestamped__functions.h"
#include "md_interfaces/msg/detail/pos_vel_timestamped__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  md_interfaces__msg__PosVelTimestamped__init(message_memory);
}

void PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_fini_function(void * message_memory)
{
  md_interfaces__msg__PosVelTimestamped__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_member_array[5] = {
  {
    "motor_pos1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__PosVelTimestamped, motor_pos1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_vel1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__PosVelTimestamped, motor_vel1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_pos2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__PosVelTimestamped, motor_pos2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_vel2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__PosVelTimestamped, motor_vel2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__PosVelTimestamped, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_members = {
  "md_interfaces__msg",  // message namespace
  "PosVelTimestamped",  // message name
  5,  // number of fields
  sizeof(md_interfaces__msg__PosVelTimestamped),
  PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_member_array,  // message members
  PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_init_function,  // function to initialize message memory (memory has to be allocated)
  PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_type_support_handle = {
  0,
  &PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_md_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, md_interfaces, msg, PosVelTimestamped)() {
  PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_type_support_handle.typesupport_identifier) {
    PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PosVelTimestamped__rosidl_typesupport_introspection_c__PosVelTimestamped_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
