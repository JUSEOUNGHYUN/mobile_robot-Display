// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "md_interfaces/msg/detail/md_robot_user_param__rosidl_typesupport_introspection_c.h"
#include "md_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "md_interfaces/msg/detail/md_robot_user_param__functions.h"
#include "md_interfaces/msg/detail/md_robot_user_param__struct.h"


// Include directives for member types
// Member `slow_sd`
#include "md_interfaces/msg/md_robot_slow_start_stop.h"
// Member `slow_sd`
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  md_interfaces__msg__MdRobotUserParam__init(message_memory);
}

void MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_fini_function(void * message_memory)
{
  md_interfaces__msg__MdRobotUserParam__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_member_array[5] = {
  {
    "param_bit_select",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__MdRobotUserParam, param_bit_select),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "slow_sd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__MdRobotUserParam, slow_sd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_linear_speed_usr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__MdRobotUserParam, max_linear_speed_usr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_angular_speed_usr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__MdRobotUserParam, max_angular_speed_usr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "brake_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__MdRobotUserParam, brake_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_members = {
  "md_interfaces__msg",  // message namespace
  "MdRobotUserParam",  // message name
  5,  // number of fields
  sizeof(md_interfaces__msg__MdRobotUserParam),
  MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_member_array,  // message members
  MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_init_function,  // function to initialize message memory (memory has to be allocated)
  MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_type_support_handle = {
  0,
  &MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_md_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, md_interfaces, msg, MdRobotUserParam)() {
  MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, md_interfaces, msg, MdRobotSlowStartStop)();
  if (!MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_type_support_handle.typesupport_identifier) {
    MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MdRobotUserParam__rosidl_typesupport_introspection_c__MdRobotUserParam_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
