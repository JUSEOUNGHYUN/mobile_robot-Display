// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from md_interfaces:msg/MdRobotSlowStartStop.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__rosidl_typesupport_introspection_c.h"
#include "md_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__functions.h"
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  md_interfaces__msg__MdRobotSlowStartStop__init(message_memory);
}

void MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_fini_function(void * message_memory)
{
  md_interfaces__msg__MdRobotSlowStartStop__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_message_member_array[2] = {
  {
    "slowstart",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__MdRobotSlowStartStop, slowstart),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "slowstop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces__msg__MdRobotSlowStartStop, slowstop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_message_members = {
  "md_interfaces__msg",  // message namespace
  "MdRobotSlowStartStop",  // message name
  2,  // number of fields
  sizeof(md_interfaces__msg__MdRobotSlowStartStop),
  MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_message_member_array,  // message members
  MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_init_function,  // function to initialize message memory (memory has to be allocated)
  MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_message_type_support_handle = {
  0,
  &MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_md_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, md_interfaces, msg, MdRobotSlowStartStop)() {
  if (!MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_message_type_support_handle.typesupport_identifier) {
    MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MdRobotSlowStartStop__rosidl_typesupport_introspection_c__MdRobotSlowStartStop_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
