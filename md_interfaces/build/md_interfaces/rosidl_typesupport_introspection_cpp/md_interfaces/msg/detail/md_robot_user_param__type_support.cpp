// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "md_interfaces/msg/detail/md_robot_user_param__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace md_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MdRobotUserParam_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) md_interfaces::msg::MdRobotUserParam(_init);
}

void MdRobotUserParam_fini_function(void * message_memory)
{
  auto typed_message = static_cast<md_interfaces::msg::MdRobotUserParam *>(message_memory);
  typed_message->~MdRobotUserParam();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MdRobotUserParam_message_member_array[5] = {
  {
    "param_bit_select",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces::msg::MdRobotUserParam, param_bit_select),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "slow_sd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<md_interfaces::msg::MdRobotSlowStartStop>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces::msg::MdRobotUserParam, slow_sd),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "max_linear_speed_usr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces::msg::MdRobotUserParam, max_linear_speed_usr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "max_angular_speed_usr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces::msg::MdRobotUserParam, max_angular_speed_usr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "brake_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(md_interfaces::msg::MdRobotUserParam, brake_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MdRobotUserParam_message_members = {
  "md_interfaces::msg",  // message namespace
  "MdRobotUserParam",  // message name
  5,  // number of fields
  sizeof(md_interfaces::msg::MdRobotUserParam),
  MdRobotUserParam_message_member_array,  // message members
  MdRobotUserParam_init_function,  // function to initialize message memory (memory has to be allocated)
  MdRobotUserParam_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MdRobotUserParam_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MdRobotUserParam_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace md_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<md_interfaces::msg::MdRobotUserParam>()
{
  return &::md_interfaces::msg::rosidl_typesupport_introspection_cpp::MdRobotUserParam_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, md_interfaces, msg, MdRobotUserParam)() {
  return &::md_interfaces::msg::rosidl_typesupport_introspection_cpp::MdRobotUserParam_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
