// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_user_param__rosidl_typesupport_fastrtps_cpp.hpp"
#include "md_interfaces/msg/detail/md_robot_user_param__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace md_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const md_interfaces::msg::MdRobotSlowStartStop &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  md_interfaces::msg::MdRobotSlowStartStop &);
size_t get_serialized_size(
  const md_interfaces::msg::MdRobotSlowStartStop &,
  size_t current_alignment);
size_t
max_serialized_size_MdRobotSlowStartStop(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace md_interfaces


namespace md_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_md_interfaces
cdr_serialize(
  const md_interfaces::msg::MdRobotUserParam & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: param_bit_select
  cdr << ros_message.param_bit_select;
  // Member: slow_sd
  md_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.slow_sd,
    cdr);
  // Member: max_linear_speed_usr
  cdr << ros_message.max_linear_speed_usr;
  // Member: max_angular_speed_usr
  cdr << ros_message.max_angular_speed_usr;
  // Member: brake_type
  cdr << ros_message.brake_type;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_md_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  md_interfaces::msg::MdRobotUserParam & ros_message)
{
  // Member: param_bit_select
  cdr >> ros_message.param_bit_select;

  // Member: slow_sd
  md_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.slow_sd);

  // Member: max_linear_speed_usr
  cdr >> ros_message.max_linear_speed_usr;

  // Member: max_angular_speed_usr
  cdr >> ros_message.max_angular_speed_usr;

  // Member: brake_type
  cdr >> ros_message.brake_type;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_md_interfaces
get_serialized_size(
  const md_interfaces::msg::MdRobotUserParam & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: param_bit_select
  {
    size_t item_size = sizeof(ros_message.param_bit_select);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: slow_sd

  current_alignment +=
    md_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.slow_sd, current_alignment);
  // Member: max_linear_speed_usr
  {
    size_t item_size = sizeof(ros_message.max_linear_speed_usr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_angular_speed_usr
  {
    size_t item_size = sizeof(ros_message.max_angular_speed_usr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: brake_type
  {
    size_t item_size = sizeof(ros_message.brake_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_md_interfaces
max_serialized_size_MdRobotUserParam(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: param_bit_select
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: slow_sd
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        md_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_MdRobotSlowStartStop(
        full_bounded, current_alignment);
    }
  }

  // Member: max_linear_speed_usr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: max_angular_speed_usr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: brake_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _MdRobotUserParam__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const md_interfaces::msg::MdRobotUserParam *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MdRobotUserParam__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<md_interfaces::msg::MdRobotUserParam *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MdRobotUserParam__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const md_interfaces::msg::MdRobotUserParam *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MdRobotUserParam__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_MdRobotUserParam(full_bounded, 0);
}

static message_type_support_callbacks_t _MdRobotUserParam__callbacks = {
  "md_interfaces::msg",
  "MdRobotUserParam",
  _MdRobotUserParam__cdr_serialize,
  _MdRobotUserParam__cdr_deserialize,
  _MdRobotUserParam__get_serialized_size,
  _MdRobotUserParam__max_serialized_size
};

static rosidl_message_type_support_t _MdRobotUserParam__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MdRobotUserParam__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace md_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_md_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<md_interfaces::msg::MdRobotUserParam>()
{
  return &md_interfaces::msg::typesupport_fastrtps_cpp::_MdRobotUserParam__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, md_interfaces, msg, MdRobotUserParam)() {
  return &md_interfaces::msg::typesupport_fastrtps_cpp::_MdRobotUserParam__handle;
}

#ifdef __cplusplus
}
#endif
