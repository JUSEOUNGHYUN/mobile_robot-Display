// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from md_interfaces:msg/MdRobotSlowStartStop.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__rosidl_typesupport_fastrtps_cpp.hpp"
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__struct.hpp"

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

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_md_interfaces
cdr_serialize(
  const md_interfaces::msg::MdRobotSlowStartStop & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: slowstart
  cdr << ros_message.slowstart;
  // Member: slowstop
  cdr << ros_message.slowstop;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_md_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  md_interfaces::msg::MdRobotSlowStartStop & ros_message)
{
  // Member: slowstart
  cdr >> ros_message.slowstart;

  // Member: slowstop
  cdr >> ros_message.slowstop;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_md_interfaces
get_serialized_size(
  const md_interfaces::msg::MdRobotSlowStartStop & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: slowstart
  {
    size_t item_size = sizeof(ros_message.slowstart);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: slowstop
  {
    size_t item_size = sizeof(ros_message.slowstop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_md_interfaces
max_serialized_size_MdRobotSlowStartStop(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: slowstart
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: slowstop
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  return current_alignment - initial_alignment;
}

static bool _MdRobotSlowStartStop__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const md_interfaces::msg::MdRobotSlowStartStop *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MdRobotSlowStartStop__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<md_interfaces::msg::MdRobotSlowStartStop *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MdRobotSlowStartStop__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const md_interfaces::msg::MdRobotSlowStartStop *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MdRobotSlowStartStop__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_MdRobotSlowStartStop(full_bounded, 0);
}

static message_type_support_callbacks_t _MdRobotSlowStartStop__callbacks = {
  "md_interfaces::msg",
  "MdRobotSlowStartStop",
  _MdRobotSlowStartStop__cdr_serialize,
  _MdRobotSlowStartStop__cdr_deserialize,
  _MdRobotSlowStartStop__get_serialized_size,
  _MdRobotSlowStartStop__max_serialized_size
};

static rosidl_message_type_support_t _MdRobotSlowStartStop__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MdRobotSlowStartStop__callbacks,
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
get_message_type_support_handle<md_interfaces::msg::MdRobotSlowStartStop>()
{
  return &md_interfaces::msg::typesupport_fastrtps_cpp::_MdRobotSlowStartStop__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, md_interfaces, msg, MdRobotSlowStartStop)() {
  return &md_interfaces::msg::typesupport_fastrtps_cpp::_MdRobotSlowStartStop__handle;
}

#ifdef __cplusplus
}
#endif
