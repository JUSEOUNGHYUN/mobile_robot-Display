// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_user_param__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "md_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "md_interfaces/msg/detail/md_robot_user_param__struct.h"
#include "md_interfaces/msg/detail/md_robot_user_param__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "md_interfaces/msg/detail/md_robot_slow_start_stop__functions.h"  // slow_sd

// forward declare type support functions
size_t get_serialized_size_md_interfaces__msg__MdRobotSlowStartStop(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_md_interfaces__msg__MdRobotSlowStartStop(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, md_interfaces, msg, MdRobotSlowStartStop)();


using _MdRobotUserParam__ros_msg_type = md_interfaces__msg__MdRobotUserParam;

static bool _MdRobotUserParam__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MdRobotUserParam__ros_msg_type * ros_message = static_cast<const _MdRobotUserParam__ros_msg_type *>(untyped_ros_message);
  // Field name: param_bit_select
  {
    cdr << ros_message->param_bit_select;
  }

  // Field name: slow_sd
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, md_interfaces, msg, MdRobotSlowStartStop
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->slow_sd, cdr))
    {
      return false;
    }
  }

  // Field name: max_linear_speed_usr
  {
    cdr << ros_message->max_linear_speed_usr;
  }

  // Field name: max_angular_speed_usr
  {
    cdr << ros_message->max_angular_speed_usr;
  }

  // Field name: brake_type
  {
    cdr << ros_message->brake_type;
  }

  return true;
}

static bool _MdRobotUserParam__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MdRobotUserParam__ros_msg_type * ros_message = static_cast<_MdRobotUserParam__ros_msg_type *>(untyped_ros_message);
  // Field name: param_bit_select
  {
    cdr >> ros_message->param_bit_select;
  }

  // Field name: slow_sd
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, md_interfaces, msg, MdRobotSlowStartStop
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->slow_sd))
    {
      return false;
    }
  }

  // Field name: max_linear_speed_usr
  {
    cdr >> ros_message->max_linear_speed_usr;
  }

  // Field name: max_angular_speed_usr
  {
    cdr >> ros_message->max_angular_speed_usr;
  }

  // Field name: brake_type
  {
    cdr >> ros_message->brake_type;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_md_interfaces
size_t get_serialized_size_md_interfaces__msg__MdRobotUserParam(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MdRobotUserParam__ros_msg_type * ros_message = static_cast<const _MdRobotUserParam__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name param_bit_select
  {
    size_t item_size = sizeof(ros_message->param_bit_select);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name slow_sd

  current_alignment += get_serialized_size_md_interfaces__msg__MdRobotSlowStartStop(
    &(ros_message->slow_sd), current_alignment);
  // field.name max_linear_speed_usr
  {
    size_t item_size = sizeof(ros_message->max_linear_speed_usr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_angular_speed_usr
  {
    size_t item_size = sizeof(ros_message->max_angular_speed_usr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name brake_type
  {
    size_t item_size = sizeof(ros_message->brake_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MdRobotUserParam__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_md_interfaces__msg__MdRobotUserParam(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_md_interfaces
size_t max_serialized_size_md_interfaces__msg__MdRobotUserParam(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: param_bit_select
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: slow_sd
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_md_interfaces__msg__MdRobotSlowStartStop(
        full_bounded, current_alignment);
    }
  }
  // member: max_linear_speed_usr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: max_angular_speed_usr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: brake_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _MdRobotUserParam__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_md_interfaces__msg__MdRobotUserParam(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MdRobotUserParam = {
  "md_interfaces::msg",
  "MdRobotUserParam",
  _MdRobotUserParam__cdr_serialize,
  _MdRobotUserParam__cdr_deserialize,
  _MdRobotUserParam__get_serialized_size,
  _MdRobotUserParam__max_serialized_size
};

static rosidl_message_type_support_t _MdRobotUserParam__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MdRobotUserParam,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, md_interfaces, msg, MdRobotUserParam)() {
  return &_MdRobotUserParam__type_support;
}

#if defined(__cplusplus)
}
#endif
