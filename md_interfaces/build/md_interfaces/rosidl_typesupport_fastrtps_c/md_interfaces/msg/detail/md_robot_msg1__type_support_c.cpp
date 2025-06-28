// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from md_interfaces:msg/MdRobotMsg1.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_msg1__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "md_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "md_interfaces/msg/detail/md_robot_msg1__struct.h"
#include "md_interfaces/msg/detail/md_robot_msg1__functions.h"
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


// forward declare type support functions


using _MdRobotMsg1__ros_msg_type = md_interfaces__msg__MdRobotMsg1;

static bool _MdRobotMsg1__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MdRobotMsg1__ros_msg_type * ros_message = static_cast<const _MdRobotMsg1__ros_msg_type *>(untyped_ros_message);
  // Field name: interval_time
  {
    cdr << ros_message->interval_time;
  }

  // Field name: motor1_rpm
  {
    cdr << ros_message->motor1_rpm;
  }

  // Field name: motor1_pos
  {
    cdr << ros_message->motor1_pos;
  }

  // Field name: motor2_rpm
  {
    cdr << ros_message->motor2_rpm;
  }

  // Field name: motor2_pos
  {
    cdr << ros_message->motor2_pos;
  }

  // Field name: motor1_state
  {
    cdr << ros_message->motor1_state;
  }

  // Field name: motor2_state
  {
    cdr << ros_message->motor2_state;
  }

  // Field name: input_voltage
  {
    cdr << ros_message->input_voltage;
  }

  return true;
}

static bool _MdRobotMsg1__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MdRobotMsg1__ros_msg_type * ros_message = static_cast<_MdRobotMsg1__ros_msg_type *>(untyped_ros_message);
  // Field name: interval_time
  {
    cdr >> ros_message->interval_time;
  }

  // Field name: motor1_rpm
  {
    cdr >> ros_message->motor1_rpm;
  }

  // Field name: motor1_pos
  {
    cdr >> ros_message->motor1_pos;
  }

  // Field name: motor2_rpm
  {
    cdr >> ros_message->motor2_rpm;
  }

  // Field name: motor2_pos
  {
    cdr >> ros_message->motor2_pos;
  }

  // Field name: motor1_state
  {
    cdr >> ros_message->motor1_state;
  }

  // Field name: motor2_state
  {
    cdr >> ros_message->motor2_state;
  }

  // Field name: input_voltage
  {
    cdr >> ros_message->input_voltage;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_md_interfaces
size_t get_serialized_size_md_interfaces__msg__MdRobotMsg1(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MdRobotMsg1__ros_msg_type * ros_message = static_cast<const _MdRobotMsg1__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name interval_time
  {
    size_t item_size = sizeof(ros_message->interval_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor1_rpm
  {
    size_t item_size = sizeof(ros_message->motor1_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor1_pos
  {
    size_t item_size = sizeof(ros_message->motor1_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor2_rpm
  {
    size_t item_size = sizeof(ros_message->motor2_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor2_pos
  {
    size_t item_size = sizeof(ros_message->motor2_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor1_state
  {
    size_t item_size = sizeof(ros_message->motor1_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor2_state
  {
    size_t item_size = sizeof(ros_message->motor2_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name input_voltage
  {
    size_t item_size = sizeof(ros_message->input_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MdRobotMsg1__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_md_interfaces__msg__MdRobotMsg1(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_md_interfaces
size_t max_serialized_size_md_interfaces__msg__MdRobotMsg1(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: interval_time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: motor1_rpm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: motor1_pos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: motor2_rpm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: motor2_pos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: motor1_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: motor2_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: input_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _MdRobotMsg1__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_md_interfaces__msg__MdRobotMsg1(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MdRobotMsg1 = {
  "md_interfaces::msg",
  "MdRobotMsg1",
  _MdRobotMsg1__cdr_serialize,
  _MdRobotMsg1__cdr_deserialize,
  _MdRobotMsg1__get_serialized_size,
  _MdRobotMsg1__max_serialized_size
};

static rosidl_message_type_support_t _MdRobotMsg1__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MdRobotMsg1,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, md_interfaces, msg, MdRobotMsg1)() {
  return &_MdRobotMsg1__type_support;
}

#if defined(__cplusplus)
}
#endif
