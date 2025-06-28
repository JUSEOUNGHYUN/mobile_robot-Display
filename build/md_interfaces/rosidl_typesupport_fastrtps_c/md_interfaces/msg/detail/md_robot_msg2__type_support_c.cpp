// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from md_interfaces:msg/MdRobotMsg2.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_msg2__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "md_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "md_interfaces/msg/detail/md_robot_msg2__struct.h"
#include "md_interfaces/msg/detail/md_robot_msg2__functions.h"
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


using _MdRobotMsg2__ros_msg_type = md_interfaces__msg__MdRobotMsg2;

static bool _MdRobotMsg2__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MdRobotMsg2__ros_msg_type * ros_message = static_cast<const _MdRobotMsg2__ros_msg_type *>(untyped_ros_message);
  // Field name: interval_time
  {
    cdr << ros_message->interval_time;
  }

  // Field name: x_pos
  {
    cdr << ros_message->x_pos;
  }

  // Field name: y_pos
  {
    cdr << ros_message->y_pos;
  }

  // Field name: angule
  {
    cdr << ros_message->angule;
  }

  // Field name: us_1
  {
    cdr << ros_message->us_1;
  }

  // Field name: us_2
  {
    cdr << ros_message->us_2;
  }

  // Field name: us_3
  {
    cdr << ros_message->us_3;
  }

  // Field name: us_4
  {
    cdr << ros_message->us_4;
  }

  // Field name: platform_state
  {
    cdr << ros_message->platform_state;
  }

  // Field name: linear_velocity
  {
    cdr << ros_message->linear_velocity;
  }

  // Field name: angular_velocity
  {
    cdr << ros_message->angular_velocity;
  }

  // Field name: input_voltage
  {
    cdr << ros_message->input_voltage;
  }

  return true;
}

static bool _MdRobotMsg2__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MdRobotMsg2__ros_msg_type * ros_message = static_cast<_MdRobotMsg2__ros_msg_type *>(untyped_ros_message);
  // Field name: interval_time
  {
    cdr >> ros_message->interval_time;
  }

  // Field name: x_pos
  {
    cdr >> ros_message->x_pos;
  }

  // Field name: y_pos
  {
    cdr >> ros_message->y_pos;
  }

  // Field name: angule
  {
    cdr >> ros_message->angule;
  }

  // Field name: us_1
  {
    cdr >> ros_message->us_1;
  }

  // Field name: us_2
  {
    cdr >> ros_message->us_2;
  }

  // Field name: us_3
  {
    cdr >> ros_message->us_3;
  }

  // Field name: us_4
  {
    cdr >> ros_message->us_4;
  }

  // Field name: platform_state
  {
    cdr >> ros_message->platform_state;
  }

  // Field name: linear_velocity
  {
    cdr >> ros_message->linear_velocity;
  }

  // Field name: angular_velocity
  {
    cdr >> ros_message->angular_velocity;
  }

  // Field name: input_voltage
  {
    cdr >> ros_message->input_voltage;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_md_interfaces
size_t get_serialized_size_md_interfaces__msg__MdRobotMsg2(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MdRobotMsg2__ros_msg_type * ros_message = static_cast<const _MdRobotMsg2__ros_msg_type *>(untyped_ros_message);
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
  // field.name x_pos
  {
    size_t item_size = sizeof(ros_message->x_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y_pos
  {
    size_t item_size = sizeof(ros_message->y_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angule
  {
    size_t item_size = sizeof(ros_message->angule);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name us_1
  {
    size_t item_size = sizeof(ros_message->us_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name us_2
  {
    size_t item_size = sizeof(ros_message->us_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name us_3
  {
    size_t item_size = sizeof(ros_message->us_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name us_4
  {
    size_t item_size = sizeof(ros_message->us_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name platform_state
  {
    size_t item_size = sizeof(ros_message->platform_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity
  {
    size_t item_size = sizeof(ros_message->linear_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angular_velocity
  {
    size_t item_size = sizeof(ros_message->angular_velocity);
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

static uint32_t _MdRobotMsg2__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_md_interfaces__msg__MdRobotMsg2(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_md_interfaces
size_t max_serialized_size_md_interfaces__msg__MdRobotMsg2(
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
  // member: x_pos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: y_pos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: angule
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: us_1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: us_2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: us_3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: us_4
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: platform_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: angular_velocity
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: input_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _MdRobotMsg2__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_md_interfaces__msg__MdRobotMsg2(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MdRobotMsg2 = {
  "md_interfaces::msg",
  "MdRobotMsg2",
  _MdRobotMsg2__cdr_serialize,
  _MdRobotMsg2__cdr_deserialize,
  _MdRobotMsg2__get_serialized_size,
  _MdRobotMsg2__max_serialized_size
};

static rosidl_message_type_support_t _MdRobotMsg2__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MdRobotMsg2,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, md_interfaces, msg, MdRobotMsg2)() {
  return &_MdRobotMsg2__type_support;
}

#if defined(__cplusplus)
}
#endif
