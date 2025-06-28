// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from md_interfaces:msg/PosVelTimestamped.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/pos_vel_timestamped__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "md_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "md_interfaces/msg/detail/pos_vel_timestamped__struct.h"
#include "md_interfaces/msg/detail/pos_vel_timestamped__functions.h"
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

#include "builtin_interfaces/msg/detail/time__functions.h"  // stamp

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_md_interfaces
size_t get_serialized_size_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_md_interfaces
size_t max_serialized_size_builtin_interfaces__msg__Time(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_md_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time)();


using _PosVelTimestamped__ros_msg_type = md_interfaces__msg__PosVelTimestamped;

static bool _PosVelTimestamped__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PosVelTimestamped__ros_msg_type * ros_message = static_cast<const _PosVelTimestamped__ros_msg_type *>(untyped_ros_message);
  // Field name: motor_pos1
  {
    cdr << ros_message->motor_pos1;
  }

  // Field name: motor_vel1
  {
    cdr << ros_message->motor_vel1;
  }

  // Field name: motor_pos2
  {
    cdr << ros_message->motor_pos2;
  }

  // Field name: motor_vel2
  {
    cdr << ros_message->motor_vel2;
  }

  // Field name: stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->stamp, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _PosVelTimestamped__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PosVelTimestamped__ros_msg_type * ros_message = static_cast<_PosVelTimestamped__ros_msg_type *>(untyped_ros_message);
  // Field name: motor_pos1
  {
    cdr >> ros_message->motor_pos1;
  }

  // Field name: motor_vel1
  {
    cdr >> ros_message->motor_vel1;
  }

  // Field name: motor_pos2
  {
    cdr >> ros_message->motor_pos2;
  }

  // Field name: motor_vel2
  {
    cdr >> ros_message->motor_vel2;
  }

  // Field name: stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->stamp))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_md_interfaces
size_t get_serialized_size_md_interfaces__msg__PosVelTimestamped(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PosVelTimestamped__ros_msg_type * ros_message = static_cast<const _PosVelTimestamped__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name motor_pos1
  {
    size_t item_size = sizeof(ros_message->motor_pos1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor_vel1
  {
    size_t item_size = sizeof(ros_message->motor_vel1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor_pos2
  {
    size_t item_size = sizeof(ros_message->motor_pos2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor_vel2
  {
    size_t item_size = sizeof(ros_message->motor_vel2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stamp

  current_alignment += get_serialized_size_builtin_interfaces__msg__Time(
    &(ros_message->stamp), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _PosVelTimestamped__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_md_interfaces__msg__PosVelTimestamped(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_md_interfaces
size_t max_serialized_size_md_interfaces__msg__PosVelTimestamped(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: motor_pos1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: motor_vel1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: motor_pos2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: motor_vel2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: stamp
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_builtin_interfaces__msg__Time(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _PosVelTimestamped__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_md_interfaces__msg__PosVelTimestamped(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_PosVelTimestamped = {
  "md_interfaces::msg",
  "PosVelTimestamped",
  _PosVelTimestamped__cdr_serialize,
  _PosVelTimestamped__cdr_deserialize,
  _PosVelTimestamped__get_serialized_size,
  _PosVelTimestamped__max_serialized_size
};

static rosidl_message_type_support_t _PosVelTimestamped__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PosVelTimestamped,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, md_interfaces, msg, PosVelTimestamped)() {
  return &_PosVelTimestamped__type_support;
}

#if defined(__cplusplus)
}
#endif
