// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from md_interfaces:msg/MdRobotMsg1.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_msg1__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
md_interfaces__msg__MdRobotMsg1__init(md_interfaces__msg__MdRobotMsg1 * msg)
{
  if (!msg) {
    return false;
  }
  // interval_time
  // motor1_rpm
  // motor1_pos
  // motor2_rpm
  // motor2_pos
  // motor1_state
  // motor2_state
  // input_voltage
  return true;
}

void
md_interfaces__msg__MdRobotMsg1__fini(md_interfaces__msg__MdRobotMsg1 * msg)
{
  if (!msg) {
    return;
  }
  // interval_time
  // motor1_rpm
  // motor1_pos
  // motor2_rpm
  // motor2_pos
  // motor1_state
  // motor2_state
  // input_voltage
}

bool
md_interfaces__msg__MdRobotMsg1__are_equal(const md_interfaces__msg__MdRobotMsg1 * lhs, const md_interfaces__msg__MdRobotMsg1 * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // interval_time
  if (lhs->interval_time != rhs->interval_time) {
    return false;
  }
  // motor1_rpm
  if (lhs->motor1_rpm != rhs->motor1_rpm) {
    return false;
  }
  // motor1_pos
  if (lhs->motor1_pos != rhs->motor1_pos) {
    return false;
  }
  // motor2_rpm
  if (lhs->motor2_rpm != rhs->motor2_rpm) {
    return false;
  }
  // motor2_pos
  if (lhs->motor2_pos != rhs->motor2_pos) {
    return false;
  }
  // motor1_state
  if (lhs->motor1_state != rhs->motor1_state) {
    return false;
  }
  // motor2_state
  if (lhs->motor2_state != rhs->motor2_state) {
    return false;
  }
  // input_voltage
  if (lhs->input_voltage != rhs->input_voltage) {
    return false;
  }
  return true;
}

bool
md_interfaces__msg__MdRobotMsg1__copy(
  const md_interfaces__msg__MdRobotMsg1 * input,
  md_interfaces__msg__MdRobotMsg1 * output)
{
  if (!input || !output) {
    return false;
  }
  // interval_time
  output->interval_time = input->interval_time;
  // motor1_rpm
  output->motor1_rpm = input->motor1_rpm;
  // motor1_pos
  output->motor1_pos = input->motor1_pos;
  // motor2_rpm
  output->motor2_rpm = input->motor2_rpm;
  // motor2_pos
  output->motor2_pos = input->motor2_pos;
  // motor1_state
  output->motor1_state = input->motor1_state;
  // motor2_state
  output->motor2_state = input->motor2_state;
  // input_voltage
  output->input_voltage = input->input_voltage;
  return true;
}

md_interfaces__msg__MdRobotMsg1 *
md_interfaces__msg__MdRobotMsg1__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotMsg1 * msg = (md_interfaces__msg__MdRobotMsg1 *)allocator.allocate(sizeof(md_interfaces__msg__MdRobotMsg1), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(md_interfaces__msg__MdRobotMsg1));
  bool success = md_interfaces__msg__MdRobotMsg1__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
md_interfaces__msg__MdRobotMsg1__destroy(md_interfaces__msg__MdRobotMsg1 * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    md_interfaces__msg__MdRobotMsg1__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
md_interfaces__msg__MdRobotMsg1__Sequence__init(md_interfaces__msg__MdRobotMsg1__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotMsg1 * data = NULL;

  if (size) {
    data = (md_interfaces__msg__MdRobotMsg1 *)allocator.zero_allocate(size, sizeof(md_interfaces__msg__MdRobotMsg1), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = md_interfaces__msg__MdRobotMsg1__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        md_interfaces__msg__MdRobotMsg1__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
md_interfaces__msg__MdRobotMsg1__Sequence__fini(md_interfaces__msg__MdRobotMsg1__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      md_interfaces__msg__MdRobotMsg1__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

md_interfaces__msg__MdRobotMsg1__Sequence *
md_interfaces__msg__MdRobotMsg1__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotMsg1__Sequence * array = (md_interfaces__msg__MdRobotMsg1__Sequence *)allocator.allocate(sizeof(md_interfaces__msg__MdRobotMsg1__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = md_interfaces__msg__MdRobotMsg1__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
md_interfaces__msg__MdRobotMsg1__Sequence__destroy(md_interfaces__msg__MdRobotMsg1__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    md_interfaces__msg__MdRobotMsg1__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
md_interfaces__msg__MdRobotMsg1__Sequence__are_equal(const md_interfaces__msg__MdRobotMsg1__Sequence * lhs, const md_interfaces__msg__MdRobotMsg1__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!md_interfaces__msg__MdRobotMsg1__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
md_interfaces__msg__MdRobotMsg1__Sequence__copy(
  const md_interfaces__msg__MdRobotMsg1__Sequence * input,
  md_interfaces__msg__MdRobotMsg1__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(md_interfaces__msg__MdRobotMsg1);
    md_interfaces__msg__MdRobotMsg1 * data =
      (md_interfaces__msg__MdRobotMsg1 *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!md_interfaces__msg__MdRobotMsg1__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          md_interfaces__msg__MdRobotMsg1__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!md_interfaces__msg__MdRobotMsg1__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
