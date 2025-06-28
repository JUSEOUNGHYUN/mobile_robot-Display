// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_user_param__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `slow_sd`
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__functions.h"

bool
md_interfaces__msg__MdRobotUserParam__init(md_interfaces__msg__MdRobotUserParam * msg)
{
  if (!msg) {
    return false;
  }
  // param_bit_select
  // slow_sd
  if (!md_interfaces__msg__MdRobotSlowStartStop__init(&msg->slow_sd)) {
    md_interfaces__msg__MdRobotUserParam__fini(msg);
    return false;
  }
  // max_linear_speed_usr
  // max_angular_speed_usr
  // brake_type
  return true;
}

void
md_interfaces__msg__MdRobotUserParam__fini(md_interfaces__msg__MdRobotUserParam * msg)
{
  if (!msg) {
    return;
  }
  // param_bit_select
  // slow_sd
  md_interfaces__msg__MdRobotSlowStartStop__fini(&msg->slow_sd);
  // max_linear_speed_usr
  // max_angular_speed_usr
  // brake_type
}

bool
md_interfaces__msg__MdRobotUserParam__are_equal(const md_interfaces__msg__MdRobotUserParam * lhs, const md_interfaces__msg__MdRobotUserParam * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // param_bit_select
  if (lhs->param_bit_select != rhs->param_bit_select) {
    return false;
  }
  // slow_sd
  if (!md_interfaces__msg__MdRobotSlowStartStop__are_equal(
      &(lhs->slow_sd), &(rhs->slow_sd)))
  {
    return false;
  }
  // max_linear_speed_usr
  if (lhs->max_linear_speed_usr != rhs->max_linear_speed_usr) {
    return false;
  }
  // max_angular_speed_usr
  if (lhs->max_angular_speed_usr != rhs->max_angular_speed_usr) {
    return false;
  }
  // brake_type
  if (lhs->brake_type != rhs->brake_type) {
    return false;
  }
  return true;
}

bool
md_interfaces__msg__MdRobotUserParam__copy(
  const md_interfaces__msg__MdRobotUserParam * input,
  md_interfaces__msg__MdRobotUserParam * output)
{
  if (!input || !output) {
    return false;
  }
  // param_bit_select
  output->param_bit_select = input->param_bit_select;
  // slow_sd
  if (!md_interfaces__msg__MdRobotSlowStartStop__copy(
      &(input->slow_sd), &(output->slow_sd)))
  {
    return false;
  }
  // max_linear_speed_usr
  output->max_linear_speed_usr = input->max_linear_speed_usr;
  // max_angular_speed_usr
  output->max_angular_speed_usr = input->max_angular_speed_usr;
  // brake_type
  output->brake_type = input->brake_type;
  return true;
}

md_interfaces__msg__MdRobotUserParam *
md_interfaces__msg__MdRobotUserParam__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotUserParam * msg = (md_interfaces__msg__MdRobotUserParam *)allocator.allocate(sizeof(md_interfaces__msg__MdRobotUserParam), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(md_interfaces__msg__MdRobotUserParam));
  bool success = md_interfaces__msg__MdRobotUserParam__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
md_interfaces__msg__MdRobotUserParam__destroy(md_interfaces__msg__MdRobotUserParam * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    md_interfaces__msg__MdRobotUserParam__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
md_interfaces__msg__MdRobotUserParam__Sequence__init(md_interfaces__msg__MdRobotUserParam__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotUserParam * data = NULL;

  if (size) {
    data = (md_interfaces__msg__MdRobotUserParam *)allocator.zero_allocate(size, sizeof(md_interfaces__msg__MdRobotUserParam), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = md_interfaces__msg__MdRobotUserParam__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        md_interfaces__msg__MdRobotUserParam__fini(&data[i - 1]);
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
md_interfaces__msg__MdRobotUserParam__Sequence__fini(md_interfaces__msg__MdRobotUserParam__Sequence * array)
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
      md_interfaces__msg__MdRobotUserParam__fini(&array->data[i]);
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

md_interfaces__msg__MdRobotUserParam__Sequence *
md_interfaces__msg__MdRobotUserParam__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotUserParam__Sequence * array = (md_interfaces__msg__MdRobotUserParam__Sequence *)allocator.allocate(sizeof(md_interfaces__msg__MdRobotUserParam__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = md_interfaces__msg__MdRobotUserParam__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
md_interfaces__msg__MdRobotUserParam__Sequence__destroy(md_interfaces__msg__MdRobotUserParam__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    md_interfaces__msg__MdRobotUserParam__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
md_interfaces__msg__MdRobotUserParam__Sequence__are_equal(const md_interfaces__msg__MdRobotUserParam__Sequence * lhs, const md_interfaces__msg__MdRobotUserParam__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!md_interfaces__msg__MdRobotUserParam__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
md_interfaces__msg__MdRobotUserParam__Sequence__copy(
  const md_interfaces__msg__MdRobotUserParam__Sequence * input,
  md_interfaces__msg__MdRobotUserParam__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(md_interfaces__msg__MdRobotUserParam);
    md_interfaces__msg__MdRobotUserParam * data =
      (md_interfaces__msg__MdRobotUserParam *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!md_interfaces__msg__MdRobotUserParam__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          md_interfaces__msg__MdRobotUserParam__fini(&data[i]);
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
    if (!md_interfaces__msg__MdRobotUserParam__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
