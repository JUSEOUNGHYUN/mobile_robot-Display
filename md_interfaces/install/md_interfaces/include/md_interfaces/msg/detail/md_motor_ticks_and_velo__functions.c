// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from md_interfaces:msg/MdMotorTicksAndVelo.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_motor_ticks_and_velo__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
md_interfaces__msg__MdMotorTicksAndVelo__init(md_interfaces__msg__MdMotorTicksAndVelo * msg)
{
  if (!msg) {
    return false;
  }
  // left_ticks
  // left_rpm
  // right_ticks
  // right_rpm
  return true;
}

void
md_interfaces__msg__MdMotorTicksAndVelo__fini(md_interfaces__msg__MdMotorTicksAndVelo * msg)
{
  if (!msg) {
    return;
  }
  // left_ticks
  // left_rpm
  // right_ticks
  // right_rpm
}

bool
md_interfaces__msg__MdMotorTicksAndVelo__are_equal(const md_interfaces__msg__MdMotorTicksAndVelo * lhs, const md_interfaces__msg__MdMotorTicksAndVelo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // left_ticks
  if (lhs->left_ticks != rhs->left_ticks) {
    return false;
  }
  // left_rpm
  if (lhs->left_rpm != rhs->left_rpm) {
    return false;
  }
  // right_ticks
  if (lhs->right_ticks != rhs->right_ticks) {
    return false;
  }
  // right_rpm
  if (lhs->right_rpm != rhs->right_rpm) {
    return false;
  }
  return true;
}

bool
md_interfaces__msg__MdMotorTicksAndVelo__copy(
  const md_interfaces__msg__MdMotorTicksAndVelo * input,
  md_interfaces__msg__MdMotorTicksAndVelo * output)
{
  if (!input || !output) {
    return false;
  }
  // left_ticks
  output->left_ticks = input->left_ticks;
  // left_rpm
  output->left_rpm = input->left_rpm;
  // right_ticks
  output->right_ticks = input->right_ticks;
  // right_rpm
  output->right_rpm = input->right_rpm;
  return true;
}

md_interfaces__msg__MdMotorTicksAndVelo *
md_interfaces__msg__MdMotorTicksAndVelo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdMotorTicksAndVelo * msg = (md_interfaces__msg__MdMotorTicksAndVelo *)allocator.allocate(sizeof(md_interfaces__msg__MdMotorTicksAndVelo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(md_interfaces__msg__MdMotorTicksAndVelo));
  bool success = md_interfaces__msg__MdMotorTicksAndVelo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
md_interfaces__msg__MdMotorTicksAndVelo__destroy(md_interfaces__msg__MdMotorTicksAndVelo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    md_interfaces__msg__MdMotorTicksAndVelo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
md_interfaces__msg__MdMotorTicksAndVelo__Sequence__init(md_interfaces__msg__MdMotorTicksAndVelo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdMotorTicksAndVelo * data = NULL;

  if (size) {
    data = (md_interfaces__msg__MdMotorTicksAndVelo *)allocator.zero_allocate(size, sizeof(md_interfaces__msg__MdMotorTicksAndVelo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = md_interfaces__msg__MdMotorTicksAndVelo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        md_interfaces__msg__MdMotorTicksAndVelo__fini(&data[i - 1]);
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
md_interfaces__msg__MdMotorTicksAndVelo__Sequence__fini(md_interfaces__msg__MdMotorTicksAndVelo__Sequence * array)
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
      md_interfaces__msg__MdMotorTicksAndVelo__fini(&array->data[i]);
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

md_interfaces__msg__MdMotorTicksAndVelo__Sequence *
md_interfaces__msg__MdMotorTicksAndVelo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdMotorTicksAndVelo__Sequence * array = (md_interfaces__msg__MdMotorTicksAndVelo__Sequence *)allocator.allocate(sizeof(md_interfaces__msg__MdMotorTicksAndVelo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = md_interfaces__msg__MdMotorTicksAndVelo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
md_interfaces__msg__MdMotorTicksAndVelo__Sequence__destroy(md_interfaces__msg__MdMotorTicksAndVelo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    md_interfaces__msg__MdMotorTicksAndVelo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
md_interfaces__msg__MdMotorTicksAndVelo__Sequence__are_equal(const md_interfaces__msg__MdMotorTicksAndVelo__Sequence * lhs, const md_interfaces__msg__MdMotorTicksAndVelo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!md_interfaces__msg__MdMotorTicksAndVelo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
md_interfaces__msg__MdMotorTicksAndVelo__Sequence__copy(
  const md_interfaces__msg__MdMotorTicksAndVelo__Sequence * input,
  md_interfaces__msg__MdMotorTicksAndVelo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(md_interfaces__msg__MdMotorTicksAndVelo);
    md_interfaces__msg__MdMotorTicksAndVelo * data =
      (md_interfaces__msg__MdMotorTicksAndVelo *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!md_interfaces__msg__MdMotorTicksAndVelo__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          md_interfaces__msg__MdMotorTicksAndVelo__fini(&data[i]);
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
    if (!md_interfaces__msg__MdMotorTicksAndVelo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
