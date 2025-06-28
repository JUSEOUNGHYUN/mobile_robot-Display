// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from md_interfaces:msg/MdRobotSlowStartStop.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
md_interfaces__msg__MdRobotSlowStartStop__init(md_interfaces__msg__MdRobotSlowStartStop * msg)
{
  if (!msg) {
    return false;
  }
  // slowstart
  // slowstop
  return true;
}

void
md_interfaces__msg__MdRobotSlowStartStop__fini(md_interfaces__msg__MdRobotSlowStartStop * msg)
{
  if (!msg) {
    return;
  }
  // slowstart
  // slowstop
}

bool
md_interfaces__msg__MdRobotSlowStartStop__are_equal(const md_interfaces__msg__MdRobotSlowStartStop * lhs, const md_interfaces__msg__MdRobotSlowStartStop * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // slowstart
  if (lhs->slowstart != rhs->slowstart) {
    return false;
  }
  // slowstop
  if (lhs->slowstop != rhs->slowstop) {
    return false;
  }
  return true;
}

bool
md_interfaces__msg__MdRobotSlowStartStop__copy(
  const md_interfaces__msg__MdRobotSlowStartStop * input,
  md_interfaces__msg__MdRobotSlowStartStop * output)
{
  if (!input || !output) {
    return false;
  }
  // slowstart
  output->slowstart = input->slowstart;
  // slowstop
  output->slowstop = input->slowstop;
  return true;
}

md_interfaces__msg__MdRobotSlowStartStop *
md_interfaces__msg__MdRobotSlowStartStop__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotSlowStartStop * msg = (md_interfaces__msg__MdRobotSlowStartStop *)allocator.allocate(sizeof(md_interfaces__msg__MdRobotSlowStartStop), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(md_interfaces__msg__MdRobotSlowStartStop));
  bool success = md_interfaces__msg__MdRobotSlowStartStop__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
md_interfaces__msg__MdRobotSlowStartStop__destroy(md_interfaces__msg__MdRobotSlowStartStop * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    md_interfaces__msg__MdRobotSlowStartStop__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
md_interfaces__msg__MdRobotSlowStartStop__Sequence__init(md_interfaces__msg__MdRobotSlowStartStop__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotSlowStartStop * data = NULL;

  if (size) {
    data = (md_interfaces__msg__MdRobotSlowStartStop *)allocator.zero_allocate(size, sizeof(md_interfaces__msg__MdRobotSlowStartStop), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = md_interfaces__msg__MdRobotSlowStartStop__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        md_interfaces__msg__MdRobotSlowStartStop__fini(&data[i - 1]);
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
md_interfaces__msg__MdRobotSlowStartStop__Sequence__fini(md_interfaces__msg__MdRobotSlowStartStop__Sequence * array)
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
      md_interfaces__msg__MdRobotSlowStartStop__fini(&array->data[i]);
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

md_interfaces__msg__MdRobotSlowStartStop__Sequence *
md_interfaces__msg__MdRobotSlowStartStop__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotSlowStartStop__Sequence * array = (md_interfaces__msg__MdRobotSlowStartStop__Sequence *)allocator.allocate(sizeof(md_interfaces__msg__MdRobotSlowStartStop__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = md_interfaces__msg__MdRobotSlowStartStop__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
md_interfaces__msg__MdRobotSlowStartStop__Sequence__destroy(md_interfaces__msg__MdRobotSlowStartStop__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    md_interfaces__msg__MdRobotSlowStartStop__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
md_interfaces__msg__MdRobotSlowStartStop__Sequence__are_equal(const md_interfaces__msg__MdRobotSlowStartStop__Sequence * lhs, const md_interfaces__msg__MdRobotSlowStartStop__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!md_interfaces__msg__MdRobotSlowStartStop__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
md_interfaces__msg__MdRobotSlowStartStop__Sequence__copy(
  const md_interfaces__msg__MdRobotSlowStartStop__Sequence * input,
  md_interfaces__msg__MdRobotSlowStartStop__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(md_interfaces__msg__MdRobotSlowStartStop);
    md_interfaces__msg__MdRobotSlowStartStop * data =
      (md_interfaces__msg__MdRobotSlowStartStop *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!md_interfaces__msg__MdRobotSlowStartStop__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          md_interfaces__msg__MdRobotSlowStartStop__fini(&data[i]);
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
    if (!md_interfaces__msg__MdRobotSlowStartStop__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
