// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from md_interfaces:msg/PosVelTimestamped.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/pos_vel_timestamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
md_interfaces__msg__PosVelTimestamped__init(md_interfaces__msg__PosVelTimestamped * msg)
{
  if (!msg) {
    return false;
  }
  // motor_pos1
  // motor_vel1
  // motor_pos2
  // motor_vel2
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    md_interfaces__msg__PosVelTimestamped__fini(msg);
    return false;
  }
  return true;
}

void
md_interfaces__msg__PosVelTimestamped__fini(md_interfaces__msg__PosVelTimestamped * msg)
{
  if (!msg) {
    return;
  }
  // motor_pos1
  // motor_vel1
  // motor_pos2
  // motor_vel2
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
md_interfaces__msg__PosVelTimestamped__are_equal(const md_interfaces__msg__PosVelTimestamped * lhs, const md_interfaces__msg__PosVelTimestamped * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor_pos1
  if (lhs->motor_pos1 != rhs->motor_pos1) {
    return false;
  }
  // motor_vel1
  if (lhs->motor_vel1 != rhs->motor_vel1) {
    return false;
  }
  // motor_pos2
  if (lhs->motor_pos2 != rhs->motor_pos2) {
    return false;
  }
  // motor_vel2
  if (lhs->motor_vel2 != rhs->motor_vel2) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
md_interfaces__msg__PosVelTimestamped__copy(
  const md_interfaces__msg__PosVelTimestamped * input,
  md_interfaces__msg__PosVelTimestamped * output)
{
  if (!input || !output) {
    return false;
  }
  // motor_pos1
  output->motor_pos1 = input->motor_pos1;
  // motor_vel1
  output->motor_vel1 = input->motor_vel1;
  // motor_pos2
  output->motor_pos2 = input->motor_pos2;
  // motor_vel2
  output->motor_vel2 = input->motor_vel2;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

md_interfaces__msg__PosVelTimestamped *
md_interfaces__msg__PosVelTimestamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__PosVelTimestamped * msg = (md_interfaces__msg__PosVelTimestamped *)allocator.allocate(sizeof(md_interfaces__msg__PosVelTimestamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(md_interfaces__msg__PosVelTimestamped));
  bool success = md_interfaces__msg__PosVelTimestamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
md_interfaces__msg__PosVelTimestamped__destroy(md_interfaces__msg__PosVelTimestamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    md_interfaces__msg__PosVelTimestamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
md_interfaces__msg__PosVelTimestamped__Sequence__init(md_interfaces__msg__PosVelTimestamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__PosVelTimestamped * data = NULL;

  if (size) {
    data = (md_interfaces__msg__PosVelTimestamped *)allocator.zero_allocate(size, sizeof(md_interfaces__msg__PosVelTimestamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = md_interfaces__msg__PosVelTimestamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        md_interfaces__msg__PosVelTimestamped__fini(&data[i - 1]);
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
md_interfaces__msg__PosVelTimestamped__Sequence__fini(md_interfaces__msg__PosVelTimestamped__Sequence * array)
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
      md_interfaces__msg__PosVelTimestamped__fini(&array->data[i]);
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

md_interfaces__msg__PosVelTimestamped__Sequence *
md_interfaces__msg__PosVelTimestamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__PosVelTimestamped__Sequence * array = (md_interfaces__msg__PosVelTimestamped__Sequence *)allocator.allocate(sizeof(md_interfaces__msg__PosVelTimestamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = md_interfaces__msg__PosVelTimestamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
md_interfaces__msg__PosVelTimestamped__Sequence__destroy(md_interfaces__msg__PosVelTimestamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    md_interfaces__msg__PosVelTimestamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
md_interfaces__msg__PosVelTimestamped__Sequence__are_equal(const md_interfaces__msg__PosVelTimestamped__Sequence * lhs, const md_interfaces__msg__PosVelTimestamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!md_interfaces__msg__PosVelTimestamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
md_interfaces__msg__PosVelTimestamped__Sequence__copy(
  const md_interfaces__msg__PosVelTimestamped__Sequence * input,
  md_interfaces__msg__PosVelTimestamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(md_interfaces__msg__PosVelTimestamped);
    md_interfaces__msg__PosVelTimestamped * data =
      (md_interfaces__msg__PosVelTimestamped *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!md_interfaces__msg__PosVelTimestamped__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          md_interfaces__msg__PosVelTimestamped__fini(&data[i]);
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
    if (!md_interfaces__msg__PosVelTimestamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
