// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from md_interfaces:msg/Pose.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
md_interfaces__msg__Pose__init(md_interfaces__msg__Pose * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // theta
  // linear_velocity
  // angular_velocity
  // us_1
  // us_2
  // us_3
  // us_4
  // platform_state
  // left_motor_state
  // right_motor_state
  return true;
}

void
md_interfaces__msg__Pose__fini(md_interfaces__msg__Pose * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // theta
  // linear_velocity
  // angular_velocity
  // us_1
  // us_2
  // us_3
  // us_4
  // platform_state
  // left_motor_state
  // right_motor_state
}

bool
md_interfaces__msg__Pose__are_equal(const md_interfaces__msg__Pose * lhs, const md_interfaces__msg__Pose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  // us_1
  if (lhs->us_1 != rhs->us_1) {
    return false;
  }
  // us_2
  if (lhs->us_2 != rhs->us_2) {
    return false;
  }
  // us_3
  if (lhs->us_3 != rhs->us_3) {
    return false;
  }
  // us_4
  if (lhs->us_4 != rhs->us_4) {
    return false;
  }
  // platform_state
  if (lhs->platform_state != rhs->platform_state) {
    return false;
  }
  // left_motor_state
  if (lhs->left_motor_state != rhs->left_motor_state) {
    return false;
  }
  // right_motor_state
  if (lhs->right_motor_state != rhs->right_motor_state) {
    return false;
  }
  return true;
}

bool
md_interfaces__msg__Pose__copy(
  const md_interfaces__msg__Pose * input,
  md_interfaces__msg__Pose * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // theta
  output->theta = input->theta;
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  // us_1
  output->us_1 = input->us_1;
  // us_2
  output->us_2 = input->us_2;
  // us_3
  output->us_3 = input->us_3;
  // us_4
  output->us_4 = input->us_4;
  // platform_state
  output->platform_state = input->platform_state;
  // left_motor_state
  output->left_motor_state = input->left_motor_state;
  // right_motor_state
  output->right_motor_state = input->right_motor_state;
  return true;
}

md_interfaces__msg__Pose *
md_interfaces__msg__Pose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__Pose * msg = (md_interfaces__msg__Pose *)allocator.allocate(sizeof(md_interfaces__msg__Pose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(md_interfaces__msg__Pose));
  bool success = md_interfaces__msg__Pose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
md_interfaces__msg__Pose__destroy(md_interfaces__msg__Pose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    md_interfaces__msg__Pose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
md_interfaces__msg__Pose__Sequence__init(md_interfaces__msg__Pose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__Pose * data = NULL;

  if (size) {
    data = (md_interfaces__msg__Pose *)allocator.zero_allocate(size, sizeof(md_interfaces__msg__Pose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = md_interfaces__msg__Pose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        md_interfaces__msg__Pose__fini(&data[i - 1]);
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
md_interfaces__msg__Pose__Sequence__fini(md_interfaces__msg__Pose__Sequence * array)
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
      md_interfaces__msg__Pose__fini(&array->data[i]);
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

md_interfaces__msg__Pose__Sequence *
md_interfaces__msg__Pose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__Pose__Sequence * array = (md_interfaces__msg__Pose__Sequence *)allocator.allocate(sizeof(md_interfaces__msg__Pose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = md_interfaces__msg__Pose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
md_interfaces__msg__Pose__Sequence__destroy(md_interfaces__msg__Pose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    md_interfaces__msg__Pose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
md_interfaces__msg__Pose__Sequence__are_equal(const md_interfaces__msg__Pose__Sequence * lhs, const md_interfaces__msg__Pose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!md_interfaces__msg__Pose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
md_interfaces__msg__Pose__Sequence__copy(
  const md_interfaces__msg__Pose__Sequence * input,
  md_interfaces__msg__Pose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(md_interfaces__msg__Pose);
    md_interfaces__msg__Pose * data =
      (md_interfaces__msg__Pose *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!md_interfaces__msg__Pose__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          md_interfaces__msg__Pose__fini(&data[i]);
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
    if (!md_interfaces__msg__Pose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
