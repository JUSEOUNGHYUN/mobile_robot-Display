// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from md_interfaces:msg/MdRobotMsg2.idl
// generated code does not contain a copyright notice
#include "md_interfaces/msg/detail/md_robot_msg2__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
md_interfaces__msg__MdRobotMsg2__init(md_interfaces__msg__MdRobotMsg2 * msg)
{
  if (!msg) {
    return false;
  }
  // interval_time
  // x_pos
  // y_pos
  // angule
  // us_1
  // us_2
  // us_3
  // us_4
  // platform_state
  // linear_velocity
  // angular_velocity
  // input_voltage
  return true;
}

void
md_interfaces__msg__MdRobotMsg2__fini(md_interfaces__msg__MdRobotMsg2 * msg)
{
  if (!msg) {
    return;
  }
  // interval_time
  // x_pos
  // y_pos
  // angule
  // us_1
  // us_2
  // us_3
  // us_4
  // platform_state
  // linear_velocity
  // angular_velocity
  // input_voltage
}

bool
md_interfaces__msg__MdRobotMsg2__are_equal(const md_interfaces__msg__MdRobotMsg2 * lhs, const md_interfaces__msg__MdRobotMsg2 * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // interval_time
  if (lhs->interval_time != rhs->interval_time) {
    return false;
  }
  // x_pos
  if (lhs->x_pos != rhs->x_pos) {
    return false;
  }
  // y_pos
  if (lhs->y_pos != rhs->y_pos) {
    return false;
  }
  // angule
  if (lhs->angule != rhs->angule) {
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
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  // input_voltage
  if (lhs->input_voltage != rhs->input_voltage) {
    return false;
  }
  return true;
}

bool
md_interfaces__msg__MdRobotMsg2__copy(
  const md_interfaces__msg__MdRobotMsg2 * input,
  md_interfaces__msg__MdRobotMsg2 * output)
{
  if (!input || !output) {
    return false;
  }
  // interval_time
  output->interval_time = input->interval_time;
  // x_pos
  output->x_pos = input->x_pos;
  // y_pos
  output->y_pos = input->y_pos;
  // angule
  output->angule = input->angule;
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
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  // input_voltage
  output->input_voltage = input->input_voltage;
  return true;
}

md_interfaces__msg__MdRobotMsg2 *
md_interfaces__msg__MdRobotMsg2__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotMsg2 * msg = (md_interfaces__msg__MdRobotMsg2 *)allocator.allocate(sizeof(md_interfaces__msg__MdRobotMsg2), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(md_interfaces__msg__MdRobotMsg2));
  bool success = md_interfaces__msg__MdRobotMsg2__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
md_interfaces__msg__MdRobotMsg2__destroy(md_interfaces__msg__MdRobotMsg2 * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    md_interfaces__msg__MdRobotMsg2__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
md_interfaces__msg__MdRobotMsg2__Sequence__init(md_interfaces__msg__MdRobotMsg2__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotMsg2 * data = NULL;

  if (size) {
    data = (md_interfaces__msg__MdRobotMsg2 *)allocator.zero_allocate(size, sizeof(md_interfaces__msg__MdRobotMsg2), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = md_interfaces__msg__MdRobotMsg2__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        md_interfaces__msg__MdRobotMsg2__fini(&data[i - 1]);
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
md_interfaces__msg__MdRobotMsg2__Sequence__fini(md_interfaces__msg__MdRobotMsg2__Sequence * array)
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
      md_interfaces__msg__MdRobotMsg2__fini(&array->data[i]);
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

md_interfaces__msg__MdRobotMsg2__Sequence *
md_interfaces__msg__MdRobotMsg2__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  md_interfaces__msg__MdRobotMsg2__Sequence * array = (md_interfaces__msg__MdRobotMsg2__Sequence *)allocator.allocate(sizeof(md_interfaces__msg__MdRobotMsg2__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = md_interfaces__msg__MdRobotMsg2__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
md_interfaces__msg__MdRobotMsg2__Sequence__destroy(md_interfaces__msg__MdRobotMsg2__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    md_interfaces__msg__MdRobotMsg2__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
md_interfaces__msg__MdRobotMsg2__Sequence__are_equal(const md_interfaces__msg__MdRobotMsg2__Sequence * lhs, const md_interfaces__msg__MdRobotMsg2__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!md_interfaces__msg__MdRobotMsg2__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
md_interfaces__msg__MdRobotMsg2__Sequence__copy(
  const md_interfaces__msg__MdRobotMsg2__Sequence * input,
  md_interfaces__msg__MdRobotMsg2__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(md_interfaces__msg__MdRobotMsg2);
    md_interfaces__msg__MdRobotMsg2 * data =
      (md_interfaces__msg__MdRobotMsg2 *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!md_interfaces__msg__MdRobotMsg2__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          md_interfaces__msg__MdRobotMsg2__fini(&data[i]);
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
    if (!md_interfaces__msg__MdRobotMsg2__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
