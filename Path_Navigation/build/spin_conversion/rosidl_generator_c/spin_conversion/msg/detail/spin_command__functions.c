// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from spin_conversion:msg/SpinCommand.idl
// generated code does not contain a copyright notice
#include "spin_conversion/msg/detail/spin_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
spin_conversion__msg__SpinCommand__init(spin_conversion__msg__SpinCommand * msg)
{
  if (!msg) {
    return false;
  }
  // enable_spin
  // direction
  // speed
  return true;
}

void
spin_conversion__msg__SpinCommand__fini(spin_conversion__msg__SpinCommand * msg)
{
  if (!msg) {
    return;
  }
  // enable_spin
  // direction
  // speed
}

bool
spin_conversion__msg__SpinCommand__are_equal(const spin_conversion__msg__SpinCommand * lhs, const spin_conversion__msg__SpinCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // enable_spin
  if (lhs->enable_spin != rhs->enable_spin) {
    return false;
  }
  // direction
  if (lhs->direction != rhs->direction) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  return true;
}

bool
spin_conversion__msg__SpinCommand__copy(
  const spin_conversion__msg__SpinCommand * input,
  spin_conversion__msg__SpinCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // enable_spin
  output->enable_spin = input->enable_spin;
  // direction
  output->direction = input->direction;
  // speed
  output->speed = input->speed;
  return true;
}

spin_conversion__msg__SpinCommand *
spin_conversion__msg__SpinCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spin_conversion__msg__SpinCommand * msg = (spin_conversion__msg__SpinCommand *)allocator.allocate(sizeof(spin_conversion__msg__SpinCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spin_conversion__msg__SpinCommand));
  bool success = spin_conversion__msg__SpinCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spin_conversion__msg__SpinCommand__destroy(spin_conversion__msg__SpinCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spin_conversion__msg__SpinCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spin_conversion__msg__SpinCommand__Sequence__init(spin_conversion__msg__SpinCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spin_conversion__msg__SpinCommand * data = NULL;

  if (size) {
    data = (spin_conversion__msg__SpinCommand *)allocator.zero_allocate(size, sizeof(spin_conversion__msg__SpinCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spin_conversion__msg__SpinCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spin_conversion__msg__SpinCommand__fini(&data[i - 1]);
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
spin_conversion__msg__SpinCommand__Sequence__fini(spin_conversion__msg__SpinCommand__Sequence * array)
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
      spin_conversion__msg__SpinCommand__fini(&array->data[i]);
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

spin_conversion__msg__SpinCommand__Sequence *
spin_conversion__msg__SpinCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spin_conversion__msg__SpinCommand__Sequence * array = (spin_conversion__msg__SpinCommand__Sequence *)allocator.allocate(sizeof(spin_conversion__msg__SpinCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spin_conversion__msg__SpinCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spin_conversion__msg__SpinCommand__Sequence__destroy(spin_conversion__msg__SpinCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spin_conversion__msg__SpinCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spin_conversion__msg__SpinCommand__Sequence__are_equal(const spin_conversion__msg__SpinCommand__Sequence * lhs, const spin_conversion__msg__SpinCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spin_conversion__msg__SpinCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spin_conversion__msg__SpinCommand__Sequence__copy(
  const spin_conversion__msg__SpinCommand__Sequence * input,
  spin_conversion__msg__SpinCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spin_conversion__msg__SpinCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spin_conversion__msg__SpinCommand * data =
      (spin_conversion__msg__SpinCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spin_conversion__msg__SpinCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spin_conversion__msg__SpinCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spin_conversion__msg__SpinCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
