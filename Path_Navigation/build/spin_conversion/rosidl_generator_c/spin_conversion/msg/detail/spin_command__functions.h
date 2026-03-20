// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from spin_conversion:msg/SpinCommand.idl
// generated code does not contain a copyright notice

#ifndef SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__FUNCTIONS_H_
#define SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "spin_conversion/msg/rosidl_generator_c__visibility_control.h"

#include "spin_conversion/msg/detail/spin_command__struct.h"

/// Initialize msg/SpinCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spin_conversion__msg__SpinCommand
 * )) before or use
 * spin_conversion__msg__SpinCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
bool
spin_conversion__msg__SpinCommand__init(spin_conversion__msg__SpinCommand * msg);

/// Finalize msg/SpinCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
void
spin_conversion__msg__SpinCommand__fini(spin_conversion__msg__SpinCommand * msg);

/// Create msg/SpinCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spin_conversion__msg__SpinCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
spin_conversion__msg__SpinCommand *
spin_conversion__msg__SpinCommand__create();

/// Destroy msg/SpinCommand message.
/**
 * It calls
 * spin_conversion__msg__SpinCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
void
spin_conversion__msg__SpinCommand__destroy(spin_conversion__msg__SpinCommand * msg);

/// Check for msg/SpinCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
bool
spin_conversion__msg__SpinCommand__are_equal(const spin_conversion__msg__SpinCommand * lhs, const spin_conversion__msg__SpinCommand * rhs);

/// Copy a msg/SpinCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
bool
spin_conversion__msg__SpinCommand__copy(
  const spin_conversion__msg__SpinCommand * input,
  spin_conversion__msg__SpinCommand * output);

/// Initialize array of msg/SpinCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spin_conversion__msg__SpinCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
bool
spin_conversion__msg__SpinCommand__Sequence__init(spin_conversion__msg__SpinCommand__Sequence * array, size_t size);

/// Finalize array of msg/SpinCommand messages.
/**
 * It calls
 * spin_conversion__msg__SpinCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
void
spin_conversion__msg__SpinCommand__Sequence__fini(spin_conversion__msg__SpinCommand__Sequence * array);

/// Create array of msg/SpinCommand messages.
/**
 * It allocates the memory for the array and calls
 * spin_conversion__msg__SpinCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
spin_conversion__msg__SpinCommand__Sequence *
spin_conversion__msg__SpinCommand__Sequence__create(size_t size);

/// Destroy array of msg/SpinCommand messages.
/**
 * It calls
 * spin_conversion__msg__SpinCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
void
spin_conversion__msg__SpinCommand__Sequence__destroy(spin_conversion__msg__SpinCommand__Sequence * array);

/// Check for msg/SpinCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
bool
spin_conversion__msg__SpinCommand__Sequence__are_equal(const spin_conversion__msg__SpinCommand__Sequence * lhs, const spin_conversion__msg__SpinCommand__Sequence * rhs);

/// Copy an array of msg/SpinCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_spin_conversion
bool
spin_conversion__msg__SpinCommand__Sequence__copy(
  const spin_conversion__msg__SpinCommand__Sequence * input,
  spin_conversion__msg__SpinCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__FUNCTIONS_H_
