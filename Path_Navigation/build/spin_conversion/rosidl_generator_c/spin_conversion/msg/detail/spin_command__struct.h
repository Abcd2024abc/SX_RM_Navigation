// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spin_conversion:msg/SpinCommand.idl
// generated code does not contain a copyright notice

#ifndef SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__STRUCT_H_
#define SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SpinCommand in the package spin_conversion.
typedef struct spin_conversion__msg__SpinCommand
{
  /// 是否启用自旋
  bool enable_spin;
  /// 自旋方向: 1=顺时针, -1=逆时针
  int8_t direction;
  /// 自旋速度 (rad/s)
  float speed;
} spin_conversion__msg__SpinCommand;

// Struct for a sequence of spin_conversion__msg__SpinCommand.
typedef struct spin_conversion__msg__SpinCommand__Sequence
{
  spin_conversion__msg__SpinCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spin_conversion__msg__SpinCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__STRUCT_H_
