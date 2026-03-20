// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from spin_conversion:msg/SpinCommand.idl
// generated code does not contain a copyright notice

#ifndef SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "spin_conversion/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "spin_conversion/msg/detail/spin_command__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace spin_conversion
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spin_conversion
cdr_serialize(
  const spin_conversion::msg::SpinCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spin_conversion
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  spin_conversion::msg::SpinCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spin_conversion
get_serialized_size(
  const spin_conversion::msg::SpinCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spin_conversion
max_serialized_size_SpinCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace spin_conversion

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spin_conversion
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spin_conversion, msg, SpinCommand)();

#ifdef __cplusplus
}
#endif

#endif  // SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
