// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spin_conversion:msg/SpinCommand.idl
// generated code does not contain a copyright notice

#ifndef SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__TRAITS_HPP_
#define SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spin_conversion/msg/detail/spin_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace spin_conversion
{

namespace msg
{

inline void to_flow_style_yaml(
  const SpinCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: enable_spin
  {
    out << "enable_spin: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_spin, out);
    out << ", ";
  }

  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SpinCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: enable_spin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_spin: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_spin, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SpinCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace spin_conversion

namespace rosidl_generator_traits
{

[[deprecated("use spin_conversion::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const spin_conversion::msg::SpinCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  spin_conversion::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spin_conversion::msg::to_yaml() instead")]]
inline std::string to_yaml(const spin_conversion::msg::SpinCommand & msg)
{
  return spin_conversion::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spin_conversion::msg::SpinCommand>()
{
  return "spin_conversion::msg::SpinCommand";
}

template<>
inline const char * name<spin_conversion::msg::SpinCommand>()
{
  return "spin_conversion/msg/SpinCommand";
}

template<>
struct has_fixed_size<spin_conversion::msg::SpinCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<spin_conversion::msg::SpinCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<spin_conversion::msg::SpinCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__TRAITS_HPP_
