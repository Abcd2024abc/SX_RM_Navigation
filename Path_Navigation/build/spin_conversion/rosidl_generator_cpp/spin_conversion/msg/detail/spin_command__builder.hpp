// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spin_conversion:msg/SpinCommand.idl
// generated code does not contain a copyright notice

#ifndef SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__BUILDER_HPP_
#define SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spin_conversion/msg/detail/spin_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spin_conversion
{

namespace msg
{

namespace builder
{

class Init_SpinCommand_speed
{
public:
  explicit Init_SpinCommand_speed(::spin_conversion::msg::SpinCommand & msg)
  : msg_(msg)
  {}
  ::spin_conversion::msg::SpinCommand speed(::spin_conversion::msg::SpinCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spin_conversion::msg::SpinCommand msg_;
};

class Init_SpinCommand_direction
{
public:
  explicit Init_SpinCommand_direction(::spin_conversion::msg::SpinCommand & msg)
  : msg_(msg)
  {}
  Init_SpinCommand_speed direction(::spin_conversion::msg::SpinCommand::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_SpinCommand_speed(msg_);
  }

private:
  ::spin_conversion::msg::SpinCommand msg_;
};

class Init_SpinCommand_enable_spin
{
public:
  Init_SpinCommand_enable_spin()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SpinCommand_direction enable_spin(::spin_conversion::msg::SpinCommand::_enable_spin_type arg)
  {
    msg_.enable_spin = std::move(arg);
    return Init_SpinCommand_direction(msg_);
  }

private:
  ::spin_conversion::msg::SpinCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spin_conversion::msg::SpinCommand>()
{
  return spin_conversion::msg::builder::Init_SpinCommand_enable_spin();
}

}  // namespace spin_conversion

#endif  // SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__BUILDER_HPP_
