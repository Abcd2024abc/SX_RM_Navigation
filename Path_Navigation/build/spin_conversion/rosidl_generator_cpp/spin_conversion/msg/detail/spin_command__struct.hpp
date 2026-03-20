// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spin_conversion:msg/SpinCommand.idl
// generated code does not contain a copyright notice

#ifndef SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__STRUCT_HPP_
#define SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__spin_conversion__msg__SpinCommand __attribute__((deprecated))
#else
# define DEPRECATED__spin_conversion__msg__SpinCommand __declspec(deprecated)
#endif

namespace spin_conversion
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SpinCommand_
{
  using Type = SpinCommand_<ContainerAllocator>;

  explicit SpinCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable_spin = false;
      this->direction = 0;
      this->speed = 0.0f;
    }
  }

  explicit SpinCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable_spin = false;
      this->direction = 0;
      this->speed = 0.0f;
    }
  }

  // field types and members
  using _enable_spin_type =
    bool;
  _enable_spin_type enable_spin;
  using _direction_type =
    int8_t;
  _direction_type direction;
  using _speed_type =
    float;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__enable_spin(
    const bool & _arg)
  {
    this->enable_spin = _arg;
    return *this;
  }
  Type & set__direction(
    const int8_t & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spin_conversion::msg::SpinCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const spin_conversion::msg::SpinCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spin_conversion::msg::SpinCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spin_conversion::msg::SpinCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spin_conversion::msg::SpinCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spin_conversion::msg::SpinCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spin_conversion::msg::SpinCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spin_conversion::msg::SpinCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spin_conversion::msg::SpinCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spin_conversion::msg::SpinCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spin_conversion__msg__SpinCommand
    std::shared_ptr<spin_conversion::msg::SpinCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spin_conversion__msg__SpinCommand
    std::shared_ptr<spin_conversion::msg::SpinCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpinCommand_ & other) const
  {
    if (this->enable_spin != other.enable_spin) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpinCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpinCommand_

// alias to use template instance with default allocator
using SpinCommand =
  spin_conversion::msg::SpinCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace spin_conversion

#endif  // SPIN_CONVERSION__MSG__DETAIL__SPIN_COMMAND__STRUCT_HPP_
