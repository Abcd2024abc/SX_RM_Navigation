// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from spin_conversion:msg/SpinCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "spin_conversion/msg/detail/spin_command__rosidl_typesupport_introspection_c.h"
#include "spin_conversion/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "spin_conversion/msg/detail/spin_command__functions.h"
#include "spin_conversion/msg/detail/spin_command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  spin_conversion__msg__SpinCommand__init(message_memory);
}

void spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_fini_function(void * message_memory)
{
  spin_conversion__msg__SpinCommand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_message_member_array[3] = {
  {
    "enable_spin",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spin_conversion__msg__SpinCommand, enable_spin),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "direction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spin_conversion__msg__SpinCommand, direction),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spin_conversion__msg__SpinCommand, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_message_members = {
  "spin_conversion__msg",  // message namespace
  "SpinCommand",  // message name
  3,  // number of fields
  sizeof(spin_conversion__msg__SpinCommand),
  spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_message_member_array,  // message members
  spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_message_type_support_handle = {
  0,
  &spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_spin_conversion
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spin_conversion, msg, SpinCommand)() {
  if (!spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_message_type_support_handle.typesupport_identifier) {
    spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &spin_conversion__msg__SpinCommand__rosidl_typesupport_introspection_c__SpinCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
