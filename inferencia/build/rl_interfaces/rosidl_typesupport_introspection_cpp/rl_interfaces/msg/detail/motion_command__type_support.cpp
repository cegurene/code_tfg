// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rl_interfaces:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rl_interfaces/msg/detail/motion_command__functions.h"
#include "rl_interfaces/msg/detail/motion_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rl_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotionCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rl_interfaces::msg::MotionCommand(_init);
}

void MotionCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rl_interfaces::msg::MotionCommand *>(message_memory);
  typed_message->~MotionCommand();
}

size_t size_function__MotionCommand__drive_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__drive_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__drive_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotionCommand__drive_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint32_t *>(
    get_const_function__MotionCommand__drive_ids(untyped_member, index));
  auto & value = *reinterpret_cast<uint32_t *>(untyped_value);
  value = item;
}

void assign_function__MotionCommand__drive_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint32_t *>(
    get_function__MotionCommand__drive_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint32_t *>(untyped_value);
  item = value;
}

void resize_function__MotionCommand__drive_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__target_position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__target_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__target_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotionCommand__target_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__MotionCommand__target_position(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__MotionCommand__target_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__MotionCommand__target_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__MotionCommand__target_position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__target_velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__target_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__target_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotionCommand__target_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__MotionCommand__target_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__MotionCommand__target_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__MotionCommand__target_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__MotionCommand__target_velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__target_torque(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__target_torque(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__target_torque(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotionCommand__target_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__MotionCommand__target_torque(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__MotionCommand__target_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__MotionCommand__target_torque(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__MotionCommand__target_torque(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotionCommand_message_member_array[4] = {
  {
    "drive_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rl_interfaces::msg::MotionCommand, drive_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__drive_ids,  // size() function pointer
    get_const_function__MotionCommand__drive_ids,  // get_const(index) function pointer
    get_function__MotionCommand__drive_ids,  // get(index) function pointer
    fetch_function__MotionCommand__drive_ids,  // fetch(index, &value) function pointer
    assign_function__MotionCommand__drive_ids,  // assign(index, value) function pointer
    resize_function__MotionCommand__drive_ids  // resize(index) function pointer
  },
  {
    "target_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rl_interfaces::msg::MotionCommand, target_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__target_position,  // size() function pointer
    get_const_function__MotionCommand__target_position,  // get_const(index) function pointer
    get_function__MotionCommand__target_position,  // get(index) function pointer
    fetch_function__MotionCommand__target_position,  // fetch(index, &value) function pointer
    assign_function__MotionCommand__target_position,  // assign(index, value) function pointer
    resize_function__MotionCommand__target_position  // resize(index) function pointer
  },
  {
    "target_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rl_interfaces::msg::MotionCommand, target_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__target_velocity,  // size() function pointer
    get_const_function__MotionCommand__target_velocity,  // get_const(index) function pointer
    get_function__MotionCommand__target_velocity,  // get(index) function pointer
    fetch_function__MotionCommand__target_velocity,  // fetch(index, &value) function pointer
    assign_function__MotionCommand__target_velocity,  // assign(index, value) function pointer
    resize_function__MotionCommand__target_velocity  // resize(index) function pointer
  },
  {
    "target_torque",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rl_interfaces::msg::MotionCommand, target_torque),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__target_torque,  // size() function pointer
    get_const_function__MotionCommand__target_torque,  // get_const(index) function pointer
    get_function__MotionCommand__target_torque,  // get(index) function pointer
    fetch_function__MotionCommand__target_torque,  // fetch(index, &value) function pointer
    assign_function__MotionCommand__target_torque,  // assign(index, value) function pointer
    resize_function__MotionCommand__target_torque  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotionCommand_message_members = {
  "rl_interfaces::msg",  // message namespace
  "MotionCommand",  // message name
  4,  // number of fields
  sizeof(rl_interfaces::msg::MotionCommand),
  false,  // has_any_key_member_
  MotionCommand_message_member_array,  // message members
  MotionCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  MotionCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotionCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotionCommand_message_members,
  get_message_typesupport_handle_function,
  &rl_interfaces__msg__MotionCommand__get_type_hash,
  &rl_interfaces__msg__MotionCommand__get_type_description,
  &rl_interfaces__msg__MotionCommand__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rl_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rl_interfaces::msg::MotionCommand>()
{
  return &::rl_interfaces::msg::rosidl_typesupport_introspection_cpp::MotionCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rl_interfaces, msg, MotionCommand)() {
  return &::rl_interfaces::msg::rosidl_typesupport_introspection_cpp::MotionCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
