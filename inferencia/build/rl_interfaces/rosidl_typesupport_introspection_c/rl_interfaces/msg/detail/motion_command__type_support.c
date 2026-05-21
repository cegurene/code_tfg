// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rl_interfaces:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rl_interfaces/msg/detail/motion_command__rosidl_typesupport_introspection_c.h"
#include "rl_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rl_interfaces/msg/detail/motion_command__functions.h"
#include "rl_interfaces/msg/detail/motion_command__struct.h"


// Include directives for member types
// Member `drive_ids`
// Member `target_position`
// Member `target_velocity`
// Member `target_torque`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rl_interfaces__msg__MotionCommand__init(message_memory);
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_fini_function(void * message_memory)
{
  rl_interfaces__msg__MotionCommand__fini(message_memory);
}

size_t rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__size_function__MotionCommand__drive_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__drive_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__drive_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__fetch_function__MotionCommand__drive_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__drive_ids(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__assign_function__MotionCommand__drive_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__drive_ids(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__resize_function__MotionCommand__drive_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

size_t rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__size_function__MotionCommand__target_position(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__fetch_function__MotionCommand__target_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__assign_function__MotionCommand__target_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__resize_function__MotionCommand__target_position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__size_function__MotionCommand__target_velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__fetch_function__MotionCommand__target_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_velocity(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__assign_function__MotionCommand__target_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_velocity(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__resize_function__MotionCommand__target_velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__size_function__MotionCommand__target_torque(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_torque(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_torque(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__fetch_function__MotionCommand__target_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_torque(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__assign_function__MotionCommand__target_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_torque(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__resize_function__MotionCommand__target_torque(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[4] = {
  {
    "drive_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rl_interfaces__msg__MotionCommand, drive_ids),  // bytes offset in struct
    NULL,  // default value
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__size_function__MotionCommand__drive_ids,  // size() function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__drive_ids,  // get_const(index) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__drive_ids,  // get(index) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__fetch_function__MotionCommand__drive_ids,  // fetch(index, &value) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__assign_function__MotionCommand__drive_ids,  // assign(index, value) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__resize_function__MotionCommand__drive_ids  // resize(index) function pointer
  },
  {
    "target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rl_interfaces__msg__MotionCommand, target_position),  // bytes offset in struct
    NULL,  // default value
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__size_function__MotionCommand__target_position,  // size() function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_position,  // get_const(index) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_position,  // get(index) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__fetch_function__MotionCommand__target_position,  // fetch(index, &value) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__assign_function__MotionCommand__target_position,  // assign(index, value) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__resize_function__MotionCommand__target_position  // resize(index) function pointer
  },
  {
    "target_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rl_interfaces__msg__MotionCommand, target_velocity),  // bytes offset in struct
    NULL,  // default value
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__size_function__MotionCommand__target_velocity,  // size() function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_velocity,  // get_const(index) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_velocity,  // get(index) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__fetch_function__MotionCommand__target_velocity,  // fetch(index, &value) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__assign_function__MotionCommand__target_velocity,  // assign(index, value) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__resize_function__MotionCommand__target_velocity  // resize(index) function pointer
  },
  {
    "target_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rl_interfaces__msg__MotionCommand, target_torque),  // bytes offset in struct
    NULL,  // default value
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__size_function__MotionCommand__target_torque,  // size() function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_const_function__MotionCommand__target_torque,  // get_const(index) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__get_function__MotionCommand__target_torque,  // get(index) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__fetch_function__MotionCommand__target_torque,  // fetch(index, &value) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__assign_function__MotionCommand__target_torque,  // assign(index, value) function pointer
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__resize_function__MotionCommand__target_torque  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_members = {
  "rl_interfaces__msg",  // message namespace
  "MotionCommand",  // message name
  4,  // number of fields
  sizeof(rl_interfaces__msg__MotionCommand),
  false,  // has_any_key_member_
  rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array,  // message members
  rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_type_support_handle = {
  0,
  &rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_members,
  get_message_typesupport_handle_function,
  &rl_interfaces__msg__MotionCommand__get_type_hash,
  &rl_interfaces__msg__MotionCommand__get_type_description,
  &rl_interfaces__msg__MotionCommand__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rl_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rl_interfaces, msg, MotionCommand)() {
  if (!rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_type_support_handle.typesupport_identifier) {
    rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rl_interfaces__msg__MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
