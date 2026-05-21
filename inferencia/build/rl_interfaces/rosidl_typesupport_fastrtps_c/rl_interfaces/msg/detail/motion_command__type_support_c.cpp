// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rl_interfaces:msg/MotionCommand.idl
// generated code does not contain a copyright notice
#include "rl_interfaces/msg/detail/motion_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rl_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rl_interfaces/msg/detail/motion_command__struct.h"
#include "rl_interfaces/msg/detail/motion_command__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // drive_ids, target_position, target_torque, target_velocity
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // drive_ids, target_position, target_torque, target_velocity

// forward declare type support functions


using _MotionCommand__ros_msg_type = rl_interfaces__msg__MotionCommand;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rl_interfaces
bool cdr_serialize_rl_interfaces__msg__MotionCommand(
  const rl_interfaces__msg__MotionCommand * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: drive_ids
  {
    size_t size = ros_message->drive_ids.size;
    auto array_ptr = ros_message->drive_ids.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: target_position
  {
    size_t size = ros_message->target_position.size;
    auto array_ptr = ros_message->target_position.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: target_velocity
  {
    size_t size = ros_message->target_velocity.size;
    auto array_ptr = ros_message->target_velocity.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: target_torque
  {
    size_t size = ros_message->target_torque.size;
    auto array_ptr = ros_message->target_torque.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rl_interfaces
bool cdr_deserialize_rl_interfaces__msg__MotionCommand(
  eprosima::fastcdr::Cdr & cdr,
  rl_interfaces__msg__MotionCommand * ros_message)
{
  // Field name: drive_ids
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->drive_ids.data) {
      rosidl_runtime_c__uint32__Sequence__fini(&ros_message->drive_ids);
    }
    if (!rosidl_runtime_c__uint32__Sequence__init(&ros_message->drive_ids, size)) {
      fprintf(stderr, "failed to create array for field 'drive_ids'");
      return false;
    }
    auto array_ptr = ros_message->drive_ids.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: target_position
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->target_position.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->target_position);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->target_position, size)) {
      fprintf(stderr, "failed to create array for field 'target_position'");
      return false;
    }
    auto array_ptr = ros_message->target_position.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: target_velocity
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->target_velocity.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->target_velocity);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->target_velocity, size)) {
      fprintf(stderr, "failed to create array for field 'target_velocity'");
      return false;
    }
    auto array_ptr = ros_message->target_velocity.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: target_torque
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->target_torque.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->target_torque);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->target_torque, size)) {
      fprintf(stderr, "failed to create array for field 'target_torque'");
      return false;
    }
    auto array_ptr = ros_message->target_torque.data;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rl_interfaces
size_t get_serialized_size_rl_interfaces__msg__MotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotionCommand__ros_msg_type * ros_message = static_cast<const _MotionCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: drive_ids
  {
    size_t array_size = ros_message->drive_ids.size;
    auto array_ptr = ros_message->drive_ids.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: target_position
  {
    size_t array_size = ros_message->target_position.size;
    auto array_ptr = ros_message->target_position.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: target_velocity
  {
    size_t array_size = ros_message->target_velocity.size;
    auto array_ptr = ros_message->target_velocity.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: target_torque
  {
    size_t array_size = ros_message->target_torque.size;
    auto array_ptr = ros_message->target_torque.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rl_interfaces
size_t max_serialized_size_rl_interfaces__msg__MotionCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: drive_ids
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: target_position
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: target_velocity
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: target_torque
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rl_interfaces__msg__MotionCommand;
    is_plain =
      (
      offsetof(DataType, target_torque) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rl_interfaces
bool cdr_serialize_key_rl_interfaces__msg__MotionCommand(
  const rl_interfaces__msg__MotionCommand * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: drive_ids
  {
    size_t size = ros_message->drive_ids.size;
    auto array_ptr = ros_message->drive_ids.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: target_position
  {
    size_t size = ros_message->target_position.size;
    auto array_ptr = ros_message->target_position.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: target_velocity
  {
    size_t size = ros_message->target_velocity.size;
    auto array_ptr = ros_message->target_velocity.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: target_torque
  {
    size_t size = ros_message->target_torque.size;
    auto array_ptr = ros_message->target_torque.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rl_interfaces
size_t get_serialized_size_key_rl_interfaces__msg__MotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotionCommand__ros_msg_type * ros_message = static_cast<const _MotionCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: drive_ids
  {
    size_t array_size = ros_message->drive_ids.size;
    auto array_ptr = ros_message->drive_ids.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: target_position
  {
    size_t array_size = ros_message->target_position.size;
    auto array_ptr = ros_message->target_position.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: target_velocity
  {
    size_t array_size = ros_message->target_velocity.size;
    auto array_ptr = ros_message->target_velocity.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: target_torque
  {
    size_t array_size = ros_message->target_torque.size;
    auto array_ptr = ros_message->target_torque.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rl_interfaces
size_t max_serialized_size_key_rl_interfaces__msg__MotionCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: drive_ids
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: target_position
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: target_velocity
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: target_torque
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rl_interfaces__msg__MotionCommand;
    is_plain =
      (
      offsetof(DataType, target_torque) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _MotionCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const rl_interfaces__msg__MotionCommand * ros_message = static_cast<const rl_interfaces__msg__MotionCommand *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_rl_interfaces__msg__MotionCommand(ros_message, cdr);
}

static bool _MotionCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  rl_interfaces__msg__MotionCommand * ros_message = static_cast<rl_interfaces__msg__MotionCommand *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_rl_interfaces__msg__MotionCommand(cdr, ros_message);
}

static uint32_t _MotionCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rl_interfaces__msg__MotionCommand(
      untyped_ros_message, 0));
}

static size_t _MotionCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rl_interfaces__msg__MotionCommand(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MotionCommand = {
  "rl_interfaces::msg",
  "MotionCommand",
  _MotionCommand__cdr_serialize,
  _MotionCommand__cdr_deserialize,
  _MotionCommand__get_serialized_size,
  _MotionCommand__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _MotionCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotionCommand,
  get_message_typesupport_handle_function,
  &rl_interfaces__msg__MotionCommand__get_type_hash,
  &rl_interfaces__msg__MotionCommand__get_type_description,
  &rl_interfaces__msg__MotionCommand__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rl_interfaces, msg, MotionCommand)() {
  return &_MotionCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
