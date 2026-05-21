// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rl_interfaces:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#include "rl_interfaces/msg/detail/motion_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rl_interfaces
const rosidl_type_hash_t *
rl_interfaces__msg__MotionCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xec, 0xe4, 0x52, 0x30, 0x3c, 0x10, 0x1b, 0xf5,
      0xc7, 0x75, 0xbb, 0xa6, 0x71, 0x87, 0xb3, 0xe4,
      0xaf, 0x48, 0xd2, 0x07, 0x26, 0x61, 0xeb, 0xba,
      0x07, 0xfd, 0x8f, 0x56, 0x3f, 0xb2, 0x97, 0x1c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rl_interfaces__msg__MotionCommand__TYPE_NAME[] = "rl_interfaces/msg/MotionCommand";

// Define type names, field names, and default values
static char rl_interfaces__msg__MotionCommand__FIELD_NAME__drive_ids[] = "drive_ids";
static char rl_interfaces__msg__MotionCommand__FIELD_NAME__target_position[] = "target_position";
static char rl_interfaces__msg__MotionCommand__FIELD_NAME__target_velocity[] = "target_velocity";
static char rl_interfaces__msg__MotionCommand__FIELD_NAME__target_torque[] = "target_torque";

static rosidl_runtime_c__type_description__Field rl_interfaces__msg__MotionCommand__FIELDS[] = {
  {
    {rl_interfaces__msg__MotionCommand__FIELD_NAME__drive_ids, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rl_interfaces__msg__MotionCommand__FIELD_NAME__target_position, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rl_interfaces__msg__MotionCommand__FIELD_NAME__target_velocity, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rl_interfaces__msg__MotionCommand__FIELD_NAME__target_torque, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rl_interfaces__msg__MotionCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rl_interfaces__msg__MotionCommand__TYPE_NAME, 31, 31},
      {rl_interfaces__msg__MotionCommand__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32[] drive_ids\n"
  "float32[] target_position\n"
  "float32[] target_velocity\n"
  "float32[] target_torque";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rl_interfaces__msg__MotionCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rl_interfaces__msg__MotionCommand__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 95, 95},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rl_interfaces__msg__MotionCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rl_interfaces__msg__MotionCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
