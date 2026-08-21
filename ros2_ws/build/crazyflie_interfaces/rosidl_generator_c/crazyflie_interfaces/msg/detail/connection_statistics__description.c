// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice

#include "crazyflie_interfaces/msg/detail/connection_statistics__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_crazyflie_interfaces
const rosidl_type_hash_t *
crazyflie_interfaces__msg__ConnectionStatistics__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x29, 0xb3, 0xb2, 0xc8, 0x0a, 0x10, 0xd5, 0x52,
      0x23, 0x7a, 0x81, 0x66, 0x9e, 0xa8, 0x86, 0x18,
      0xce, 0xda, 0x38, 0x50, 0xcc, 0x68, 0x43, 0xa0,
      0x71, 0xa9, 0x09, 0x9a, 0xf0, 0xe6, 0xb8, 0xe0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char crazyflie_interfaces__msg__ConnectionStatistics__TYPE_NAME[] = "crazyflie_interfaces/msg/ConnectionStatistics";

// Define type names, field names, and default values
static char crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__uri[] = "uri";
static char crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__sent_count[] = "sent_count";
static char crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__sent_ping_count[] = "sent_ping_count";
static char crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__receive_count[] = "receive_count";
static char crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__enqueued_count[] = "enqueued_count";
static char crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__ack_count[] = "ack_count";

static rosidl_runtime_c__type_description__Field crazyflie_interfaces__msg__ConnectionStatistics__FIELDS[] = {
  {
    {crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__uri, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__sent_count, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__sent_ping_count, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__receive_count, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__enqueued_count, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__ConnectionStatistics__FIELD_NAME__ack_count, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
crazyflie_interfaces__msg__ConnectionStatistics__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {crazyflie_interfaces__msg__ConnectionStatistics__TYPE_NAME, 45, 45},
      {crazyflie_interfaces__msg__ConnectionStatistics__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string uri\n"
  "uint64 sent_count\n"
  "uint64 sent_ping_count\n"
  "uint64 receive_count\n"
  "uint64 enqueued_count\n"
  "uint64 ack_count";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
crazyflie_interfaces__msg__ConnectionStatistics__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {crazyflie_interfaces__msg__ConnectionStatistics__TYPE_NAME, 45, 45},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 112, 112},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
crazyflie_interfaces__msg__ConnectionStatistics__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *crazyflie_interfaces__msg__ConnectionStatistics__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
