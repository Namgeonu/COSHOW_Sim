// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from crazyflie_interfaces:msg/ConnectionStatisticsArray.idl
// generated code does not contain a copyright notice

#include "crazyflie_interfaces/msg/detail/connection_statistics_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_crazyflie_interfaces
const rosidl_type_hash_t *
crazyflie_interfaces__msg__ConnectionStatisticsArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7c, 0x4b, 0x09, 0x5f, 0xd4, 0x23, 0x92, 0x6c,
      0xb6, 0xad, 0x69, 0xa3, 0x4d, 0x1c, 0x7e, 0x65,
      0xd0, 0x2a, 0xca, 0xb0, 0xf1, 0x19, 0x7a, 0xe4,
      0xc5, 0x8e, 0x58, 0xce, 0xc5, 0x2c, 0x3d, 0xe9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "crazyflie_interfaces/msg/detail/connection_statistics__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t crazyflie_interfaces__msg__ConnectionStatistics__EXPECTED_HASH = {1, {
    0x29, 0xb3, 0xb2, 0xc8, 0x0a, 0x10, 0xd5, 0x52,
    0x23, 0x7a, 0x81, 0x66, 0x9e, 0xa8, 0x86, 0x18,
    0xce, 0xda, 0x38, 0x50, 0xcc, 0x68, 0x43, 0xa0,
    0x71, 0xa9, 0x09, 0x9a, 0xf0, 0xe6, 0xb8, 0xe0,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char crazyflie_interfaces__msg__ConnectionStatisticsArray__TYPE_NAME[] = "crazyflie_interfaces/msg/ConnectionStatisticsArray";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char crazyflie_interfaces__msg__ConnectionStatistics__TYPE_NAME[] = "crazyflie_interfaces/msg/ConnectionStatistics";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char crazyflie_interfaces__msg__ConnectionStatisticsArray__FIELD_NAME__header[] = "header";
static char crazyflie_interfaces__msg__ConnectionStatisticsArray__FIELD_NAME__stats[] = "stats";

static rosidl_runtime_c__type_description__Field crazyflie_interfaces__msg__ConnectionStatisticsArray__FIELDS[] = {
  {
    {crazyflie_interfaces__msg__ConnectionStatisticsArray__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__ConnectionStatisticsArray__FIELD_NAME__stats, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {crazyflie_interfaces__msg__ConnectionStatistics__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription crazyflie_interfaces__msg__ConnectionStatisticsArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__ConnectionStatistics__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
crazyflie_interfaces__msg__ConnectionStatisticsArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {crazyflie_interfaces__msg__ConnectionStatisticsArray__TYPE_NAME, 50, 50},
      {crazyflie_interfaces__msg__ConnectionStatisticsArray__FIELDS, 2, 2},
    },
    {crazyflie_interfaces__msg__ConnectionStatisticsArray__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&crazyflie_interfaces__msg__ConnectionStatistics__EXPECTED_HASH, crazyflie_interfaces__msg__ConnectionStatistics__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = crazyflie_interfaces__msg__ConnectionStatistics__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "ConnectionStatistics[] stats";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
crazyflie_interfaces__msg__ConnectionStatisticsArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {crazyflie_interfaces__msg__ConnectionStatisticsArray__TYPE_NAME, 50, 50},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 52, 52},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
crazyflie_interfaces__msg__ConnectionStatisticsArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *crazyflie_interfaces__msg__ConnectionStatisticsArray__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *crazyflie_interfaces__msg__ConnectionStatistics__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
