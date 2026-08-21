// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from crazyflie_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#include "crazyflie_interfaces/msg/detail/status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_crazyflie_interfaces
const rosidl_type_hash_t *
crazyflie_interfaces__msg__Status__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xff, 0x5d, 0x15, 0x76, 0x7a, 0x98, 0x40, 0xce,
      0x66, 0x3a, 0x6d, 0x80, 0xe7, 0xe9, 0xeb, 0x18,
      0x79, 0x29, 0xcb, 0x7e, 0x0c, 0xe5, 0x97, 0xf9,
      0xea, 0xea, 0xd0, 0x40, 0x2c, 0x3f, 0x44, 0xb9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char crazyflie_interfaces__msg__Status__TYPE_NAME[] = "crazyflie_interfaces/msg/Status";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char crazyflie_interfaces__msg__Status__FIELD_NAME__header[] = "header";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__supervisor_info[] = "supervisor_info";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__battery_voltage[] = "battery_voltage";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__pm_state[] = "pm_state";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__rssi[] = "rssi";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__num_rx_broadcast[] = "num_rx_broadcast";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__num_tx_broadcast[] = "num_tx_broadcast";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__num_rx_unicast[] = "num_rx_unicast";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__num_tx_unicast[] = "num_tx_unicast";
static char crazyflie_interfaces__msg__Status__FIELD_NAME__latency_unicast[] = "latency_unicast";

static rosidl_runtime_c__type_description__Field crazyflie_interfaces__msg__Status__FIELDS[] = {
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__supervisor_info, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__battery_voltage, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__pm_state, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__rssi, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__num_rx_broadcast, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__num_tx_broadcast, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__num_rx_unicast, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__num_tx_unicast, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {crazyflie_interfaces__msg__Status__FIELD_NAME__latency_unicast, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription crazyflie_interfaces__msg__Status__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
crazyflie_interfaces__msg__Status__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {crazyflie_interfaces__msg__Status__TYPE_NAME, 31, 31},
      {crazyflie_interfaces__msg__Status__FIELDS, 10, 10},
    },
    {crazyflie_interfaces__msg__Status__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "\n"
  "# Constants\n"
  "uint16 SUPERVISOR_INFO_CAN_BE_ARMED = 1\n"
  "uint16 SUPERVISOR_INFO_IS_ARMED     = 2\n"
  "uint16 SUPERVISOR_INFO_AUTO_ARM     = 4\n"
  "uint16 SUPERVISOR_INFO_CAN_FLY      = 8\n"
  "uint16 SUPERVISOR_INFO_IS_FLYING    = 16\n"
  "uint16 SUPERVISOR_INFO_IS_TUMBLED   = 32\n"
  "uint16 SUPERVISOR_INFO_IS_LOCKED    = 64\n"
  "\n"
  "uint8 PM_STATE_BATTERY   = 0\n"
  "uint8 PM_STATE_CHARGING  = 1\n"
  "uint8 PM_STATE_CHARGED   = 2\n"
  "uint8 PM_STATE_LOW_POWER = 3\n"
  "uint8 PM_STATE_SHUTDOWN  = 4\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# general status\n"
  "uint16 supervisor_info # Bitfield, see SUPERVISOR_INFO for individual bits\n"
  "\n"
  "# battery related\n"
  "float32 battery_voltage  # internal battery voltage [V]\n"
  "uint8 pm_state # See PM_STATE_* for potential values\n"
  "\n"
  "# radio related\n"
  "uint8 rssi      # latest radio signal strength indicator [dBm]\n"
  "uint32 num_rx_broadcast  # number of received broadcast packets during the last period\n"
  "uint32 num_tx_broadcast  # number of broadcast packets sent during the last period\n"
  "uint32 num_rx_unicast    # number of received unicast packets during the last period\n"
  "uint32 num_tx_unicast    # number of unicast packets sent during the last period\n"
  "uint16 latency_unicast   # latency (in ms) for unicast packets";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
crazyflie_interfaces__msg__Status__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {crazyflie_interfaces__msg__Status__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1166, 1166},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
crazyflie_interfaces__msg__Status__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *crazyflie_interfaces__msg__Status__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
