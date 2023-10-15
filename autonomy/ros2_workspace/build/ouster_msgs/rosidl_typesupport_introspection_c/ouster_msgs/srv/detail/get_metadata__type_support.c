// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ouster_msgs:srv/GetMetadata.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ouster_msgs/srv/detail/get_metadata__rosidl_typesupport_introspection_c.h"
#include "ouster_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ouster_msgs/srv/detail/get_metadata__functions.h"
#include "ouster_msgs/srv/detail/get_metadata__struct.h"


// Include directives for member types
// Member `metadata_filepath`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ouster_msgs__srv__GetMetadata_Request__init(message_memory);
}

void ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_fini_function(void * message_memory)
{
  ouster_msgs__srv__GetMetadata_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_message_member_array[1] = {
  {
    "metadata_filepath",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ouster_msgs__srv__GetMetadata_Request, metadata_filepath),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_message_members = {
  "ouster_msgs__srv",  // message namespace
  "GetMetadata_Request",  // message name
  1,  // number of fields
  sizeof(ouster_msgs__srv__GetMetadata_Request),
  ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_message_member_array,  // message members
  ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_message_type_support_handle = {
  0,
  &ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ouster_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_msgs, srv, GetMetadata_Request)() {
  if (!ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_message_type_support_handle.typesupport_identifier) {
    ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ouster_msgs__srv__GetMetadata_Request__rosidl_typesupport_introspection_c__GetMetadata_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ouster_msgs/srv/detail/get_metadata__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ouster_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ouster_msgs/srv/detail/get_metadata__functions.h"
// already included above
// #include "ouster_msgs/srv/detail/get_metadata__struct.h"


// Include directives for member types
// Member `metadata`
#include "ouster_msgs/msg/metadata.h"
// Member `metadata`
#include "ouster_msgs/msg/detail/metadata__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ouster_msgs__srv__GetMetadata_Response__init(message_memory);
}

void ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_fini_function(void * message_memory)
{
  ouster_msgs__srv__GetMetadata_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_member_array[1] = {
  {
    "metadata",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ouster_msgs__srv__GetMetadata_Response, metadata),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_members = {
  "ouster_msgs__srv",  // message namespace
  "GetMetadata_Response",  // message name
  1,  // number of fields
  sizeof(ouster_msgs__srv__GetMetadata_Response),
  ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_member_array,  // message members
  ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_type_support_handle = {
  0,
  &ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ouster_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_msgs, srv, GetMetadata_Response)() {
  ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_msgs, msg, Metadata)();
  if (!ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_type_support_handle.typesupport_identifier) {
    ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ouster_msgs__srv__GetMetadata_Response__rosidl_typesupport_introspection_c__GetMetadata_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ouster_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ouster_msgs/srv/detail/get_metadata__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_service_members = {
  "ouster_msgs__srv",  // service namespace
  "GetMetadata",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_Request_message_type_support_handle,
  NULL  // response message
  // ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_Response_message_type_support_handle
};

static rosidl_service_type_support_t ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_service_type_support_handle = {
  0,
  &ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_msgs, srv, GetMetadata_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_msgs, srv, GetMetadata_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ouster_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_msgs, srv, GetMetadata)() {
  if (!ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_service_type_support_handle.typesupport_identifier) {
    ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_msgs, srv, GetMetadata_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_msgs, srv, GetMetadata_Response)()->data;
  }

  return &ouster_msgs__srv__detail__get_metadata__rosidl_typesupport_introspection_c__GetMetadata_service_type_support_handle;
}
