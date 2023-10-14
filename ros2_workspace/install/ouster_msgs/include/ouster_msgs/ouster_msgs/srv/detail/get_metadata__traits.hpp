// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ouster_msgs:srv/GetMetadata.idl
// generated code does not contain a copyright notice

#ifndef OUSTER_MSGS__SRV__DETAIL__GET_METADATA__TRAITS_HPP_
#define OUSTER_MSGS__SRV__DETAIL__GET_METADATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ouster_msgs/srv/detail/get_metadata__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ouster_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMetadata_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: metadata_filepath
  {
    out << "metadata_filepath: ";
    rosidl_generator_traits::value_to_yaml(msg.metadata_filepath, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMetadata_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: metadata_filepath
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "metadata_filepath: ";
    rosidl_generator_traits::value_to_yaml(msg.metadata_filepath, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMetadata_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ouster_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ouster_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ouster_msgs::srv::GetMetadata_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ouster_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ouster_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ouster_msgs::srv::GetMetadata_Request & msg)
{
  return ouster_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ouster_msgs::srv::GetMetadata_Request>()
{
  return "ouster_msgs::srv::GetMetadata_Request";
}

template<>
inline const char * name<ouster_msgs::srv::GetMetadata_Request>()
{
  return "ouster_msgs/srv/GetMetadata_Request";
}

template<>
struct has_fixed_size<ouster_msgs::srv::GetMetadata_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ouster_msgs::srv::GetMetadata_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ouster_msgs::srv::GetMetadata_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'metadata'
#include "ouster_msgs/msg/detail/metadata__traits.hpp"

namespace ouster_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMetadata_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: metadata
  {
    out << "metadata: ";
    to_flow_style_yaml(msg.metadata, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMetadata_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: metadata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "metadata:\n";
    to_block_style_yaml(msg.metadata, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMetadata_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ouster_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ouster_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ouster_msgs::srv::GetMetadata_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ouster_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ouster_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ouster_msgs::srv::GetMetadata_Response & msg)
{
  return ouster_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ouster_msgs::srv::GetMetadata_Response>()
{
  return "ouster_msgs::srv::GetMetadata_Response";
}

template<>
inline const char * name<ouster_msgs::srv::GetMetadata_Response>()
{
  return "ouster_msgs/srv/GetMetadata_Response";
}

template<>
struct has_fixed_size<ouster_msgs::srv::GetMetadata_Response>
  : std::integral_constant<bool, has_fixed_size<ouster_msgs::msg::Metadata>::value> {};

template<>
struct has_bounded_size<ouster_msgs::srv::GetMetadata_Response>
  : std::integral_constant<bool, has_bounded_size<ouster_msgs::msg::Metadata>::value> {};

template<>
struct is_message<ouster_msgs::srv::GetMetadata_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ouster_msgs::srv::GetMetadata>()
{
  return "ouster_msgs::srv::GetMetadata";
}

template<>
inline const char * name<ouster_msgs::srv::GetMetadata>()
{
  return "ouster_msgs/srv/GetMetadata";
}

template<>
struct has_fixed_size<ouster_msgs::srv::GetMetadata>
  : std::integral_constant<
    bool,
    has_fixed_size<ouster_msgs::srv::GetMetadata_Request>::value &&
    has_fixed_size<ouster_msgs::srv::GetMetadata_Response>::value
  >
{
};

template<>
struct has_bounded_size<ouster_msgs::srv::GetMetadata>
  : std::integral_constant<
    bool,
    has_bounded_size<ouster_msgs::srv::GetMetadata_Request>::value &&
    has_bounded_size<ouster_msgs::srv::GetMetadata_Response>::value
  >
{
};

template<>
struct is_service<ouster_msgs::srv::GetMetadata>
  : std::true_type
{
};

template<>
struct is_service_request<ouster_msgs::srv::GetMetadata_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ouster_msgs::srv::GetMetadata_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // OUSTER_MSGS__SRV__DETAIL__GET_METADATA__TRAITS_HPP_
