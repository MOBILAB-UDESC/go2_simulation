// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from go2_interfaces:msg/Go2FrontVideoData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "go2_interfaces/msg/go2_front_video_data.hpp"


#ifndef GO2_INTERFACES__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__TRAITS_HPP_
#define GO2_INTERFACES__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "go2_interfaces/msg/detail/go2_front_video_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace go2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Go2FrontVideoData & msg,
  std::ostream & out)
{
  out << "{";
  // member: time_frame
  {
    out << "time_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.time_frame, out);
    out << ", ";
  }

  // member: video720p
  {
    if (msg.video720p.size() == 0) {
      out << "video720p: []";
    } else {
      out << "video720p: [";
      size_t pending_items = msg.video720p.size();
      for (auto item : msg.video720p) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: video360p
  {
    if (msg.video360p.size() == 0) {
      out << "video360p: []";
    } else {
      out << "video360p: [";
      size_t pending_items = msg.video360p.size();
      for (auto item : msg.video360p) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: video180p
  {
    if (msg.video180p.size() == 0) {
      out << "video180p: []";
    } else {
      out << "video180p: [";
      size_t pending_items = msg.video180p.size();
      for (auto item : msg.video180p) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Go2FrontVideoData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: time_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.time_frame, out);
    out << "\n";
  }

  // member: video720p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.video720p.size() == 0) {
      out << "video720p: []\n";
    } else {
      out << "video720p:\n";
      for (auto item : msg.video720p) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: video360p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.video360p.size() == 0) {
      out << "video360p: []\n";
    } else {
      out << "video360p:\n";
      for (auto item : msg.video360p) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: video180p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.video180p.size() == 0) {
      out << "video180p: []\n";
    } else {
      out << "video180p:\n";
      for (auto item : msg.video180p) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Go2FrontVideoData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace go2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use go2_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_interfaces::msg::Go2FrontVideoData & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const go2_interfaces::msg::Go2FrontVideoData & msg)
{
  return go2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<go2_interfaces::msg::Go2FrontVideoData>()
{
  return "go2_interfaces::msg::Go2FrontVideoData";
}

template<>
inline const char * name<go2_interfaces::msg::Go2FrontVideoData>()
{
  return "go2_interfaces/msg/Go2FrontVideoData";
}

template<>
struct has_fixed_size<go2_interfaces::msg::Go2FrontVideoData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<go2_interfaces::msg::Go2FrontVideoData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<go2_interfaces::msg::Go2FrontVideoData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GO2_INTERFACES__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__TRAITS_HPP_
