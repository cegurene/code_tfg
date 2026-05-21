// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rl_interfaces:msg/MotionCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rl_interfaces/msg/motion_command.hpp"


#ifndef RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_
#define RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rl_interfaces/msg/detail/motion_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rl_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotionCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: drive_ids
  {
    if (msg.drive_ids.size() == 0) {
      out << "drive_ids: []";
    } else {
      out << "drive_ids: [";
      size_t pending_items = msg.drive_ids.size();
      for (auto item : msg.drive_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target_position
  {
    if (msg.target_position.size() == 0) {
      out << "target_position: []";
    } else {
      out << "target_position: [";
      size_t pending_items = msg.target_position.size();
      for (auto item : msg.target_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target_velocity
  {
    if (msg.target_velocity.size() == 0) {
      out << "target_velocity: []";
    } else {
      out << "target_velocity: [";
      size_t pending_items = msg.target_velocity.size();
      for (auto item : msg.target_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target_torque
  {
    if (msg.target_torque.size() == 0) {
      out << "target_torque: []";
    } else {
      out << "target_torque: [";
      size_t pending_items = msg.target_torque.size();
      for (auto item : msg.target_torque) {
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
  const MotionCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drive_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.drive_ids.size() == 0) {
      out << "drive_ids: []\n";
    } else {
      out << "drive_ids:\n";
      for (auto item : msg.drive_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_position.size() == 0) {
      out << "target_position: []\n";
    } else {
      out << "target_position:\n";
      for (auto item : msg.target_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_velocity.size() == 0) {
      out << "target_velocity: []\n";
    } else {
      out << "target_velocity:\n";
      for (auto item : msg.target_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_torque.size() == 0) {
      out << "target_torque: []\n";
    } else {
      out << "target_torque:\n";
      for (auto item : msg.target_torque) {
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

inline std::string to_yaml(const MotionCommand & msg, bool use_flow_style = false)
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

}  // namespace rl_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rl_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rl_interfaces::msg::MotionCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  rl_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rl_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rl_interfaces::msg::MotionCommand & msg)
{
  return rl_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rl_interfaces::msg::MotionCommand>()
{
  return "rl_interfaces::msg::MotionCommand";
}

template<>
inline const char * name<rl_interfaces::msg::MotionCommand>()
{
  return "rl_interfaces/msg/MotionCommand";
}

template<>
struct has_fixed_size<rl_interfaces::msg::MotionCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rl_interfaces::msg::MotionCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rl_interfaces::msg::MotionCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_
