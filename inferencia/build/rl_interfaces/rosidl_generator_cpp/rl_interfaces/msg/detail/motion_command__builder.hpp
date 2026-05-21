// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rl_interfaces:msg/MotionCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rl_interfaces/msg/motion_command.hpp"


#ifndef RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_
#define RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rl_interfaces/msg/detail/motion_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rl_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotionCommand_target_torque
{
public:
  explicit Init_MotionCommand_target_torque(::rl_interfaces::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  ::rl_interfaces::msg::MotionCommand target_torque(::rl_interfaces::msg::MotionCommand::_target_torque_type arg)
  {
    msg_.target_torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rl_interfaces::msg::MotionCommand msg_;
};

class Init_MotionCommand_target_velocity
{
public:
  explicit Init_MotionCommand_target_velocity(::rl_interfaces::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_target_torque target_velocity(::rl_interfaces::msg::MotionCommand::_target_velocity_type arg)
  {
    msg_.target_velocity = std::move(arg);
    return Init_MotionCommand_target_torque(msg_);
  }

private:
  ::rl_interfaces::msg::MotionCommand msg_;
};

class Init_MotionCommand_target_position
{
public:
  explicit Init_MotionCommand_target_position(::rl_interfaces::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_target_velocity target_position(::rl_interfaces::msg::MotionCommand::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_MotionCommand_target_velocity(msg_);
  }

private:
  ::rl_interfaces::msg::MotionCommand msg_;
};

class Init_MotionCommand_drive_ids
{
public:
  Init_MotionCommand_drive_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionCommand_target_position drive_ids(::rl_interfaces::msg::MotionCommand::_drive_ids_type arg)
  {
    msg_.drive_ids = std::move(arg);
    return Init_MotionCommand_target_position(msg_);
  }

private:
  ::rl_interfaces::msg::MotionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rl_interfaces::msg::MotionCommand>()
{
  return rl_interfaces::msg::builder::Init_MotionCommand_drive_ids();
}

}  // namespace rl_interfaces

#endif  // RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_
