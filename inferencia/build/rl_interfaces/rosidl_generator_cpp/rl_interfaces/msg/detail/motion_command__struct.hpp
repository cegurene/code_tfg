// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rl_interfaces:msg/MotionCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rl_interfaces/msg/motion_command.hpp"


#ifndef RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__STRUCT_HPP_
#define RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rl_interfaces__msg__MotionCommand __attribute__((deprecated))
#else
# define DEPRECATED__rl_interfaces__msg__MotionCommand __declspec(deprecated)
#endif

namespace rl_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotionCommand_
{
  using Type = MotionCommand_<ContainerAllocator>;

  explicit MotionCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotionCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _drive_ids_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _drive_ids_type drive_ids;
  using _target_position_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _target_position_type target_position;
  using _target_velocity_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _target_velocity_type target_velocity;
  using _target_torque_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _target_torque_type target_torque;

  // setters for named parameter idiom
  Type & set__drive_ids(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->drive_ids = _arg;
    return *this;
  }
  Type & set__target_position(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__target_velocity(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->target_velocity = _arg;
    return *this;
  }
  Type & set__target_torque(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->target_torque = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rl_interfaces::msg::MotionCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rl_interfaces::msg::MotionCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rl_interfaces::msg::MotionCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rl_interfaces::msg::MotionCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rl_interfaces::msg::MotionCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rl_interfaces::msg::MotionCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rl_interfaces::msg::MotionCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rl_interfaces::msg::MotionCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rl_interfaces::msg::MotionCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rl_interfaces::msg::MotionCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rl_interfaces__msg__MotionCommand
    std::shared_ptr<rl_interfaces::msg::MotionCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rl_interfaces__msg__MotionCommand
    std::shared_ptr<rl_interfaces::msg::MotionCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionCommand_ & other) const
  {
    if (this->drive_ids != other.drive_ids) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->target_velocity != other.target_velocity) {
      return false;
    }
    if (this->target_torque != other.target_torque) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionCommand_

// alias to use template instance with default allocator
using MotionCommand =
  rl_interfaces::msg::MotionCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rl_interfaces

#endif  // RL_INTERFACES__MSG__DETAIL__MOTION_COMMAND__STRUCT_HPP_
