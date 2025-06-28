// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from md_interfaces:msg/PosVelTimestamped.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__STRUCT_HPP_
#define MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__md_interfaces__msg__PosVelTimestamped __attribute__((deprecated))
#else
# define DEPRECATED__md_interfaces__msg__PosVelTimestamped __declspec(deprecated)
#endif

namespace md_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PosVelTimestamped_
{
  using Type = PosVelTimestamped_<ContainerAllocator>;

  explicit PosVelTimestamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_pos1 = 0l;
      this->motor_vel1 = 0;
      this->motor_pos2 = 0l;
      this->motor_vel2 = 0l;
    }
  }

  explicit PosVelTimestamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_pos1 = 0l;
      this->motor_vel1 = 0;
      this->motor_pos2 = 0l;
      this->motor_vel2 = 0l;
    }
  }

  // field types and members
  using _motor_pos1_type =
    int32_t;
  _motor_pos1_type motor_pos1;
  using _motor_vel1_type =
    int16_t;
  _motor_vel1_type motor_vel1;
  using _motor_pos2_type =
    int32_t;
  _motor_pos2_type motor_pos2;
  using _motor_vel2_type =
    int32_t;
  _motor_vel2_type motor_vel2;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__motor_pos1(
    const int32_t & _arg)
  {
    this->motor_pos1 = _arg;
    return *this;
  }
  Type & set__motor_vel1(
    const int16_t & _arg)
  {
    this->motor_vel1 = _arg;
    return *this;
  }
  Type & set__motor_pos2(
    const int32_t & _arg)
  {
    this->motor_pos2 = _arg;
    return *this;
  }
  Type & set__motor_vel2(
    const int32_t & _arg)
  {
    this->motor_vel2 = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    md_interfaces::msg::PosVelTimestamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const md_interfaces::msg::PosVelTimestamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<md_interfaces::msg::PosVelTimestamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<md_interfaces::msg::PosVelTimestamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::PosVelTimestamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::PosVelTimestamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::PosVelTimestamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::PosVelTimestamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<md_interfaces::msg::PosVelTimestamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<md_interfaces::msg::PosVelTimestamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__md_interfaces__msg__PosVelTimestamped
    std::shared_ptr<md_interfaces::msg::PosVelTimestamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__md_interfaces__msg__PosVelTimestamped
    std::shared_ptr<md_interfaces::msg::PosVelTimestamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PosVelTimestamped_ & other) const
  {
    if (this->motor_pos1 != other.motor_pos1) {
      return false;
    }
    if (this->motor_vel1 != other.motor_vel1) {
      return false;
    }
    if (this->motor_pos2 != other.motor_pos2) {
      return false;
    }
    if (this->motor_vel2 != other.motor_vel2) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PosVelTimestamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PosVelTimestamped_

// alias to use template instance with default allocator
using PosVelTimestamped =
  md_interfaces::msg::PosVelTimestamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__STRUCT_HPP_
