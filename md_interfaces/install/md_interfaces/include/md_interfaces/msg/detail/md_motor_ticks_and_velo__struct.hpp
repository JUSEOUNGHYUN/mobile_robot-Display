// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from md_interfaces:msg/MdMotorTicksAndVelo.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__STRUCT_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__md_interfaces__msg__MdMotorTicksAndVelo __attribute__((deprecated))
#else
# define DEPRECATED__md_interfaces__msg__MdMotorTicksAndVelo __declspec(deprecated)
#endif

namespace md_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MdMotorTicksAndVelo_
{
  using Type = MdMotorTicksAndVelo_<ContainerAllocator>;

  explicit MdMotorTicksAndVelo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_ticks = 0l;
      this->left_rpm = 0;
      this->right_ticks = 0l;
      this->right_rpm = 0;
    }
  }

  explicit MdMotorTicksAndVelo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_ticks = 0l;
      this->left_rpm = 0;
      this->right_ticks = 0l;
      this->right_rpm = 0;
    }
  }

  // field types and members
  using _left_ticks_type =
    int32_t;
  _left_ticks_type left_ticks;
  using _left_rpm_type =
    int16_t;
  _left_rpm_type left_rpm;
  using _right_ticks_type =
    int32_t;
  _right_ticks_type right_ticks;
  using _right_rpm_type =
    int16_t;
  _right_rpm_type right_rpm;

  // setters for named parameter idiom
  Type & set__left_ticks(
    const int32_t & _arg)
  {
    this->left_ticks = _arg;
    return *this;
  }
  Type & set__left_rpm(
    const int16_t & _arg)
  {
    this->left_rpm = _arg;
    return *this;
  }
  Type & set__right_ticks(
    const int32_t & _arg)
  {
    this->right_ticks = _arg;
    return *this;
  }
  Type & set__right_rpm(
    const int16_t & _arg)
  {
    this->right_rpm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator> *;
  using ConstRawPtr =
    const md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__md_interfaces__msg__MdMotorTicksAndVelo
    std::shared_ptr<md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__md_interfaces__msg__MdMotorTicksAndVelo
    std::shared_ptr<md_interfaces::msg::MdMotorTicksAndVelo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MdMotorTicksAndVelo_ & other) const
  {
    if (this->left_ticks != other.left_ticks) {
      return false;
    }
    if (this->left_rpm != other.left_rpm) {
      return false;
    }
    if (this->right_ticks != other.right_ticks) {
      return false;
    }
    if (this->right_rpm != other.right_rpm) {
      return false;
    }
    return true;
  }
  bool operator!=(const MdMotorTicksAndVelo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MdMotorTicksAndVelo_

// alias to use template instance with default allocator
using MdMotorTicksAndVelo =
  md_interfaces::msg::MdMotorTicksAndVelo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__STRUCT_HPP_
