// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__STRUCT_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'slow_sd'
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__md_interfaces__msg__MdRobotUserParam __attribute__((deprecated))
#else
# define DEPRECATED__md_interfaces__msg__MdRobotUserParam __declspec(deprecated)
#endif

namespace md_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MdRobotUserParam_
{
  using Type = MdRobotUserParam_<ContainerAllocator>;

  explicit MdRobotUserParam_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : slow_sd(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->param_bit_select = 0;
      this->max_linear_speed_usr = 0.0;
      this->max_angular_speed_usr = 0.0;
      this->brake_type = 0;
    }
  }

  explicit MdRobotUserParam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : slow_sd(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->param_bit_select = 0;
      this->max_linear_speed_usr = 0.0;
      this->max_angular_speed_usr = 0.0;
      this->brake_type = 0;
    }
  }

  // field types and members
  using _param_bit_select_type =
    uint16_t;
  _param_bit_select_type param_bit_select;
  using _slow_sd_type =
    md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator>;
  _slow_sd_type slow_sd;
  using _max_linear_speed_usr_type =
    double;
  _max_linear_speed_usr_type max_linear_speed_usr;
  using _max_angular_speed_usr_type =
    double;
  _max_angular_speed_usr_type max_angular_speed_usr;
  using _brake_type_type =
    uint8_t;
  _brake_type_type brake_type;

  // setters for named parameter idiom
  Type & set__param_bit_select(
    const uint16_t & _arg)
  {
    this->param_bit_select = _arg;
    return *this;
  }
  Type & set__slow_sd(
    const md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator> & _arg)
  {
    this->slow_sd = _arg;
    return *this;
  }
  Type & set__max_linear_speed_usr(
    const double & _arg)
  {
    this->max_linear_speed_usr = _arg;
    return *this;
  }
  Type & set__max_angular_speed_usr(
    const double & _arg)
  {
    this->max_angular_speed_usr = _arg;
    return *this;
  }
  Type & set__brake_type(
    const uint8_t & _arg)
  {
    this->brake_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    md_interfaces::msg::MdRobotUserParam_<ContainerAllocator> *;
  using ConstRawPtr =
    const md_interfaces::msg::MdRobotUserParam_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<md_interfaces::msg::MdRobotUserParam_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<md_interfaces::msg::MdRobotUserParam_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdRobotUserParam_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdRobotUserParam_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdRobotUserParam_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdRobotUserParam_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<md_interfaces::msg::MdRobotUserParam_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<md_interfaces::msg::MdRobotUserParam_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__md_interfaces__msg__MdRobotUserParam
    std::shared_ptr<md_interfaces::msg::MdRobotUserParam_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__md_interfaces__msg__MdRobotUserParam
    std::shared_ptr<md_interfaces::msg::MdRobotUserParam_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MdRobotUserParam_ & other) const
  {
    if (this->param_bit_select != other.param_bit_select) {
      return false;
    }
    if (this->slow_sd != other.slow_sd) {
      return false;
    }
    if (this->max_linear_speed_usr != other.max_linear_speed_usr) {
      return false;
    }
    if (this->max_angular_speed_usr != other.max_angular_speed_usr) {
      return false;
    }
    if (this->brake_type != other.brake_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const MdRobotUserParam_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MdRobotUserParam_

// alias to use template instance with default allocator
using MdRobotUserParam =
  md_interfaces::msg::MdRobotUserParam_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__STRUCT_HPP_
