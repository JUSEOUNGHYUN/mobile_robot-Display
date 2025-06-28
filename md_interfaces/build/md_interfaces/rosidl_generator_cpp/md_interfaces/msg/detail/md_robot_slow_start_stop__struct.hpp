// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from md_interfaces:msg/MdRobotSlowStartStop.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__STRUCT_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__md_interfaces__msg__MdRobotSlowStartStop __attribute__((deprecated))
#else
# define DEPRECATED__md_interfaces__msg__MdRobotSlowStartStop __declspec(deprecated)
#endif

namespace md_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MdRobotSlowStartStop_
{
  using Type = MdRobotSlowStartStop_<ContainerAllocator>;

  explicit MdRobotSlowStartStop_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slowstart = 0;
      this->slowstop = 0;
    }
  }

  explicit MdRobotSlowStartStop_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slowstart = 0;
      this->slowstop = 0;
    }
  }

  // field types and members
  using _slowstart_type =
    uint16_t;
  _slowstart_type slowstart;
  using _slowstop_type =
    uint16_t;
  _slowstop_type slowstop;

  // setters for named parameter idiom
  Type & set__slowstart(
    const uint16_t & _arg)
  {
    this->slowstart = _arg;
    return *this;
  }
  Type & set__slowstop(
    const uint16_t & _arg)
  {
    this->slowstop = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator> *;
  using ConstRawPtr =
    const md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__md_interfaces__msg__MdRobotSlowStartStop
    std::shared_ptr<md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__md_interfaces__msg__MdRobotSlowStartStop
    std::shared_ptr<md_interfaces::msg::MdRobotSlowStartStop_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MdRobotSlowStartStop_ & other) const
  {
    if (this->slowstart != other.slowstart) {
      return false;
    }
    if (this->slowstop != other.slowstop) {
      return false;
    }
    return true;
  }
  bool operator!=(const MdRobotSlowStartStop_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MdRobotSlowStartStop_

// alias to use template instance with default allocator
using MdRobotSlowStartStop =
  md_interfaces::msg::MdRobotSlowStartStop_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__STRUCT_HPP_
