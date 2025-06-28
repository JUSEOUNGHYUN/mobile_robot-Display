// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from md_interfaces:msg/MdRobotMsg1.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG1__STRUCT_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG1__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__md_interfaces__msg__MdRobotMsg1 __attribute__((deprecated))
#else
# define DEPRECATED__md_interfaces__msg__MdRobotMsg1 __declspec(deprecated)
#endif

namespace md_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MdRobotMsg1_
{
  using Type = MdRobotMsg1_<ContainerAllocator>;

  explicit MdRobotMsg1_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->interval_time = 0.0;
      this->motor1_rpm = 0;
      this->motor1_pos = 0l;
      this->motor2_rpm = 0;
      this->motor2_pos = 0l;
      this->motor1_state = 0;
      this->motor2_state = 0;
      this->input_voltage = 0.0f;
    }
  }

  explicit MdRobotMsg1_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->interval_time = 0.0;
      this->motor1_rpm = 0;
      this->motor1_pos = 0l;
      this->motor2_rpm = 0;
      this->motor2_pos = 0l;
      this->motor1_state = 0;
      this->motor2_state = 0;
      this->input_voltage = 0.0f;
    }
  }

  // field types and members
  using _interval_time_type =
    double;
  _interval_time_type interval_time;
  using _motor1_rpm_type =
    int16_t;
  _motor1_rpm_type motor1_rpm;
  using _motor1_pos_type =
    int32_t;
  _motor1_pos_type motor1_pos;
  using _motor2_rpm_type =
    int16_t;
  _motor2_rpm_type motor2_rpm;
  using _motor2_pos_type =
    int32_t;
  _motor2_pos_type motor2_pos;
  using _motor1_state_type =
    uint8_t;
  _motor1_state_type motor1_state;
  using _motor2_state_type =
    uint8_t;
  _motor2_state_type motor2_state;
  using _input_voltage_type =
    float;
  _input_voltage_type input_voltage;

  // setters for named parameter idiom
  Type & set__interval_time(
    const double & _arg)
  {
    this->interval_time = _arg;
    return *this;
  }
  Type & set__motor1_rpm(
    const int16_t & _arg)
  {
    this->motor1_rpm = _arg;
    return *this;
  }
  Type & set__motor1_pos(
    const int32_t & _arg)
  {
    this->motor1_pos = _arg;
    return *this;
  }
  Type & set__motor2_rpm(
    const int16_t & _arg)
  {
    this->motor2_rpm = _arg;
    return *this;
  }
  Type & set__motor2_pos(
    const int32_t & _arg)
  {
    this->motor2_pos = _arg;
    return *this;
  }
  Type & set__motor1_state(
    const uint8_t & _arg)
  {
    this->motor1_state = _arg;
    return *this;
  }
  Type & set__motor2_state(
    const uint8_t & _arg)
  {
    this->motor2_state = _arg;
    return *this;
  }
  Type & set__input_voltage(
    const float & _arg)
  {
    this->input_voltage = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    md_interfaces::msg::MdRobotMsg1_<ContainerAllocator> *;
  using ConstRawPtr =
    const md_interfaces::msg::MdRobotMsg1_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<md_interfaces::msg::MdRobotMsg1_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<md_interfaces::msg::MdRobotMsg1_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdRobotMsg1_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdRobotMsg1_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdRobotMsg1_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdRobotMsg1_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<md_interfaces::msg::MdRobotMsg1_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<md_interfaces::msg::MdRobotMsg1_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__md_interfaces__msg__MdRobotMsg1
    std::shared_ptr<md_interfaces::msg::MdRobotMsg1_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__md_interfaces__msg__MdRobotMsg1
    std::shared_ptr<md_interfaces::msg::MdRobotMsg1_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MdRobotMsg1_ & other) const
  {
    if (this->interval_time != other.interval_time) {
      return false;
    }
    if (this->motor1_rpm != other.motor1_rpm) {
      return false;
    }
    if (this->motor1_pos != other.motor1_pos) {
      return false;
    }
    if (this->motor2_rpm != other.motor2_rpm) {
      return false;
    }
    if (this->motor2_pos != other.motor2_pos) {
      return false;
    }
    if (this->motor1_state != other.motor1_state) {
      return false;
    }
    if (this->motor2_state != other.motor2_state) {
      return false;
    }
    if (this->input_voltage != other.input_voltage) {
      return false;
    }
    return true;
  }
  bool operator!=(const MdRobotMsg1_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MdRobotMsg1_

// alias to use template instance with default allocator
using MdRobotMsg1 =
  md_interfaces::msg::MdRobotMsg1_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG1__STRUCT_HPP_
