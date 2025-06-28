// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from md_interfaces:msg/MdRobotMsg2.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__STRUCT_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__md_interfaces__msg__MdRobotMsg2 __attribute__((deprecated))
#else
# define DEPRECATED__md_interfaces__msg__MdRobotMsg2 __declspec(deprecated)
#endif

namespace md_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MdRobotMsg2_
{
  using Type = MdRobotMsg2_<ContainerAllocator>;

  explicit MdRobotMsg2_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->interval_time = 0.0;
      this->x_pos = 0l;
      this->y_pos = 0l;
      this->angule = 0;
      this->us_1 = 0;
      this->us_2 = 0;
      this->us_3 = 0;
      this->us_4 = 0;
      this->platform_state = 0;
      this->linear_velocity = 0;
      this->angular_velocity = 0;
      this->input_voltage = 0.0f;
    }
  }

  explicit MdRobotMsg2_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->interval_time = 0.0;
      this->x_pos = 0l;
      this->y_pos = 0l;
      this->angule = 0;
      this->us_1 = 0;
      this->us_2 = 0;
      this->us_3 = 0;
      this->us_4 = 0;
      this->platform_state = 0;
      this->linear_velocity = 0;
      this->angular_velocity = 0;
      this->input_voltage = 0.0f;
    }
  }

  // field types and members
  using _interval_time_type =
    double;
  _interval_time_type interval_time;
  using _x_pos_type =
    int32_t;
  _x_pos_type x_pos;
  using _y_pos_type =
    int32_t;
  _y_pos_type y_pos;
  using _angule_type =
    int16_t;
  _angule_type angule;
  using _us_1_type =
    uint8_t;
  _us_1_type us_1;
  using _us_2_type =
    uint8_t;
  _us_2_type us_2;
  using _us_3_type =
    uint8_t;
  _us_3_type us_3;
  using _us_4_type =
    uint8_t;
  _us_4_type us_4;
  using _platform_state_type =
    uint8_t;
  _platform_state_type platform_state;
  using _linear_velocity_type =
    int16_t;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    int16_t;
  _angular_velocity_type angular_velocity;
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
  Type & set__x_pos(
    const int32_t & _arg)
  {
    this->x_pos = _arg;
    return *this;
  }
  Type & set__y_pos(
    const int32_t & _arg)
  {
    this->y_pos = _arg;
    return *this;
  }
  Type & set__angule(
    const int16_t & _arg)
  {
    this->angule = _arg;
    return *this;
  }
  Type & set__us_1(
    const uint8_t & _arg)
  {
    this->us_1 = _arg;
    return *this;
  }
  Type & set__us_2(
    const uint8_t & _arg)
  {
    this->us_2 = _arg;
    return *this;
  }
  Type & set__us_3(
    const uint8_t & _arg)
  {
    this->us_3 = _arg;
    return *this;
  }
  Type & set__us_4(
    const uint8_t & _arg)
  {
    this->us_4 = _arg;
    return *this;
  }
  Type & set__platform_state(
    const uint8_t & _arg)
  {
    this->platform_state = _arg;
    return *this;
  }
  Type & set__linear_velocity(
    const int16_t & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const int16_t & _arg)
  {
    this->angular_velocity = _arg;
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
    md_interfaces::msg::MdRobotMsg2_<ContainerAllocator> *;
  using ConstRawPtr =
    const md_interfaces::msg::MdRobotMsg2_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<md_interfaces::msg::MdRobotMsg2_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<md_interfaces::msg::MdRobotMsg2_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdRobotMsg2_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdRobotMsg2_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::MdRobotMsg2_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::MdRobotMsg2_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<md_interfaces::msg::MdRobotMsg2_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<md_interfaces::msg::MdRobotMsg2_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__md_interfaces__msg__MdRobotMsg2
    std::shared_ptr<md_interfaces::msg::MdRobotMsg2_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__md_interfaces__msg__MdRobotMsg2
    std::shared_ptr<md_interfaces::msg::MdRobotMsg2_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MdRobotMsg2_ & other) const
  {
    if (this->interval_time != other.interval_time) {
      return false;
    }
    if (this->x_pos != other.x_pos) {
      return false;
    }
    if (this->y_pos != other.y_pos) {
      return false;
    }
    if (this->angule != other.angule) {
      return false;
    }
    if (this->us_1 != other.us_1) {
      return false;
    }
    if (this->us_2 != other.us_2) {
      return false;
    }
    if (this->us_3 != other.us_3) {
      return false;
    }
    if (this->us_4 != other.us_4) {
      return false;
    }
    if (this->platform_state != other.platform_state) {
      return false;
    }
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->input_voltage != other.input_voltage) {
      return false;
    }
    return true;
  }
  bool operator!=(const MdRobotMsg2_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MdRobotMsg2_

// alias to use template instance with default allocator
using MdRobotMsg2 =
  md_interfaces::msg::MdRobotMsg2_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__STRUCT_HPP_
