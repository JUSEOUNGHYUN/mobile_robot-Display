// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from md_interfaces:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__POSE__STRUCT_HPP_
#define MD_INTERFACES__MSG__DETAIL__POSE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__md_interfaces__msg__Pose __attribute__((deprecated))
#else
# define DEPRECATED__md_interfaces__msg__Pose __declspec(deprecated)
#endif

namespace md_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pose_
{
  using Type = Pose_<ContainerAllocator>;

  explicit Pose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->theta = 0.0f;
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
      this->us_1 = 0;
      this->us_2 = 0;
      this->us_3 = 0;
      this->us_4 = 0;
      this->platform_state = 0;
      this->left_motor_state = 0;
      this->right_motor_state = 0;
    }
  }

  explicit Pose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->theta = 0.0f;
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
      this->us_1 = 0;
      this->us_2 = 0;
      this->us_3 = 0;
      this->us_4 = 0;
      this->platform_state = 0;
      this->left_motor_state = 0;
      this->right_motor_state = 0;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _theta_type =
    float;
  _theta_type theta;
  using _linear_velocity_type =
    float;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    float;
  _angular_velocity_type angular_velocity;
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
  using _left_motor_state_type =
    uint8_t;
  _left_motor_state_type left_motor_state;
  using _right_motor_state_type =
    uint8_t;
  _right_motor_state_type right_motor_state;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__theta(
    const float & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__linear_velocity(
    const float & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const float & _arg)
  {
    this->angular_velocity = _arg;
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
  Type & set__left_motor_state(
    const uint8_t & _arg)
  {
    this->left_motor_state = _arg;
    return *this;
  }
  Type & set__right_motor_state(
    const uint8_t & _arg)
  {
    this->right_motor_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    md_interfaces::msg::Pose_<ContainerAllocator> *;
  using ConstRawPtr =
    const md_interfaces::msg::Pose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<md_interfaces::msg::Pose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<md_interfaces::msg::Pose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::Pose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::Pose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      md_interfaces::msg::Pose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<md_interfaces::msg::Pose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<md_interfaces::msg::Pose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<md_interfaces::msg::Pose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__md_interfaces__msg__Pose
    std::shared_ptr<md_interfaces::msg::Pose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__md_interfaces__msg__Pose
    std::shared_ptr<md_interfaces::msg::Pose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pose_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
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
    if (this->left_motor_state != other.left_motor_state) {
      return false;
    }
    if (this->right_motor_state != other.right_motor_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pose_

// alias to use template instance with default allocator
using Pose =
  md_interfaces::msg::Pose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__POSE__STRUCT_HPP_
