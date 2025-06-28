// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from md_interfaces:msg/MdMotorTicksAndVelo.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__BUILDER_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__BUILDER_HPP_

#include "md_interfaces/msg/detail/md_motor_ticks_and_velo__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace md_interfaces
{

namespace msg
{

namespace builder
{

class Init_MdMotorTicksAndVelo_right_rpm
{
public:
  explicit Init_MdMotorTicksAndVelo_right_rpm(::md_interfaces::msg::MdMotorTicksAndVelo & msg)
  : msg_(msg)
  {}
  ::md_interfaces::msg::MdMotorTicksAndVelo right_rpm(::md_interfaces::msg::MdMotorTicksAndVelo::_right_rpm_type arg)
  {
    msg_.right_rpm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::md_interfaces::msg::MdMotorTicksAndVelo msg_;
};

class Init_MdMotorTicksAndVelo_right_ticks
{
public:
  explicit Init_MdMotorTicksAndVelo_right_ticks(::md_interfaces::msg::MdMotorTicksAndVelo & msg)
  : msg_(msg)
  {}
  Init_MdMotorTicksAndVelo_right_rpm right_ticks(::md_interfaces::msg::MdMotorTicksAndVelo::_right_ticks_type arg)
  {
    msg_.right_ticks = std::move(arg);
    return Init_MdMotorTicksAndVelo_right_rpm(msg_);
  }

private:
  ::md_interfaces::msg::MdMotorTicksAndVelo msg_;
};

class Init_MdMotorTicksAndVelo_left_rpm
{
public:
  explicit Init_MdMotorTicksAndVelo_left_rpm(::md_interfaces::msg::MdMotorTicksAndVelo & msg)
  : msg_(msg)
  {}
  Init_MdMotorTicksAndVelo_right_ticks left_rpm(::md_interfaces::msg::MdMotorTicksAndVelo::_left_rpm_type arg)
  {
    msg_.left_rpm = std::move(arg);
    return Init_MdMotorTicksAndVelo_right_ticks(msg_);
  }

private:
  ::md_interfaces::msg::MdMotorTicksAndVelo msg_;
};

class Init_MdMotorTicksAndVelo_left_ticks
{
public:
  Init_MdMotorTicksAndVelo_left_ticks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MdMotorTicksAndVelo_left_rpm left_ticks(::md_interfaces::msg::MdMotorTicksAndVelo::_left_ticks_type arg)
  {
    msg_.left_ticks = std::move(arg);
    return Init_MdMotorTicksAndVelo_left_rpm(msg_);
  }

private:
  ::md_interfaces::msg::MdMotorTicksAndVelo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::md_interfaces::msg::MdMotorTicksAndVelo>()
{
  return md_interfaces::msg::builder::Init_MdMotorTicksAndVelo_left_ticks();
}

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__BUILDER_HPP_
