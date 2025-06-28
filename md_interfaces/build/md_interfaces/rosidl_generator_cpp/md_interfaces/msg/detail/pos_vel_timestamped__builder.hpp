// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from md_interfaces:msg/PosVelTimestamped.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__BUILDER_HPP_
#define MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__BUILDER_HPP_

#include "md_interfaces/msg/detail/pos_vel_timestamped__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace md_interfaces
{

namespace msg
{

namespace builder
{

class Init_PosVelTimestamped_stamp
{
public:
  explicit Init_PosVelTimestamped_stamp(::md_interfaces::msg::PosVelTimestamped & msg)
  : msg_(msg)
  {}
  ::md_interfaces::msg::PosVelTimestamped stamp(::md_interfaces::msg::PosVelTimestamped::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::md_interfaces::msg::PosVelTimestamped msg_;
};

class Init_PosVelTimestamped_motor_vel2
{
public:
  explicit Init_PosVelTimestamped_motor_vel2(::md_interfaces::msg::PosVelTimestamped & msg)
  : msg_(msg)
  {}
  Init_PosVelTimestamped_stamp motor_vel2(::md_interfaces::msg::PosVelTimestamped::_motor_vel2_type arg)
  {
    msg_.motor_vel2 = std::move(arg);
    return Init_PosVelTimestamped_stamp(msg_);
  }

private:
  ::md_interfaces::msg::PosVelTimestamped msg_;
};

class Init_PosVelTimestamped_motor_pos2
{
public:
  explicit Init_PosVelTimestamped_motor_pos2(::md_interfaces::msg::PosVelTimestamped & msg)
  : msg_(msg)
  {}
  Init_PosVelTimestamped_motor_vel2 motor_pos2(::md_interfaces::msg::PosVelTimestamped::_motor_pos2_type arg)
  {
    msg_.motor_pos2 = std::move(arg);
    return Init_PosVelTimestamped_motor_vel2(msg_);
  }

private:
  ::md_interfaces::msg::PosVelTimestamped msg_;
};

class Init_PosVelTimestamped_motor_vel1
{
public:
  explicit Init_PosVelTimestamped_motor_vel1(::md_interfaces::msg::PosVelTimestamped & msg)
  : msg_(msg)
  {}
  Init_PosVelTimestamped_motor_pos2 motor_vel1(::md_interfaces::msg::PosVelTimestamped::_motor_vel1_type arg)
  {
    msg_.motor_vel1 = std::move(arg);
    return Init_PosVelTimestamped_motor_pos2(msg_);
  }

private:
  ::md_interfaces::msg::PosVelTimestamped msg_;
};

class Init_PosVelTimestamped_motor_pos1
{
public:
  Init_PosVelTimestamped_motor_pos1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PosVelTimestamped_motor_vel1 motor_pos1(::md_interfaces::msg::PosVelTimestamped::_motor_pos1_type arg)
  {
    msg_.motor_pos1 = std::move(arg);
    return Init_PosVelTimestamped_motor_vel1(msg_);
  }

private:
  ::md_interfaces::msg::PosVelTimestamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::md_interfaces::msg::PosVelTimestamped>()
{
  return md_interfaces::msg::builder::Init_PosVelTimestamped_motor_pos1();
}

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__BUILDER_HPP_
