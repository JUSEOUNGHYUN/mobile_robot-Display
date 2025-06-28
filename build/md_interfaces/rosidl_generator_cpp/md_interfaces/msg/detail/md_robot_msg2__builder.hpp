// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from md_interfaces:msg/MdRobotMsg2.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__BUILDER_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__BUILDER_HPP_

#include "md_interfaces/msg/detail/md_robot_msg2__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace md_interfaces
{

namespace msg
{

namespace builder
{

class Init_MdRobotMsg2_input_voltage
{
public:
  explicit Init_MdRobotMsg2_input_voltage(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  ::md_interfaces::msg::MdRobotMsg2 input_voltage(::md_interfaces::msg::MdRobotMsg2::_input_voltage_type arg)
  {
    msg_.input_voltage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_angular_velocity
{
public:
  explicit Init_MdRobotMsg2_angular_velocity(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_input_voltage angular_velocity(::md_interfaces::msg::MdRobotMsg2::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_MdRobotMsg2_input_voltage(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_linear_velocity
{
public:
  explicit Init_MdRobotMsg2_linear_velocity(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_angular_velocity linear_velocity(::md_interfaces::msg::MdRobotMsg2::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_MdRobotMsg2_angular_velocity(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_platform_state
{
public:
  explicit Init_MdRobotMsg2_platform_state(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_linear_velocity platform_state(::md_interfaces::msg::MdRobotMsg2::_platform_state_type arg)
  {
    msg_.platform_state = std::move(arg);
    return Init_MdRobotMsg2_linear_velocity(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_us_4
{
public:
  explicit Init_MdRobotMsg2_us_4(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_platform_state us_4(::md_interfaces::msg::MdRobotMsg2::_us_4_type arg)
  {
    msg_.us_4 = std::move(arg);
    return Init_MdRobotMsg2_platform_state(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_us_3
{
public:
  explicit Init_MdRobotMsg2_us_3(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_us_4 us_3(::md_interfaces::msg::MdRobotMsg2::_us_3_type arg)
  {
    msg_.us_3 = std::move(arg);
    return Init_MdRobotMsg2_us_4(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_us_2
{
public:
  explicit Init_MdRobotMsg2_us_2(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_us_3 us_2(::md_interfaces::msg::MdRobotMsg2::_us_2_type arg)
  {
    msg_.us_2 = std::move(arg);
    return Init_MdRobotMsg2_us_3(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_us_1
{
public:
  explicit Init_MdRobotMsg2_us_1(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_us_2 us_1(::md_interfaces::msg::MdRobotMsg2::_us_1_type arg)
  {
    msg_.us_1 = std::move(arg);
    return Init_MdRobotMsg2_us_2(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_angule
{
public:
  explicit Init_MdRobotMsg2_angule(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_us_1 angule(::md_interfaces::msg::MdRobotMsg2::_angule_type arg)
  {
    msg_.angule = std::move(arg);
    return Init_MdRobotMsg2_us_1(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_y_pos
{
public:
  explicit Init_MdRobotMsg2_y_pos(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_angule y_pos(::md_interfaces::msg::MdRobotMsg2::_y_pos_type arg)
  {
    msg_.y_pos = std::move(arg);
    return Init_MdRobotMsg2_angule(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_x_pos
{
public:
  explicit Init_MdRobotMsg2_x_pos(::md_interfaces::msg::MdRobotMsg2 & msg)
  : msg_(msg)
  {}
  Init_MdRobotMsg2_y_pos x_pos(::md_interfaces::msg::MdRobotMsg2::_x_pos_type arg)
  {
    msg_.x_pos = std::move(arg);
    return Init_MdRobotMsg2_y_pos(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

class Init_MdRobotMsg2_interval_time
{
public:
  Init_MdRobotMsg2_interval_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MdRobotMsg2_x_pos interval_time(::md_interfaces::msg::MdRobotMsg2::_interval_time_type arg)
  {
    msg_.interval_time = std::move(arg);
    return Init_MdRobotMsg2_x_pos(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotMsg2 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::md_interfaces::msg::MdRobotMsg2>()
{
  return md_interfaces::msg::builder::Init_MdRobotMsg2_interval_time();
}

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__BUILDER_HPP_
