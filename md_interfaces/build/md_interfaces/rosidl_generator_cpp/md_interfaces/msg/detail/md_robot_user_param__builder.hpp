// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__BUILDER_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__BUILDER_HPP_

#include "md_interfaces/msg/detail/md_robot_user_param__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace md_interfaces
{

namespace msg
{

namespace builder
{

class Init_MdRobotUserParam_brake_type
{
public:
  explicit Init_MdRobotUserParam_brake_type(::md_interfaces::msg::MdRobotUserParam & msg)
  : msg_(msg)
  {}
  ::md_interfaces::msg::MdRobotUserParam brake_type(::md_interfaces::msg::MdRobotUserParam::_brake_type_type arg)
  {
    msg_.brake_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotUserParam msg_;
};

class Init_MdRobotUserParam_max_angular_speed_usr
{
public:
  explicit Init_MdRobotUserParam_max_angular_speed_usr(::md_interfaces::msg::MdRobotUserParam & msg)
  : msg_(msg)
  {}
  Init_MdRobotUserParam_brake_type max_angular_speed_usr(::md_interfaces::msg::MdRobotUserParam::_max_angular_speed_usr_type arg)
  {
    msg_.max_angular_speed_usr = std::move(arg);
    return Init_MdRobotUserParam_brake_type(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotUserParam msg_;
};

class Init_MdRobotUserParam_max_linear_speed_usr
{
public:
  explicit Init_MdRobotUserParam_max_linear_speed_usr(::md_interfaces::msg::MdRobotUserParam & msg)
  : msg_(msg)
  {}
  Init_MdRobotUserParam_max_angular_speed_usr max_linear_speed_usr(::md_interfaces::msg::MdRobotUserParam::_max_linear_speed_usr_type arg)
  {
    msg_.max_linear_speed_usr = std::move(arg);
    return Init_MdRobotUserParam_max_angular_speed_usr(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotUserParam msg_;
};

class Init_MdRobotUserParam_slow_sd
{
public:
  explicit Init_MdRobotUserParam_slow_sd(::md_interfaces::msg::MdRobotUserParam & msg)
  : msg_(msg)
  {}
  Init_MdRobotUserParam_max_linear_speed_usr slow_sd(::md_interfaces::msg::MdRobotUserParam::_slow_sd_type arg)
  {
    msg_.slow_sd = std::move(arg);
    return Init_MdRobotUserParam_max_linear_speed_usr(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotUserParam msg_;
};

class Init_MdRobotUserParam_param_bit_select
{
public:
  Init_MdRobotUserParam_param_bit_select()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MdRobotUserParam_slow_sd param_bit_select(::md_interfaces::msg::MdRobotUserParam::_param_bit_select_type arg)
  {
    msg_.param_bit_select = std::move(arg);
    return Init_MdRobotUserParam_slow_sd(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotUserParam msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::md_interfaces::msg::MdRobotUserParam>()
{
  return md_interfaces::msg::builder::Init_MdRobotUserParam_param_bit_select();
}

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__BUILDER_HPP_
