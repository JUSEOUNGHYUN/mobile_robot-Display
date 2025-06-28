// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from md_interfaces:msg/MdRobotSlowStartStop.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__BUILDER_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__BUILDER_HPP_

#include "md_interfaces/msg/detail/md_robot_slow_start_stop__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace md_interfaces
{

namespace msg
{

namespace builder
{

class Init_MdRobotSlowStartStop_slowstop
{
public:
  explicit Init_MdRobotSlowStartStop_slowstop(::md_interfaces::msg::MdRobotSlowStartStop & msg)
  : msg_(msg)
  {}
  ::md_interfaces::msg::MdRobotSlowStartStop slowstop(::md_interfaces::msg::MdRobotSlowStartStop::_slowstop_type arg)
  {
    msg_.slowstop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotSlowStartStop msg_;
};

class Init_MdRobotSlowStartStop_slowstart
{
public:
  Init_MdRobotSlowStartStop_slowstart()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MdRobotSlowStartStop_slowstop slowstart(::md_interfaces::msg::MdRobotSlowStartStop::_slowstart_type arg)
  {
    msg_.slowstart = std::move(arg);
    return Init_MdRobotSlowStartStop_slowstop(msg_);
  }

private:
  ::md_interfaces::msg::MdRobotSlowStartStop msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::md_interfaces::msg::MdRobotSlowStartStop>()
{
  return md_interfaces::msg::builder::Init_MdRobotSlowStartStop_slowstart();
}

}  // namespace md_interfaces

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_SLOW_START_STOP__BUILDER_HPP_
