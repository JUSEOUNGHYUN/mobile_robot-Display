// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__TRAITS_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__TRAITS_HPP_

#include "md_interfaces/msg/detail/md_robot_user_param__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'slow_sd'
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<md_interfaces::msg::MdRobotUserParam>()
{
  return "md_interfaces::msg::MdRobotUserParam";
}

template<>
inline const char * name<md_interfaces::msg::MdRobotUserParam>()
{
  return "md_interfaces/msg/MdRobotUserParam";
}

template<>
struct has_fixed_size<md_interfaces::msg::MdRobotUserParam>
  : std::integral_constant<bool, has_fixed_size<md_interfaces::msg::MdRobotSlowStartStop>::value> {};

template<>
struct has_bounded_size<md_interfaces::msg::MdRobotUserParam>
  : std::integral_constant<bool, has_bounded_size<md_interfaces::msg::MdRobotSlowStartStop>::value> {};

template<>
struct is_message<md_interfaces::msg::MdRobotUserParam>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_USER_PARAM__TRAITS_HPP_
