// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from md_interfaces:msg/MdRobotMsg2.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__TRAITS_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__TRAITS_HPP_

#include "md_interfaces/msg/detail/md_robot_msg2__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<md_interfaces::msg::MdRobotMsg2>()
{
  return "md_interfaces::msg::MdRobotMsg2";
}

template<>
inline const char * name<md_interfaces::msg::MdRobotMsg2>()
{
  return "md_interfaces/msg/MdRobotMsg2";
}

template<>
struct has_fixed_size<md_interfaces::msg::MdRobotMsg2>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<md_interfaces::msg::MdRobotMsg2>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<md_interfaces::msg::MdRobotMsg2>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MD_INTERFACES__MSG__DETAIL__MD_ROBOT_MSG2__TRAITS_HPP_
