// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from md_interfaces:msg/MdMotorTicksAndVelo.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__TRAITS_HPP_
#define MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__TRAITS_HPP_

#include "md_interfaces/msg/detail/md_motor_ticks_and_velo__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<md_interfaces::msg::MdMotorTicksAndVelo>()
{
  return "md_interfaces::msg::MdMotorTicksAndVelo";
}

template<>
inline const char * name<md_interfaces::msg::MdMotorTicksAndVelo>()
{
  return "md_interfaces/msg/MdMotorTicksAndVelo";
}

template<>
struct has_fixed_size<md_interfaces::msg::MdMotorTicksAndVelo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<md_interfaces::msg::MdMotorTicksAndVelo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<md_interfaces::msg::MdMotorTicksAndVelo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MD_INTERFACES__MSG__DETAIL__MD_MOTOR_TICKS_AND_VELO__TRAITS_HPP_
