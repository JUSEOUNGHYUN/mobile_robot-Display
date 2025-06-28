// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from md_interfaces:msg/PosVelTimestamped.idl
// generated code does not contain a copyright notice

#ifndef MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__TRAITS_HPP_
#define MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__TRAITS_HPP_

#include "md_interfaces/msg/detail/pos_vel_timestamped__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<md_interfaces::msg::PosVelTimestamped>()
{
  return "md_interfaces::msg::PosVelTimestamped";
}

template<>
inline const char * name<md_interfaces::msg::PosVelTimestamped>()
{
  return "md_interfaces/msg/PosVelTimestamped";
}

template<>
struct has_fixed_size<md_interfaces::msg::PosVelTimestamped>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<md_interfaces::msg::PosVelTimestamped>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<md_interfaces::msg::PosVelTimestamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MD_INTERFACES__MSG__DETAIL__POS_VEL_TIMESTAMPED__TRAITS_HPP_
