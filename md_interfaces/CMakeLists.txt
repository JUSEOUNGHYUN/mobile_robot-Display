cmake_minimum_required(VERSION 3.8)
project(md_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose.msg"
  "msg/MdRobotMsg1.msg"
  "msg/MdRobotMsg2.msg"
  "msg/PosVelTimestamped.msg"
  "msg/MdRobotSlowStartStop.msg"
  "msg/MdRobotUserParam.msg"
  "msg/MdMotorTicksAndVelo.msg"
  DEPENDENCIES builtin_interfaces 
)


ament_export_dependencies(rosidl_default_runtime)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_package()
