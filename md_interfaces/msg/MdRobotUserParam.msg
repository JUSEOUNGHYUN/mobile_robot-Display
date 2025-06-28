### Message for Managing Robot Configuration Parameters for robot users

# Notifies that which parameters has to be over-written 
uint16 param_bit_select

##1:
# Slow start and slow down
# Acceleration and Deceleration parameter
# Range 0 ~ 1023 / 0 = no change / 1023 = takes about 15s at 0 to 300 RPM or 300 RPM to 0
# You can set slowstart and slowdown param seperately 
MdRobotSlowStartStop slow_sd 

##2:
# Robot Max speed
# User defined Robot maximum speed
float64 max_linear_speed_usr
float64 max_angular_speed_usr

##3:
# Select brake type 
uint8 brake_type 
