Header header
#latitude(deg),longitude(deg),alitude(m)
geometry_msgs/Vector3 lla

#NED velocity (m/s)
geometry_msgs/Vector3 vel

#euler angle: roll,pitch,yaw (deg)
geometry_msgs/Vector3 euler

#quaternion from euler angle -> world from body
geometry_msgs/Quaternion q_w_from_b

#imu message,linear:acc(g), angular:gyro(deg/s)
geometry_msgs/Accel imu

#daoyuan status
uint8 status

#signal valid 
uint8 valid_flag

float64 HDOP
float64 VDOP
#
int8 temp1
float64 temp2
float64 temp3