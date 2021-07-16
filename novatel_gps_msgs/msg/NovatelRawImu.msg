# Parsed raw IMU data from Novatel OEM7 receiver
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

uint32 gps_week_num
float64 gps_seconds

geometry_msgs/Vector3 angular_velocity

geometry_msgs/Vector3 linear_acceleration
