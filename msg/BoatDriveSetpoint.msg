uint64 timestamp # time since system start (microseconds)

float32 speed # [m/s] collective roll-off speed in body x-axis
bool closed_loop_speed_control # true if speed is controlled using estimator feedback, false if direct feed-forward
float32 rudder_angle # [rad] rudder_angle
bool closed_loop_rudder_angle_control # true if rudder angle is controlled using encoder feedback, false if direct feed-forward
float32 lateral_acceleration # [m/s^2] later acceleration
bool closed_loop_lateral_acceleration_control # true if later_acceleration is controlled using gyroscope feedback, false if direct feed-forward

# TOPICS differential_drive_setpoint differential_drive_control_output
