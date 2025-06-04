# Vehicle control commands for autonomous driving
float64 throttle        # Throttle control [0.0, 1.0]
float64 brake          # Brake control [0.0, 1.0]  
float64 steer          # Steering angle in radians [-1.57, 1.57]
bool hand_brake        # Whether to use hand brake
bool reverse           # Whether to reverse
int32 gear             # Gear selection (-1: reverse, 0: neutral, 1-6: forward gears)
float64 target_velocity # Target velocity in m/s (optional, for higher level control)
float64 max_steer_rate  # Maximum steering angle rate in rad/s (optional)
