# This is an attitude message from another robot, received over MAVLink
uint8 sys_id        # System ID of the sender of this message
float32 roll	    # Angular position about body north axis (x) in rad
float32 pitch	    # Angular position about body east axis (y) in rad
float32 yaw	        # Angular position about body down axis (z) in rad
float32 rollspeed   # Angular speed body north axis (x) in rad/s
float32 pitchspeed  # Angular speed body east axis (y) in rad/s
float32 yawspeed    # Angular speed body down axis (z) in rad/s
