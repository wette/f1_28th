#ip addresses of vehicles
vehicles = {
        "orange": {
            "ip": "10.134.137.90", 
            "port": 6446,
            "motor_pid": (20, 0, 0.01),    #PID parameters to control the motor
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "max_steering_angle_deg": 45,
            "steering_angle_offset_deg": 0.0,
            "lidar_field_of_view_deg": 80, 
            "lidar_numRays": 50, 
            "lidar_rayLength_m" : 1.0
        },
        "green": {
            "ip": "10.134.137.41", 
            "port": 6446,
            "motor_pid": (15, 0, 0),         #PID parameters to control the motor
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "max_steering_angle_deg": 40,
            "steering_angle_offset_deg": 0.0,
            "lidar_field_of_view_deg": 140, 
            "lidar_numRays": 140, 
            "lidar_rayLength_m" : 1.0
        },
        "red": {
            "ip": "10.134.137.90", # actually: "10.134.137.42", 
            "port": 6446,
            "motor_pid": (0, 0, 0),         #PID parameters to control the motor
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "max_steering_angle_deg": 40,
            "steering_angle_offset_deg": 0.0,
            "lidar_field_of_view_deg": 140, 
            "lidar_numRays": 140, 
            "lidar_rayLength_m" : 1.0
        },
}