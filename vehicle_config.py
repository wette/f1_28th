#ip addresses of vehicles
vehicles = {
        "orange": {
            "ip": "10.134.137.90", 
            "port": 6446,
            "motor_pid": (20, 0, 0.01),    #PID parameters to control the motor
            "steering_pid": (0.6, 0.0, 0),  #PID parameters to control the steering
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "steering_measurements" : [(10, -45.0), (170, 45.0)],   #TODO: fill out using vehicle_calibration.py
            "max_steering_angle": 40,
            "min_steering_angle": -40,
            "min_motor_value" : 80,         #min motor voltage (0-255) to get the vehicle moving
            "max_motor_value" : 160,        #max motor voltage (0-255)
            "speedFactor": 1.0,
            "lidar_field_of_view_deg": 80, 
            "lidar_numRays": 50, 
            "lidar_rayLength_m" : 1.0
        },
        "green": {
            "ip": "10.134.137.41",
            "port": 6446,
            "motor_pid": (6, 0, 0),         #PID parameters to control the motor
            "steering_pid": (1.0, 0.0001, 0),  #PID parameters to control the steering
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "steering_measurements" :  [(10, -46.0), (18, -46.0), (26, -46.0), (34, -37.0), (42, -31.0), (50, -28.0), (58, -21.0), (66, -16.0), (74, -12.0), (82, -7.0), (90, 0.0), (98, 2.0), (106, 11.0), (114, 13.0), (122, 21.0), (130, 25.0), (138, 35.0), (146, 38.0), (154, 40.0), (162, 40.0), (170, 40.0)],
            #"steering_measurements" :  [(10, -43.0), (38, -40.0), (51, -31.0), (64, -20.0), (77, -15.0), (90, 0.0), (103, 12.0), (116, 20.0), (129, 27.0), (142, 35.0), (180, 42.0)],
            "max_steering_angle": 35,
            "min_steering_angle": -35,
            "min_motor_value" :60,         #min motor voltage (0-255) to get the vehicle moving
            "max_motor_value" : 110,        #max motor voltage (0-255)
            "speed_factor": 0.9,
            "lidar_field_of_view_deg": 140, 
            "lidar_numRays": 60, 
            "lidar_rayLength_m" : 1.0
        },
        "red": {
            "ip": "10.134.137.42",
            "port": 6446,
            "motor_pid": (10, 0, 0),         #PID parameters to control the motor
            "steering_pid": (1.3, 0.0001, 0),  #PID parameters to control the steering
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "steering_measurements" : [(10, -45.0), (18, -45.0), (26, -45.0), (34, -45.0), (42, -42.0), (50, -40.0), (58, -37.0), (66, -32.0), (74, -22.0), (82, -20.0), (90, -10.0), (98, -6.0), (106, 4.0), (114, 7.0), (122, 15.0), (130, 19.0), (138, 31.0), (146, 35.0), (154, 44.0), (162, 44.0), (170, 44.0)],
            "max_steering_angle": 32,
            "min_steering_angle": -32,
            "min_motor_value" : 70,         #min motor voltage (0-255) to get the vehicle moving
            "max_motor_value" : 160,        #max motor voltage (0-255)
            "speed_factor": 1.0,
            "lidar_field_of_view_deg": 140, 
            "lidar_numRays": 60, 
            "lidar_rayLength_m" : 1.0
        },
}