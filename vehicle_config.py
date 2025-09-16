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
            "steering_measurements" : [],   #TODO: fill out using vehicle_calibration.py
            "min_motor_value" : 80,         #min motor voltage (0-255) to get the vehicle moving
            "max_motor_value" : 160,        #max motor voltage (0-255)
            "lidar_field_of_view_deg": 80, 
            "lidar_numRays": 50, 
            "lidar_rayLength_m" : 1.0
        },
        "green": {
            "ip": "10.134.137.41",
            "port": 6446,
            "motor_pid": (7, 0, 0),         #PID parameters to control the motor
            "steering_pid": (0.6, 0.0001, 0),  #PID parameters to control the steering
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "steering_measurements" : [],   #TODO: fill out using vehicle_calibration.py
            "min_motor_value" : 60,         #min motor voltage (0-255) to get the vehicle moving
            "max_motor_value" : 150,        #max motor voltage (0-255)
            "lidar_field_of_view_deg": 140, 
            "lidar_numRays": 70, 
            "lidar_rayLength_m" : 1.0
        },
        "red": {
            "ip": "10.134.137.42",
            "port": 6446,
            "motor_pid": (7, 0, 0),         #PID parameters to control the motor
            "steering_pid": (0.6, 0.0001, 0),  #PID parameters to control the steering
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "steering_measurements" : [],   #TODO: fill out using vehicle_calibration.py
            "min_motor_value" : 100,         #min motor voltage (0-255) to get the vehicle moving
            "max_motor_value" : 190,        #max motor voltage (0-255)
            "lidar_field_of_view_deg": 140, 
            "lidar_numRays": 70, 
            "lidar_rayLength_m" : 1.0
        },
}