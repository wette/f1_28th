from localization.camera import Camera
from localization.vehicle import Vehicle
from control.track import Track
import numpy as np
import math


#ip addresses of vehicles
vehicles = {
        "orange": {
            "ip": "10.134.137.90", 
            "port": 6446,
            "motor_pid": (30, 0, 0),    #PID parameters to control the motor
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "max_steering_angle_deg": 45,
            "steering_angle_offset_deg": 0.0
        },
        "green": {
            "ip": "10.134.137.91", 
            "port": 6446,
            "motor_pid": (30, 0, 0),         #PID parameters to control the motor
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "max_steering_angle_deg": 45,
            "steering_angle_offset_deg": 0.0
        },
}

#camera parameters
vertical_resolution_px = 1080
horizontal_resolution_px = 1920
frames_per_seconds = 90
opening_angle_vertical_degrees = 88.0
opening_angle_horizontal_degrees = 126.0

#physical parameters of camera and vehicle features
meters_to_pixels = 1075     #how many pixels are in one meter?
max_speed_vehicle_mps = 4.0        #max speed of a car in meters per second
minimum_brightness = 1.0 #2.7   #used to brighten the image of the webcam
threshold_brightness_of_black = 150       #rgb from 0-255
threshold_brightness_of_white = 200        #rgb from 0-255
circle_diameter_meters = 0.02  #diameter of black and white dots (2cm)
size_between_black_and_white_center_meters = 0.08  #8cm from center to center
height_over_ground_black_meters = 0.042    #how high is the black dot on the vehicle measured from the ground
height_over_ground_white_meters = 0.025    #how high is the white dot on the vehicle measured from the ground

#camera correction parameters
cameraMatrix = np.array([[1.19164513e+03, 0.00000000e+00, 9.32255365e+02],
                         [0.00000000e+00, 1.19269246e+03, 5.44789222e+02],
                         [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortionCoefficients = np.array([[ 0.02473071, -0.39668063,  0.00151336,  0.00085757,  0.25759047]])


def main():

    #setup camera
    cam = Camera(vertical_resolution_px = vertical_resolution_px,
                 horizontal_resolution_px = horizontal_resolution_px,
                 frames_per_seconds = frames_per_seconds,
                 opening_angle_vertical_degrees = opening_angle_vertical_degrees,
                 opening_angle_horizontal_degrees = opening_angle_horizontal_degrees,
                 meters_to_pixels = meters_to_pixels,
                 max_speed_vehicle_mps = max_speed_vehicle_mps,
                 minimum_brightness = minimum_brightness,
                 threshold_brightness_of_black = threshold_brightness_of_black,
                 threshold_brightness_of_white = threshold_brightness_of_white,
                 circle_diameter_meters = circle_diameter_meters,
                 size_between_black_and_white_center_meters = size_between_black_and_white_center_meters,
                 height_over_ground_black_meters = height_over_ground_black_meters,
                 height_over_ground_white_meters = height_over_ground_white_meters,
                 cameraMatrix = cameraMatrix,
                 distortionCoefficients = distortionCoefficients)

    #calibrate (physical) setup of the camera by giving visual guidance
    print("Do you want to calibrate the physical setup of the camera (y/n)?")
    key = input()
    if key.upper() == "Y":
        cam.cameraCalibrationWizard()

    #calibrate track
    print("Do you want to re-set the track boundaries (y/n)?")
    key = input()
    if key.upper() == "Y":
        t = Track()
        t.manuallyPickTrackBorders(cam.get_frame())
        t.saveToFile("track_borders.npy")



    while True:
        if cam.detectVehicles() > 0:
            print(f"found {len(cam.tracked_vehicles)} vehicles.")

            #setup vehicles
            for v in cam.tracked_vehicles:
                if v.color in vehicles.keys():
                    #set physical properties
                    v.setPhysicalProperties(vehicles[v.color]["length_m"], 
                                            vehicles[v.color]["width_m"], 
                                            vehicles[v.color]["rear_axle_offset_m"], 
                                            vehicles[v.color]["max_steering_angle_deg"], 
                                            vehicles[v.color]["steering_angle_offset_deg"])
                    #setup IP communication
                    v.initNetworkConnection(ip_adress=vehicles[v.color]["ip"], port=vehicles[v.color]["port"])

                    #setup servo parameters
                    v.servo_offset = vehicles[v.color]["servo_offset"]

                    #setup motor parameters
                    v.initMotorPID(vehicles[v.color]["motor_pid"][0],
                                   vehicles[v.color]["motor_pid"][1],
                                   vehicles[v.color]["motor_pid"][2])
                    
                    #setup controller
                    v.controller = None
                else:
                    print("Could not properly detect color of vehicle!")

            #tracking loop
            delta_t = 1.0/frames_per_seconds
            while len(cam.tracked_vehicles) > 0:
                #update position of each vehicle
                cam.trackVehicles()

                for v in cam.tracked_vehicles:
                    x, y, yaw = v.getPositionEstimate(0.0)  #TODO: insert delay between frame capture and vehicle action here!
                    #compute vehicle actions
                    target_velocity_mps, target_steering_angle_deg = v.controller.compute_next_command(x, 
                                                                                                y, 
                                                                                                yaw, 
                                                                                                v.vehicle_speed,
                                                                                                delta_t=delta_t)

                    #send actions to vehicle
                    v.sendControlsToHardware(target_velocity_mps=target_velocity_mps,
                                             target_steering_angle_rad=math.radians(target_steering_angle_deg))

        else:
            print("Could not detect any vehicle - restarting.")




if __name__ == "__main__":
    main()