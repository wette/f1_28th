from localization.camera import Camera
from localization.vehicle import Vehicle
from localization.helper_functions import hueToBGR, createPlot
from localization.pid_controller import PIDController
from control.track import Track
from control.disparity_extender import DisparityExtender
from control.lidarSensor import LidarSensor
from shapely import LineString
import numpy as np
import math

from multiprocessing import Process, Manager
import cv2 as cv
import time
import copy
import collections


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
            "motor_pid": (20, 0, 0),         #PID parameters to control the motor
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "max_steering_angle_deg": 40,
            "steering_angle_offset_deg": 0.0,
            "lidar_field_of_view_deg": 80, 
            "lidar_numRays": 50, 
            "lidar_rayLength_m" : 1.0
        },
}

#camera parameters
vertical_resolution_px = 1200
horizontal_resolution_px = 1920
frames_per_seconds = 90
opening_angle_vertical_degrees = 88.0
opening_angle_horizontal_degrees = 126.0

#physical parameters of camera and vehicle features
meters_to_pixels = 681     #how many pixels are in one meter?
max_speed_vehicle_mps = 4.0        #max speed of a car in meters per second
minimum_brightness = 1.0 #2.7   #used to brighten the image of the webcam
threshold_brightness_of_black = 150       #rgb from 0-255
threshold_brightness_of_white = 200        #rgb from 0-255
circle_diameter_meters = 0.025  #diameter of black and white dots (2cm)
size_between_black_and_white_center_meters = 0.08  #8cm from center to center
height_over_ground_black_meters = 0.055    #how high is the black dot on the vehicle measured from the ground
height_over_ground_white_meters = 0.035    #how high is the white dot on the vehicle measured from the ground

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
                 distortionCoefficients = distortionCoefficients,
                 from_file="last_video.avi",
                 create_debug_video=False)
    
    racetrack = Track()
    racetrack.loadFromFile("track_borders.npy")


    while cam.is_video_stream_active():
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
                    
                    #setup motor parameters
                    v.initMotorPID(vehicles[v.color]["motor_pid"][0],
                                   vehicles[v.color]["motor_pid"][1],
                                   vehicles[v.color]["motor_pid"][2],
                                   error_history_length=50)
                    
                    #setup controller
                    v.controller = DisparityExtender(car_width=(vehicles[v.color]["width_m"])*meters_to_pixels, 
                                                     disparity_threshold=50, 
                                                     tolerance=20)
                    
                    #setup lidar
                    v.lidar = LidarSensor(track=racetrack, 
                                          field_of_view_deg=vehicles[v.color]["lidar_field_of_view_deg"], 
                                          numRays=vehicles[v.color]["lidar_numRays"], 
                                          rayLength_px=vehicles[v.color]["lidar_rayLength_m"] * meters_to_pixels)
                    
                else:
                    print("Could not properly detect color of vehicle!")

            #tracking loop
            delta_t = 1.0/frames_per_seconds
            while len(cam.tracked_vehicles) > 0:
                
                #update position of each vehicle
                cam.trackVehicles(dt=delta_t)

                cam.checkFinishLine(top_left=(1000, 950), bottom_right=(1050, 1120))

                #send frame and currently tracked vehicles to other processes
                frame = cam.get_last_frame()

                #end-to-end-delay
                end_to_end_delay_s = 0.1

                #virtual opponents
                op = Vehicle(1000, 500, math.radians(-70), meters_to_pixels)
                op.setPhysicalProperties(0.18, 0.08, 0.04, 40, 0)
                virtual_opponents = [op]

                for vehicle in cam.tracked_vehicles:
                    if v.color in vehicles.keys():
                        #compute vehicle actions
                        target_velocity_mps, target_steering_angle_rad, rays, setpoint = vehicle.compute_next_command(delta_t=end_to_end_delay_s, opponents=virtual_opponents)

                        #draw outcome.
                        boundingbox = vehicle.getBoundingBox(cam.current_time) #TODO: think about which time to use here!
                        color = (0, 255, 0)
                        if vehicle.ttl < 15:
                            color = (0, 0, 255)
                        Camera.drawBoundingBox(frame, boundingbox, color=color)
                        cv.putText(frame, f"Speed: {vehicle.getSpeed():.2f} m/s", (int(boundingbox[0][0]), int(boundingbox[0][1])), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0,255,0), 1, cv.LINE_AA)
                        
                        #draw boundingbox set 100ms in future for reference:
                        num_boxes = 3
                        for i in range(0, num_boxes):
                            boundingbox = vehicle.getBoundingBox(cam.current_time + i/num_boxes*end_to_end_delay_s) #TODO: think about which time to use here!
                            color = (255, 255, max(0, 255-i*100))
                            Camera.drawBoundingBox(frame, boundingbox, color=color)

                        
                        #draw lidar rays and setpoint:
                        if rays is not None:
                            for ray in rays:
                                xy = ray.coords.xy
                                cv.line(frame, [int(xy[0][0]), int(xy[1][0])], [int(xy[0][1]), int(xy[1][1])], hueToBGR(Camera.ColorMap[vehicle.color]), 1)

                    
                        if setpoint is not None:    
                            c = [int(setpoint[0]), int(setpoint[1])]
                            cv.circle(frame, c, radius=5, color=hueToBGR(Camera.ColorMap[vehicle.color]), thickness=5, lineType=1)
                    
                cv.imshow('frame', frame)
                key = cv.waitKey(0)
                if key == ord('q'):
                    break
            print("No more cam.tracked_vehicles left.")


        else:
            print("Could not detect any vehicle - restarting.")


if __name__ == "__main__":
    main()