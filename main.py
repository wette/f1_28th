from localization.camera import Camera
from localization.vehicle import Vehicle
from localization.helper_functions import hueToBGR
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
            "motor_pid": (10, 0, 0.01),    #PID parameters to control the motor
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "max_steering_angle_deg": 45,
            "steering_angle_offset_deg": 0.0,
            "lidar_field_of_view_deg": 60, 
            "lidar_numRays": 40, 
            "lidar_rayLength_m" : 1.0
        },
        "green": {
            "ip": "10.134.137.91", 
            "port": 6446,
            "motor_pid": (30, 0, 0),         #PID parameters to control the motor
            "length_m": 0.18,           #vehicle length
            "width_m":  0.08,           #vehicle width
            "rear_axle_offset_m" : 0.065, #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
            "max_steering_angle_deg": 45,
            "steering_angle_offset_deg": 0.0,
            "lidar_field_of_view_deg": 60, 
            "lidar_numRays": 40, 
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
height_over_ground_black_meters = 0.042    #how high is the black dot on the vehicle measured from the ground
height_over_ground_white_meters = 0.025    #how high is the white dot on the vehicle measured from the ground

#camera correction parameters
cameraMatrix = np.array([[1.19164513e+03, 0.00000000e+00, 9.32255365e+02],
                         [0.00000000e+00, 1.19269246e+03, 5.44789222e+02],
                         [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortionCoefficients = np.array([[ 0.02473071, -0.39668063,  0.00151336,  0.00085757,  0.25759047]])


# this code is running in another process. 
# Data is transfered from the main thread using the d parameter
def showImageThread(d: dict, track: Track):

    history_plot_motor_values = dict()
    history_plot_delta_speed  = dict()

    for color in Camera.ColorMap.keys():
        history_plot_motor_values[color] = collections.deque(maxlen=100)
        history_plot_delta_speed[color] = collections.deque(maxlen=100)


    while d["showVisualization"]:
        frame                    = d["frame"]
        vehicles :list[Vehicle]  = d["vehicles"]

        if frame is not None:
            #draw racetrack:
            for idx in range(-1, len(track.innerBounds)-1):
                cv.line(frame, track.innerBounds[idx], track.innerBounds[idx+1], (255,255,255), 2)

            for idx in range(-1, len(track.outerBounds)-1):
                cv.line(frame, track.outerBounds[idx], track.outerBounds[idx+1], (255,255,255), 2)

            #draw vehicle bounding boxes:
            for vehicle in vehicles:
                boundingbox = vehicle.getBoundingBox(time.time()) #TODO: think about which time to use here!
                color = (0, 255, 0)
                if vehicle.ttl < 15:
                    color = (0, 0, 255)
                Camera.drawBoundingBox(frame, boundingbox, color=color)
                cv.putText(frame, f"Speed: {vehicle.getSpeed():.2f} m/s", (int(boundingbox[0][0]), int(boundingbox[0][1])), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0,255,0), 1, cv.LINE_AA)
                
            #draw lidar rays and setpoint:
            for color in Camera.ColorMap.keys():
                if "lidar_rays_"+color in d:
                    rays : list[LineString] = d["lidar_rays_"+color] #rays:      the rays sent out by the lidar -> list of shapely.LineString objects
                    if rays is not None:
                        for ray in rays:
                            xy = ray.coords.xy
                            cv.line(frame, [int(xy[0][0]), int(xy[1][0])], [int(xy[0][1]), int(xy[1][1])], hueToBGR(Camera.ColorMap[color]), 1)

                
                if "setpoints_"+color in d:
                    sp = d["setpoints_"+color]
                    if sp is not None:    
                        c = [int(sp[0]), int(sp[1])]
                        cv.circle(frame, c, radius=5, color=hueToBGR(Camera.ColorMap[color]), thickness=5, lineType=1)
                

            #plot target speed over demanded motor voltage
            i = -1
            for color in Camera.ColorMap.keys():
                if "target_velocity_delta_mps_"+color in d and "motor_voltage_"+color in d:
                    i += 1

                    dx = 3  #space between two readings in x
                    dy = 100    #height of y axis.
                    plot_x, plot_y = 20, 400 +i*dy+20
                    
                    delta_speed_m = d["target_velocity_delta_mps_" +  color]
                    motor_value = d["motor_voltage_" +  color]
                    history_plot_motor_values[vehicle.color].append((motor_value / 255)*dy)
                    history_plot_delta_speed[vehicle.color].append((delta_speed_m/3)   *dy)

                    bgr = hueToBGR(Camera.ColorMap[color])

                    cv.putText(frame, f"Velocity delta and applied motor voltage", (plot_x, plot_y-dy-10), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                            bgr, 1, cv.LINE_AA)

                    
                    #draw x and y axis:
                    numMaxValues = len(history_plot_motor_values[vehicle.color])
                    cv.line(frame, [plot_x, plot_y], [plot_x+dx*numMaxValues, plot_y], (255,255,255), 1) #x-axis
                    cv.line(frame, [plot_x, plot_y], [plot_x, plot_y-dy], (255,255,255), 1) #y-axis

                    for i in range(0, len(history_plot_motor_values[vehicle.color])-1):
                        cv.line(frame, [plot_x + i*dx, plot_y - int(history_plot_motor_values[vehicle.color][i])], [plot_x + (i+1)*dx, plot_y - int(history_plot_motor_values[vehicle.color][i+1])], (0,0,0), 1)
                        cv.line(frame, [plot_x + i*dx, plot_y - int(history_plot_delta_speed[vehicle.color][i])], [plot_x + (i+1)*dx, plot_y  - int(history_plot_delta_speed[vehicle.color][i+1])], bgr, 1)




                
            cv.imshow('frame', frame)
            if cv.waitKey(1) == ord('q'):
                break

# this code is running in another process. 
# Data is transfered from the main thread using the d parameter
def controlVehicleThread(d: dict, vehicleColor: str, delta_t: float):
    print(f"Starting controlVehicleThread for the {vehicleColor} vehicle.")
    current_motor_value = 0
    while d["raceEnabled"]:

        #look for the vehicle with our color:
        v : Vehicle = None
        for x in d["vehicles"]:
            if x.color == vehicleColor:
                v = x
                break
        
        if v is None:
            print(f"Stopping controlVehicleThread for the {vehicleColor} vehicle as it is no longer found on the racetrack.")
            d["lidar_rays_" + vehicleColor] = None
            d["setpoints_" + vehicleColor]  = None
            return

        #compute vehicle actions
        target_velocity_mps, target_steering_angle_rad, rays, setpoint = v.compute_next_command(delta_t=delta_t)


        #send actions to vehicle
        current_motor_value = v.sendControlsToHardware(target_velocity_mps=target_velocity_mps,
                                                       target_steering_angle_rad=target_steering_angle_rad,
                                                       current_motor_value=current_motor_value)

        
        #forward lidar information to visualization process through the dict.
        d["lidar_rays_" + vehicleColor]  = rays
        d["setpoints_" +  vehicleColor]  = setpoint        
        d["target_velocity_delta_mps_" +  vehicleColor]  = target_velocity_mps - v.getSpeed()
        d["motor_voltage_" +  vehicleColor]  = current_motor_value
        
        time.sleep(1.0/90.0) #execute with 90 Hz-ish TODO: make real 90Hz!



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
    racetrack = Track()
    if key.upper() == "Y":
        
        racetrack.manuallyPickTrackBorders(cam.get_frame())
        racetrack.saveToFile("track_borders.npy")
    else:
        racetrack.loadFromFile("track_borders.npy")

    #create manager object for multiprocessing
    manager = Manager()
    d = manager.dict()
    d["frame"] = None
    d["vehicles"] = []
    d["showVisualization"] = True
    d["raceEnabled"] = True


    #spawn one process for visualization:
    process_visualization = Process(target=showImageThread, args=(d, racetrack))
    process_visualization.start()

    #list of processes - one for each vehicle to compute steering commands
    control_processes = {}

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

                    #setup motor parameters
                    v.initMotorPID(vehicles[v.color]["motor_pid"][0],
                                   vehicles[v.color]["motor_pid"][1],
                                   vehicles[v.color]["motor_pid"][2],
                                   error_history_length=50)
                    
                    #setup controller
                    v.controller = DisparityExtender(car_width=vehicles[v.color]["width_m"]*meters_to_pixels, 
                                                     disparity_threshold=50, 
                                                     tolerance=20)
                    
                    #setup lidar
                    v.lidar = LidarSensor(track=racetrack, 
                                          field_of_view_deg=vehicles[v.color]["lidar_field_of_view_deg"], 
                                          numRays=vehicles[v.color]["lidar_numRays"], 
                                          rayLength_px=vehicles[v.color]["lidar_rayLength_m"] * meters_to_pixels)

                    #create process for controlling:
                    control_processes[v.color] = Process(target=controlVehicleThread, args=(d, v.color, 1.0/frames_per_seconds))
                    control_processes[v.color].start()
                else:
                    print("Could not properly detect color of vehicle!")

            #tracking loop
            delta_t = 1.0/frames_per_seconds
            while len(cam.tracked_vehicles) > 0:
                #update position of each vehicle
                cam.trackVehicles()

                #send frame and currently tracked vehicles to other processes
                d["frame"] = cam.get_last_frame()
                d["vehicles"] = [copy.copy(v) for v in cam.tracked_vehicles]

                #this is now done in processes
                """for v in cam.tracked_vehicles:
                    x, y, yaw = v.getPositionEstimate(0.0)  #TODO: insert delay between frame capture and vehicle action here!
                    #compute vehicle actions
                    target_velocity_mps, target_steering_angle_deg = v.controller.compute_next_command(x, 
                                                                                                y, 
                                                                                                yaw, 
                                                                                                v.vehicle_speed,
                                                                                                delta_t=delta_t)

                    #send actions to vehicle
                    v.sendControlsToHardware(target_velocity_mps=target_velocity_mps,
                                             target_steering_angle_rad=math.radians(target_steering_angle_deg))"""

        else:
            print("Could not detect any vehicle - restarting.")
            d["vehicles"] = []

    #wait for all threads to end.
    process_visualization.join()
    for k in control_processes.keys():
        control_processes[k].join()



if __name__ == "__main__":
    main()