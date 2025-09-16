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
from multiprocessing.shared_memory import SharedMemory
import cv2 as cv
import time
import copy
import collections


#ip addresses and parameters of vehicles
from vehicle_config import vehicles

#camera parameters
vertical_resolution_px = 1200
horizontal_resolution_px = 1920
frames_per_seconds = 90
opening_angle_vertical_degrees = 88.0
opening_angle_horizontal_degrees = 126.0

#physical parameters of camera and vehicle features
meters_to_pixels = 681     #how many pixels are in one meter?
max_speed_vehicle_mps = 5.0        #max speed of a car in meters per second
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


# this code is running in another process. 
# Data is transfered from the main thread using the d parameter
def showImageThread(d: dict, track: Track):

    history_plot_motor_values = dict()
    history_plot_delta_speed  = dict()
    history_plot_lateral_speed = dict()
    history_plot_lateral_acceleration = dict()

    for color in Camera.ColorMap.keys():
        history_plot_motor_values[color] = collections.deque(maxlen=100)
        history_plot_delta_speed[color] = collections.deque(maxlen=100)
        history_plot_lateral_speed[color] = collections.deque(maxlen=100)
        history_plot_lateral_acceleration[color] = collections.deque(maxlen=100)

    #connect to shared memory
    shm = SharedMemory(name="frame")

    while d["showVisualization"]:
        vehicles :list[Vehicle]  = d["vehicles"]

        frame = np.ndarray((vertical_resolution_px, horizontal_resolution_px, 3), dtype=np.uint8, buffer=shm.buf).copy()

        if frame is not None:
            #draw control loop frequency:
            i = 1
            for vehicle in vehicles:
                if vehicle.color is not None:
                    if "control_loop_frequency_" + vehicle.color in d.keys():
                        cv.putText(frame, f"Control frequency: {d['control_loop_frequency_' + vehicle.color]:.1f} Hz", (50, 50+i*20), cv.FONT_HERSHEY_SIMPLEX, 0.7,
                            hueToBGR(Camera.ColorMap[vehicle.color]), 1, cv.LINE_AA)
                        i += 1
            
            #draw laptimes:
            i = 0
            for v in vehicles:
                cv.putText(frame, f"Lap time: {(time.time()-v.time_start_of_lap):.2f}s ({v.last_laptime:.2f}s)", (850, 50+i*20), cv.FONT_HERSHEY_SIMPLEX, 1,
                        hueToBGR(Camera.ColorMap[vehicle.color]), 2, cv.LINE_AA)
                i += 1

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
            for vehicle in vehicles:
                color = vehicle.color
                if "target_velocity_delta_mps_"+color in d and "motor_voltage_"+color in d:
                    i += 1
                    dy = 100    #height of y axis.
                    plot_x, plot_y = 20, 400 +i*dy+20
                    
                    delta_speed_m = d["target_velocity_delta_mps_" +  color]
                    motor_value = d["motor_voltage_" +  color]
                    history_plot_motor_values[color].append((motor_value / 255)*dy)
                    history_plot_delta_speed[color].append((delta_speed_m/3)   *dy)

                    bgr = hueToBGR(Camera.ColorMap[color])

                    createPlot(frame, 
                               plot_x, 
                               plot_y, 
                               plot_height=dy, 
                               title="Velocity delta and applied motor voltage",
                               colors=[bgr, [0,0,0]],
                               values=[list(history_plot_motor_values[color]), list(history_plot_delta_speed[color])])

            #plot vehicle speed and accelerations:
            i = -1
            for vehicle in vehicles:
                i += 1
                dy = 100    #height of y axis.
                history_plot_lateral_acceleration[vehicle.color].append(abs(vehicle.lateral_acceleration_mps * dy))
                history_plot_lateral_speed[vehicle.color].append(vehicle.lateral_speed_mps*5 * dy)

                bgr = hueToBGR(Camera.ColorMap[vehicle.color])
                createPlot(frame, 
                            1600, 
                            200 + i*dy+20, 
                            plot_height=dy, 
                            title="Lateral speed",
                            colors=[bgr],
                            values=[list(history_plot_lateral_speed[vehicle.color])],
                            moving_average=10)

            cv.imshow('frame', frame)
            if cv.waitKey(1) == ord('q'):
                break

# this code is running in another process. 
# Data is transfered from the main thread using the d parameter
def controlVehicleThread(d: dict, vehicleColor: str, delta_t: float):
    print(f"Starting controlVehicleThread for the {vehicleColor} vehicle.")
    target_control_frequency = 40
    loop_time_s = 1.0 / target_control_frequency
    current_motor_value = 0
    pid_control_motor : PIDController = None  #we need to store the motor pid in the process as our copy of the vehicle it not synchronized back to the other processes.
    pid_control_steer : PIDController = None  #we need to store the motor pid in the process as our copy of the vehicle it not synchronized back to the other processes.
    command_history = None

    #end-to-end-delay
    end_to_end_delay_s = 0.15 #0.15

    smooth_start_time = time.time()

    while d["raceEnabled"]:
        start_time = time.time()

        #look for the vehicle with our color and out opponents:
        opponents : list[Vehicle] = []
        v         : Vehicle       = None

        for x in d["vehicles"]:
            if x.color == vehicleColor:
                v = x
            else:
                opponents.append(x)
        
        
        
        if v is None:
            #print(f"Skipping controlVehicleThread for the {vehicleColor} vehicle as it is no longer found on the racetrack.")
            d["lidar_rays_" + vehicleColor] = None
            d["setpoints_" + vehicleColor]  = None

            continue
        
        if pid_control_motor is None:
            pid_control_motor = v.motor_pid
            pid_control_steer = v.steering_pid
        else:
            v.motor_pid = pid_control_motor #put in our local copy of the pid controller.
            v.steering_pid = pid_control_steer
        
        if command_history is None:
            command_history = v.command_history #put in our local copy of the command history
        else:
            v.command_history = command_history
        
        v.command_frequency_hz = target_control_frequency

        #compute vehicle actions
        target_velocity_mps, target_steering_angle_rad, rays, setpoint = v.compute_next_command(delta_t=end_to_end_delay_s, opponents=opponents)

        #start smoothly:
        if time.time() - smooth_start_time < 1.0:
            target_velocity_mps = 0.6

        #send actions to vehicle
        current_motor_value = v.sendControlsToHardware(target_velocity_mps=target_velocity_mps,
                                                       target_steering_angle_rad=target_steering_angle_rad,
                                                       current_motor_value=current_motor_value)

        
        #forward lidar information to visualization process through the dict.
        d["lidar_rays_" + vehicleColor]  = rays
        d["setpoints_" +  vehicleColor]  = setpoint        
        d["target_velocity_delta_mps_" +  vehicleColor]  = target_velocity_mps - v.getSpeed()
        d["motor_voltage_" +  vehicleColor]  = current_motor_value


        end_time = time.time()
        dt = end_time - start_time
        sleep_time = loop_time_s - dt

        if sleep_time > 0:
            time.sleep(sleep_time)

        d["control_loop_frequency_" +  vehicleColor] = 1.0/(dt+sleep_time)



def main():


    #TEST
    """v = Vehicle(0,0,0,meters_to_pixels=meters_to_pixels)
    v.color = "green"
    v.setPhysicalProperties(vehicles[v.color]["length_m"], 
                            vehicles[v.color]["width_m"], 
                            vehicles[v.color]["rear_axle_offset_m"], 
                            vehicles[v.color]["max_steering_angle_deg"], 
                            vehicles[v.color]["steering_angle_offset_deg"],
                            vehicles[v.color]["min_motor_value"],
                            vehicles[v.color]["max_motor_value"])
    #setup IP communication
    v.initNetworkConnection(ip_adress=vehicles[v.color]["ip"], port=vehicles[v.color]["port"])

    #setup motor parameters
    v.initMotorPID(vehicles[v.color]["motor_pid"][0],
                    vehicles[v.color]["motor_pid"][1],
                    vehicles[v.color]["motor_pid"][2],
                    error_history_length=50)
    
    v.initSteeringPID(0.6,
                      0.0,
                      0.0,
                      error_history_length=50)

    while True:
        v.sendControlsToHardware(0.0,math.radians(0),0.0)
        input()
        v.sendControlsToHardware(0.0,math.radians(-40),0.0)
        input()
        v.sendControlsToHardware(0.0,math.radians(40),0.0)
        input()"""
    # END TEST

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

    print("Do you want to write the camera stream as video to disk for debugging (y/n)?")
    key = input()
    if key.upper() == "Y":
        cam.setup_debug_video()


    # create manager object for multiprocessing - used for inter-process communication
    # all processes can read and write to this dict.
    manager = Manager()
    d = manager.dict()
    d["vehicles"] = []
    d["showVisualization"] = True
    d["raceEnabled"] = True

    #create shared memory for the last image of the camera (because its faster than the manager!)
    last_frame = cam.get_frame()
    print(last_frame.shape)
    shm = SharedMemory(name="frame", create=True, size=last_frame.nbytes)
    shared_memory_buffer = np.ndarray(last_frame.shape, dtype=np.uint8, buffer=shm.buf)


    #spawn one process for visualization:
    process_visualization = Process(target=showImageThread, args=(d, racetrack))
    process_visualization.start()

    #list of processes - one for each vehicle to compute steering commands
    control_processes = {}


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
                                            vehicles[v.color]["steering_angle_offset_deg"],
                                            vehicles[v.color]["min_motor_value"],
                                            vehicles[v.color]["max_motor_value"])
                    #setup IP communication
                    v.initNetworkConnection(ip_adress=vehicles[v.color]["ip"], port=vehicles[v.color]["port"])

                    #setup motor parameters
                    v.initMotorPID(vehicles[v.color]["motor_pid"][0],
                                   vehicles[v.color]["motor_pid"][1],
                                   vehicles[v.color]["motor_pid"][2],
                                   error_history_length=50)
                    #setup steering parameters
                    v.initSteeringPID(vehicles[v.color]["steering_pid"][0],
                                   vehicles[v.color]["steering_pid"][1],
                                   vehicles[v.color]["steering_pid"][2],
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

                cam.checkFinishLine(top_left=(1000, 950), bottom_right=(1050, 1120))

                #send frame and currently tracked vehicles to other processes
                shared_memory_buffer[:] = cam.get_last_frame()[:]
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

    shm.close()
    shm.unlink()



if __name__ == "__main__":
    main()