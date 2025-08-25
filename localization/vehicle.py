import math
import numpy
import socket
import struct
import time

from .helper_functions import *
from .pid_controller import PIDController

from control.disparity_extender import DisparityExtender
from control.lidarSensor import LidarSensor
from control.track import Track

class Vehicle:
    def __init__(self, x, y, yaw, meters_to_pixels):
        #position and orientation of the vehicle
        self.x = x
        self.y = y
        self.yaw = yaw
        self.yaw_rate = 0.0
        
        #IP address
        self.port   = None
        self.IP     = None
        self.sock   = None

        #Offset Steering Servo
        self.servo_offset = 0.0
        self.max_steering_angle_rad = math.radians(45)

        #PID for speed control
        self.motor_pid = None
        self.target_velocity_mps = 0

        #reference to contoroller
        self.controller = None

        #reference to lidar sensor
        self.lidar = None

        #color of the vehicle
        self.color = None

        #convert meters to pixels
        self.meters_to_pixels = meters_to_pixels #how many pixels are in one meter?

        #vehicle dimensions in pixels
        self.length_px = 0.18 * self.meters_to_pixels
        self.width_px = 0.08 * self.meters_to_pixels

        #rear-center offset of the center of rear axle (where the black Dot on the vehicle is)
        self.rear_axle_offset_px = 0.065 * self.meters_to_pixels

        #current speed of the vehicle
        self.vehicle_speed = 0 #meters per second; absolute speed

        self.lateral_acceleration_mps = 0
        self.lateral_speed_mps = 0
        self.longitudinal_acceleration_mps = 0
        self.longitudinal_speed_mps = 0

        self.speed_filter_values = [] #history of speed values to filter
        self.yaw_rate_filter_values = [] #history of yaw rate values to filter

        #timekeeping
        self.is_on_finish_line = False
        self.time_start_of_lap = time.time()
        self.last_laptime = 0

        self.last_update = 0

        self.ttl = 5

    def compute_next_command(self, delta_t : float):
        x, y, yaw = self.getPositionEstimate(self.last_update + delta_t) #TODO: find out which delta is required here!
        distances, rays = self.lidar.getReadings(x, y, yaw)

        
        #TODO: this is not very clean as this code is assuming the controler is a disparity extender
        disparities_indexes, directions = self.controller.find_disparities(distances)
        modified_distances, setpoint, target_steering_angle_rad = self.controller.extend_disparities(distances, 
                                                                                          rays, 
                                                                                          disparities_indexes, 
                                                                                          directions)


        target_steering_angle_rad = target_steering_angle_rad - self.yaw

        #unwind target angle
        while target_steering_angle_rad < -math.pi:
            target_steering_angle_rad += math.pi*2
        while target_steering_angle_rad > math.pi:
            target_steering_angle_rad -= math.pi*2


        #compute target velocity
        target_velocity_mps = 0.5

        #find out how fast we can go: TODO: find out more reasonable numbers.
        target_steering_angle_deg = abs(math.degrees(target_steering_angle_rad))
        if 0 <= target_steering_angle_deg < 5:
            target_velocity_mps = 2.0
        if 5 <= target_steering_angle_deg < 10:
            target_velocity_mps = 1.2
        if 10 <= target_steering_angle_deg < 20:
            target_velocity_mps = 0.9
        if 20 <= target_steering_angle_deg < 30:
            target_velocity_mps = 0.8
        if 30 <= target_steering_angle_deg:
            target_velocity_mps = 0.7

        #dist to setpoint
        d = math.sqrt((setpoint[0]-x)**2 + (setpoint[1]-y)**2) * (1.0/self.meters_to_pixels)
        print(d)
        if d < 0.6:
            target_velocity_mps = min(target_velocity_mps, 0.8)


        self.target_velocity_mps = target_velocity_mps

        return target_velocity_mps, target_steering_angle_rad, rays, setpoint



    def setPhysicalProperties(self, length_m, width_m, rear_axle_offset_m, max_steering_angle_deg, steering_angle_offset_deg):
        self.length_px = length_m * self.meters_to_pixels
        self.width_px = width_m * self.meters_to_pixels
        self.rear_axle_offset_px = rear_axle_offset_m * self.meters_to_pixels
        self.max_steering_angle_rad = math.radians(max_steering_angle_deg)
        self.servo_offset = math.radians(steering_angle_offset_deg)

    def getPosition(self):
        return self.x, self.y
    
    def getOrientation(self):
        return self.yaw
    
    def getSpeed(self):
        return self.vehicle_speed
    
    def updatePose(self, x, y, yaw, current_time):
        dx = x - self.x
        dy = y - self.y
        dt = current_time - self.last_update
        yaw_rate_rad_per_sec = 0.0
        dist = math.sqrt(dx**2 + dy**2)
        if dt > 0:
            speed = dist / dt
            yaw_rate_rad_per_sec = (yaw - self.yaw) / dt
        else:
            speed = self.vehicle_speed

        if speed < 0.05:
            speed = 0.0     #standstill

        #direction: forwards or backwards?
        #TODO: fix bug by unwinding angles. otherwise compare does not work.
        """angle_of_movement = getyaw( (self.x, self.y), (x, y) )
        forwards_motion = False
        if math.radians(-45) < angle_of_movement - yaw < math.radians(45):
            #movement along positive x axis of the vehicle
            forwards_motion = True
        
        if not forwards_motion:
            speed *= -1.0

        print(f"angle_of_movement: {math.degrees(angle_of_movement)}, forwards: {forwards_motion}")"""

        speed = speed / self.meters_to_pixels #convert to m/s

        #compute lateral speed and lateral acceleration
        #current position in coordinate frame of last position to find out lateral and longitudinal acc
        if dt != 0:
            mx, my = rotate(x - self.x, y - self.y, -1 * self.yaw)
            mx /= self.meters_to_pixels #convert to m/s
            my /= self.meters_to_pixels #convert to m/s
            self.lateral_acceleration_mps = my / dt - self.lateral_speed_mps
            self.lateral_speed_mps = my / dt
            self.longitudinal_acceleration_mps = mx / dt - self.longitudinal_speed_mps
            self.longitudinal_speed_mps = mx / dt

        self.speed_filter_values.append(speed)
        self.yaw_rate_filter_values.append(yaw_rate_rad_per_sec)
        self.speed_filter_values = self.speed_filter_values[-5:] #keep 5 most recent values
        self.yaw_rate_filter_values = self.yaw_rate_filter_values[-5:] #keep 5 most recent values

        self.x = x
        self.y = y
        self.vehicle_speed = sum(self.speed_filter_values) / len(self.speed_filter_values) #average
        self.yaw_rate = sum(self.yaw_rate_filter_values) / len(self.yaw_rate_filter_values) #average
        self.yaw = yaw
        self.last_update = current_time
    
    #returns projected x,y,yaw for time at_time
    def getPositionEstimate(self, at_time):
        #TODO: implement constant velocity and turn rate model
        #for now simple linear estimation...
        dt = at_time - self.last_update

        distance = dt * self.vehicle_speed * self.meters_to_pixels
        x1 = self.x + math.cos(self.yaw) * distance
        y1 = self.y + math.sin(self.yaw) * distance

        return x1, y1, self.yaw
    
    #returns coordinates of a boundingbox of the vehicle.
    #at at_time is not None: projected into future.
    def getBoundingBox(self, at_time = None):
        pos_x, pos_y = self.getPosition()
        yaw = self.getOrientation
        if at_time is not None:
            pos_x, pos_y, yaw = self.getPositionEstimate(at_time)

        #define boundingbox in vehicle coordinate system (vehicle drives along x axis)
        bottom_left  = numpy.array([ -self.rear_axle_offset_px, int(-self.width_px/2) ])
        bottom_right = numpy.array([ -self.rear_axle_offset_px,  int(self.width_px/2) ])
        top_left     = numpy.array([ -self.rear_axle_offset_px+self.length_px, int(-self.width_px/2)])
        top_right    = numpy.array([ -self.rear_axle_offset_px+self.length_px,  int(self.width_px/2)])

        #rotate coordinates by yaw angle
        rotMatrix = numpy.array([[numpy.cos(yaw), -numpy.sin(yaw)], 
                         [numpy.sin(yaw),  numpy.cos(yaw)]])
        
        bottom_left = numpy.dot(rotMatrix, bottom_left)
        bottom_right = numpy.dot(rotMatrix, bottom_right)
        top_left = numpy.dot(rotMatrix, top_left)
        top_right = numpy.dot(rotMatrix, top_right)

        #move to x,y position
        bbox = [ list(bottom_left), list(bottom_right), list(top_right), list(top_left) ]
        for p in bbox:
            p[0] += pos_x
            p[1] += pos_y

        return bbox
    
    #initialize the PID controler used to control the speed of the vehicle
    def initMotorPID(self, kp:float, ki:float, kd:float, error_history_length:int):
        self.motor_pid = PIDController(kp, ki, kd, error_history_length)

    def initNetworkConnection(self, ip_adress:str, port:int):
        self.port = port
        self.IP = ip_adress
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP

    # send controls via UDP to the vehicle hardware.
    # target_velocity_mps:          target velocity of the vehicle. in meters/second
    # target_steering_angle_rad:    target steering angle of the vehicle. in map coordiates.
    #                               positive is to the right
    # current_motor_value:          the (voltage) value last set for the vehicle
    #
    # returns:
    # current_motor_value:          the value just set for the motor
    def sendControlsToHardware(self, target_velocity_mps:float, target_steering_angle_rad:float, current_motor_value:int):

        #ask PID control what to do with the motor voltage
        delta_motor_value = self.motor_pid.update(target_velocity_mps - self.vehicle_speed)
        current_motor_value += delta_motor_value

        current_motor_value = int(max(80, min(180, current_motor_value))) #clip between 80 and 240 - keep a minimum motor value to prevent the vehicle from getting stuck.
        #current_motor_value = 80

        #print(f"orig angle: {math.degrees(target_steering_angle_rad)} - ", end="")

        #compute delta steering angle
        """target_steering_angle_rad = target_steering_angle_rad - self.yaw
        #print(f"veh. frame angle: {math.degrees(target_steering_angle_rad)} - ", end="")

        #unwind target angle
        while target_steering_angle_rad < -math.pi:
            target_steering_angle_rad += math.pi*2
        while target_steering_angle_rad > math.pi:
            target_steering_angle_rad -= math.pi*2"""
        
        #print(f"unwound angle: {math.degrees(target_steering_angle_rad)} - ", end="")

        #check steering angle is in bounds
        target_steering_angle_rad = max(-self.max_steering_angle_rad, min(self.max_steering_angle_rad, target_steering_angle_rad))
        #print(f"bounds angle: {math.degrees(target_steering_angle_rad)} - ", end="")

        #TODO: think about filtering the steering angle to make it more smooth --> another PID controller for steering?
        target_steering_angle_rad *= 0.6 # poor man's P controller ;)

        #convert steering angle to number between 10 and 170 (for some reason...), 90 beeing 0°, 10 beeing max left, 170 max right
        #target_steering_angle_rad can be between -self.max_steering_angle_rad and +self.max_steering_angle_rad
        target_angle_percent = target_steering_angle_rad / self.max_steering_angle_rad + 1#between 0 and 2 - one beeing forward steering
        steering_angle = target_angle_percent*80 + 10
        steering_angle = min(steering_angle, 170) #clip at 170
        steering_angle = max(steering_angle, 10)  #clip at 10

        #print(f"PWM steering: {steering_angle}, PWM Motor: {current_motor_value}")

        #steering_angle = int(((target_steering_angle_rad/self.max_steering_angle_rad + 1.0) / 0.5) * 160)+10 #TODO: this is most likely NOT what the vehicle expects!

        #send data over to hardware
        
        #binary protocol
        #msg = bytes()
        #msg += struct.pack("!H", self.current_motor_value)    #network-byte-order unsigned 16bit int
        #msg += struct.pack("!H", steering_angle)              #network-byte-order unsigned 16bit int

        #somehow the students decided to use an utf-8 format?!... TODO: find out what the first value (called "status" in their implementation) actually means and convert to binary format.
        output = [0, current_motor_value, steering_angle]
        msg = ";".join(f"{e:03}" for e in output)
        
        self.sock.sendto(bytes(msg, "utf-8"), (self.IP, self.port))

        return current_motor_value
