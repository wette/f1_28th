from localization.vehicle import Vehicle
from vehicle_config import vehicles
import socket
import math

def send_steering_pwm(pwm, color):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP

    output = [0, 0, int(pwm)]
    msg = ";".join(f"{e:03}" for e in output)
        
    sock.sendto(bytes(msg, "utf-8"), (vehicles[color]["ip"], vehicles[color]["port"]))

def main():
    color = "red"
    print(f"Calibrating steering for the  #### {color} #### vehicle. Make sure it is turned on.")

    print("turning left is negative steering angle, right positive")

    results = []

    measurement_points = 20
    start = 10
    stop = 170
    delta = int((stop-start)/measurement_points)

    for steering_input in list(range(start, stop, delta)) + [stop]:
        send_steering_pwm(steering_input, color)
        print("Measure steering angle in degrees on the vehicle")
        angle = float(input("Steering Angle [deg]: "))


        results.append( (steering_input, angle) )

    print(results)

if __name__ == "__main__":
    main()
