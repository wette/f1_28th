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
    print(f"Calibrating steering for the {color} vehicle.")

    results = []

    car_width = 6.5 / 100
    wheelbase = 9.4 / 100

    for pwm in [20, 90, 160]:
        send_steering_pwm(pwm, color)
        print("Measure and Input turn cicle diameter in cm from outside to outside")
        turning_circle = float(input())/100

        angle = math.atan(wheelbase / (turning_circle - car_width)) 

        results.append( (pwm, math.degrees(angle)) )

    print(results)

if __name__ == "__main__":
    main()
