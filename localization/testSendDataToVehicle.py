import socket
import struct

import sys

motor = int(sys.argv[1])

UDP_IP = "10.134.137.153"
UDP_PORT = 2222

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

msg = bytes()
msg += struct.pack("!H", motor)    #network-byte-order unsigned 16bit int MOTOR
msg += struct.pack("!H", 0)    #network-byte-order unsigned 16bit int MITTE bei 768
print(msg)
print(sock.sendto(msg, (UDP_IP, UDP_PORT)))
