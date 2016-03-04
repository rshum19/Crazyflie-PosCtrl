#!/usr/bin/env python
# Original code from rshum19


"""
Code showing how to control the Crazyflie using the ZMQ input socket.
This code will ramp the Crazyflie motors from 25% to 45%

To work, ZMQ has to be enabled in the client configuration and the client needs
to be connected to a Crazyflie.
See https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:zmq#input_device
for the protocol documentation.
"""

import time
import socket
import sys
import struct

try:
    import zmq
except ImportError as e:
    raise Exception("ZMQ library probably not installed ({})".format(e))

context = zmq.Context()
sender = context.socket(zmq.PUSH)
bind_addr = "tcp://127.0.0.1:{}".format(1024 + 188)
sender.connect(bind_addr)

cmdmess = {
    "version": 1,
    "ctrl": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "thrust": 0.0
    }
}
print("starting to send control commands!")

# Take off
for i in range(0, 50, 2):
    thrust = i
    print("command send:", thrust)
    cmdmess["ctrl"]["thrust"] = thrust
    sender.send_json(cmdmess)
    time.sleep(0.075)

# Unlocking thrust protection
#cmdmess["ctrl"]["thrust"] = 0
#sender.send_json(cmdmess)

# Connect to Matlab Socket
UDP_IP = "localhost"
UDP_IP = "192.168.1.215"
UDP_PORT = 51001
BUFF_SIZE = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Read 1st package
cmdrx, addr = sock.recvfrom(BUFF_SIZE)
cmdrx = struct.unpack('!ddddd',cmdrx)
runCMD = cmdrx[0]

while runCMD:
    # Recieve and unpack command
    cmdrx, addr = sock.recvfrom(BUFF_SIZE)
    cmdrx = struct.unpack('!ddddd',cmdrx)

    # Set command values rcvd from Matlab/Simulink
    cmdmess = {
        "version": 1,
        "ctrl": {
            "roll": cmdrx[2],
            "pitch": cmdrx[3],
            "yaw": cmdrx[4],
            "thrust": cmdrx[1]
         }
    }
    print ("recieved command:", cmdrx)

    # Send command via ZMQ to python client
    sender.send_json(cmdmess)

    # Check if Matlab/Simulink is still running
    runCMD = cmdrx[0]

# Land CrazyFlie
print("Landing CrazyFlie...")

for i in range(0, int(cmdrx[1]), 1):
    thrust = cmdrx[1]-i
    print("command send:", thrust)
    cmdmess["ctrl"]["thrust"] = thrust
    sender.send_json(cmdmess)
    time.sleep(0.075)

print ("Closing connection to CrazyFlie Python Client")
cmdmess["ctrl"]["thrust"] = 0
sender.send_json(cmdmess)
