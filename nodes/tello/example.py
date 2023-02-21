#!/usr/bin/python3

from tello import Tello
import cv2
from time import sleep
def handle_state(data,ip):
    print(ip)
    print(data)

def handle_frame(data):
    cv2.imshow('frame',data)
    cv2.waitKey(1)

# Note that the additional parameters are optional

# Create an instance of the drone
drone = Tello("192.168.10.1", 8889,"0.0.0.0")
# Begin data streaming
drone.start_data_stream(handle_state,"0.0.0.0")
# Begin video streaming
drone.start_video_stream(handle_frame,"0.0.0.0")

# Poll to keep alive
while True:
    drone.command(False)
    sleep(1)