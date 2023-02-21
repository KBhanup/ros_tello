import socket
import threading
import time
import cv2
from .stats import Stats
from .tello_command import TelloCommand
from .tello_datastream import TelloDataStream
from .tello_videostream import TelloVideoStream

## Instantiate a tello drone object
#  Only specify a state port if you have a single drone. All drones will broadcast to the local ip 0.0.0.0 on port 8890
class Tello:

    ## Create the instance of the tello object.
    #  Opens communication to the tello and begins a listening thread.
    #  @param self This object pointer.
    #  @param tello_ip Ip address of the drone on teh network (default: '192.168.10.1')
    #  @param local_port Port to connect on from the local computer (default: 8889)
    #  @param local_ip Ip address of the host computer (default: '0.0.0.0')
    def __init__(self, tello_ip='192.168.10.1', local_port=8889, local_ip='0.0.0.0'):
        print(f"Creating Tello at {tello_ip} from {local_ip}:{local_port}")
        #command interface
        self.tello_ip = tello_ip
        self.local_ip = local_ip
        self.cmd_if = TelloCommand(tello_ip,local_port,local_ip)
        self.command(False)

    ## Delete the current object
    #  @param self This object pointer.
    def __del__(self,):
        if hasattr(self,'datastream'):
            del self.datastream
        if hasattr(self,'videostream'):
            del self.videostream
        if hasattr(self,'cmd_if'):
            self.cmd_if.close()

    # Controll Commands

    ## Connect the Tello to an ap.
    #  This may be called to connect to a router for use with the multi_router modality. The tello will restart after receiving this command.
    #  @param self The object pointer.
    #  @param ap The name (SSID) of the AP.
    #  @param pass The password for the AP.
    def ap(self,ap,passwd):
        return self.cmd_if.ap(ap,passwd)

    ## Send a command to the tello
    #  @param self The object pointer.
    #  @param wait Boolean flag whether to wait on a response (default: True).
    def command(self,wait=True):
        if(wait):
            return self.cmd_if.command(wait)
        self.cmd_if.command(False)

    ## Command the tello to take off.
    #  @param self The object pointer.
    def takeoff(self):
        self.cmd_if.takeoff()

    ## Command the Tello to land
    #  @param self The object pointer.
    def land(self):
        self.cmd_if.land()

    ## Command the Tello to hover.
    #  @param self The object pointer.
    def stop(self):
        self.cmd_if.stop()

    ## Command the Tello to immediately power off motors.
    #  This is an emergency function (hence the name). The motors will immediately power off whether in flight or on the
    #  ground. If it has velocity, your drone is now a projectile.
    #  @param self The object pointer.
    def emergency(self):
        self.cmd_if.emergency()

    ## Command the Tello to begin streaming data.
    #  @param self The object pointer.
    #  @param stream_callback callback to receive data as a dictionary and ip address as a cpuple.
    #  @param local_ip Ip to bind to locally (default: "0.0.0.0").
    def start_data_stream(self,stream_callback, local_ip="0.0.0.0"):
        self.datastream = TelloDataStream(stream_callback,local_ip)
        return self.cmd_if.streamon()

    ## Command the Tello to stop streaming data.
    #  @param self The object pointer.
    def stop_data_stream(self):
        self.datastream.stop()
        del self.datastream
        return self.cmd_if.streamoff()

    ## Command the Tello to begin streaming data.
    #  @param self The object pointer.
    #  @param stream_callback callback to receive the frame data.
    #  @param local_ip Ip to bind to locally (default: "0.0.0.0").
    def start_video_stream(self,stream_callback,local_ip="0.0.0.0"):
        self.videostream = TelloVideoStream(stream_callback,local_ip)

    ## Command the Tello to stop streaming video.
    #  @param self The object pointer.
    def stop_video_stream(self):
        self.videostream.stop()
        del self.videostream


    
    # Movement Commands
    ## Command the Tello to move up.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move up [20,500]
    def up(self, dist):
        return self.cmd_if.up(dist)

    ## Command the Tello to move down.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move down [20,500]
    def down(self, dist):
        return self.cmd_if.down(dist)

    ## Command the Tello to move left.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move left [20,500]
    def left(self, dist):
        return self.cmd_if.left(dist)

    ## Command the Tello to move right.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move right [20,500]
    def right(self, dist):
        return self.cmd_if.right(dist)

    ## Command the Tello to move forward.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move forward [20,500]
    def forward(self, dist):
        return self.cmd_if.forward(dist)

    ## Command the Tello to move back.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move back [20,500]
    def back(self, dist):
        return self.cmd_if.back(dist)

    ## Command the Tello to rotate clockwise.
    #  @param self The object pointer.
    #  @param dist Distance in deg to rotate clockwise [1,360]
    def cw(self, degr):
        return self.cmd_if.cw(degr)

    ## Command the Tello to rotate counter-clockwise.
    #  @param self The object pointer.
    #  @param dist Distance in deg to rotate counter-clockwise [1,360]
    def ccw(self, degr):
        return self.cmd_if.ccw(degr)

    ## Command the Tello to flip.
    #  @param self The object pointer.
    #  @param direc Direction to flip {"l", "r", "f", "b"}.
    def flip(self, direc):
        return self.cmd_if.flip(direc)

    ## Command the Tello to go a distance at a speed.
    #  @param self The object pointer.
    #  @param x Distance in cm to move forward [-500,500]
    #  @param y Distance in cm to move left [-500,500]
    #  @param z Distance in cm to move up [-500,500]
    #  @param speed Speed at which to travel.
    def go(self, x, y, z, speed):
        return self.cmd_if.go(x,y,z,speed)

    ## Command the Tello to move in a curve at a speed.
    # Moves the drone on a curve from the two cordinates and speed.
    #  @param self The object pointer.
    #  @param x1 First x coordinate
    #  @param y1 First y coordinate
    #  @param z1 First z coordinate
    #  @param x2 First x coordinate
    #  @param y2 First y coordinate
    #  @param z2 First z coordinate
    #  @param speed Speed at which to travel.
    def curve(self, x1, y1, z1, x2, y2, z2, speed):
        return self.cmd_if.curve(x1, y1, z1, x2, y2, z2, speed)

    # Set Commands

    ## Set the tello speed
    #  @param self The object pointer.
    #  @param speed Speed to set
    def set_speed(self, speed):
        return self.cmd_if.set_speed(speed)

    ## Set the tello velocities
    #  @param self The object pointer.
    #  @param a velocity in x direction
    #  @param b velocity in y direction
    #  @param c velocity in z direction
    #  @param d angular velocity
    def rc_control(self, a, b, c, d):
        return self.cmd_if.rc_control(a,b,c,d)

    ## Set the tello wifi name and password
    #  @param self The object pointer.
    #  @param ssid name for the network
    #  @param passwrd password for the network
    def set_wifi(self, ssid, passwrd):
        return self.cmd_if.set_wifi(ssid, passwrd)

    # Read Commands
    ## Read the tello speed
    #  @param self The object pointer.
    def get_speed(self):
        return self.cmd_if.get_speed()

    ## Read the tello battery
    #  @param self The object pointer.
    def get_battery(self):
        return self.cmd_if.get_battery()

    ## Read the tello time
    #  @param self The object pointer.
    def get_time(self):
        return self.cmd_if.get_time()

    ## Read the tello height
    #  @param self The object pointer.
    def get_height(self):
        return self.cmd_if.get_height()

    ## Read the tello temperature
    #  @param self The object pointer.
    def get_temp(self):
        return self.cmd_if.get_temp()

    ## Read the tello attitude
    #  @param self The object pointer.
    def get_attitude(self):
        return self.cmd_if.get_attitude()

     ## Read the tello barometer
    #  @param self The object pointer.
    def get_baro(self):
        return self.cmd_if.get_baro()

    ## Read the tello acceleration
    #  @param self The object pointer.
    def get_acceleration(self):
        return self.cmd_if.get_acceleration()
    
    ## Read the tello time of flight
    #  @param self The object pointer.
    def get_tof(self):
        return self.cmd_if.get_tof()

    ## Read the tello wifi name
    #  @param self The object pointer.
    def get_wifi(self):
        return self.cmd_if.get_wifi()

