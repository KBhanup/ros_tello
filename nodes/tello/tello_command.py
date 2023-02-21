import socket
import threading
import time
import cv2
from .stats import Stats


## Instantiate a tello drone object
# Only specify a state port if you have a single drone. All drones will broadcast to the local ip 0.0.0.0 on port 8890
class TelloCommand:
    MAX_TIME_OUT = 2

    ## Constructor
    #  @param self The object pointer.
    #  @param tello_ip Ip of the drone on the network (default: '19.168.10.1').
    #  @param local_port Port on the local computer to listen for responses on (default: 8889).
    #  @param local_ip Ip of the host computer (default: '').
    def __init__(self, tello_ip='19.168.10.1', local_port=8889, local_ip=''):
        # Opening local UDP port on 8889 for Tello communication
        self.local_ip = local_ip
        self.local_port = local_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print(f"Command Binding to {self.local_ip}:{self.local_port}")
        self.socket.bind((self.local_ip, self.local_port))

        # Setting Tello ip and port info
        self.tello_address = (tello_ip, 8889)
        self.log = []

        # Intializing response thread
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    ## Send a command to the tello and await a reponse
    #  @param self The object pointer.
    #  @param command command to send.
    #  @param wait Boolean flag to wait for response (default: True).
    def send_command(self, command, wait=True):
        # New log entry created for the outbound command
        self.log.append(Stats(command, len(self.log)))

        # Sending command to Tello
        self.socket.sendto(command.encode('utf-8'), self.tello_address)

        start = time.time()
        while not self.log[-1].got_response() and wait:  # Runs while no repsonse has been received in log
            now = time.time()
            difference = now - start
            if difference > self.MAX_TIME_OUT:
                print('Connection timed out!')
                break

    ## Thread to receive responses
    #  This function should only be called internally
    #  @param self The object pointer.
    def _receive_thread(self):
        while True:
            # Checking for Tello response, throws socket error
            try:
                self.response, ip = self.socket.recvfrom(1024)
                self.log[-1].add_response(self.response)
            except socket.error as exc:
                print('Socket error: {}'.format(exc))

    ## Delete and Close the connection to the socket
    #  @param self The object pointer.
    def __del__(self):
        self.close()

    ## Close the connection to the socket
    #  @param self The object pointer.
    def close(self):
        self.emergency()
        self.socket.close()

    # Controll Commands

    ## Send a command to the tello
    #  @param self The object pointer.
    #  @param wait Boolean flag whether to wait on a response (default: True).
    def command(self, wait=True):
        if (wait):
            return self.send_command('command')
        self.send_command('command', False)

    ## Connect the Tello to an ap.
    #  This may be called to connect to a router for use with the multi_router modality. The tello will restart after receiving this command.
    #  @param self The object pointer.
    #  @param ap The name (SSID) of the AP.
    #  @param pass The password for the AP.
    def ap(self, ap, passwd):
        return self.send_command('ap {} {}'.format(ap, passwd))

    ## Command the tello to take off.
    #  @param self The object pointer.
    def takeoff(self, wait=False):
        self.send_command('takeoff', False)

    ## Command the Tello to land
    #  @param self The object pointer.
    def land(self):
        self.send_command('land', False)

    ## Command the Tello to hover.
    #  @param self The object pointer.
    def stop(self):
        self.send_command('stop', False)

    ## Command the Tello to immediately power off motors.
    #  This is an emergency function (hence the name). The motors will immediately power off whether in flight or on the ground. If it has velocity, your drone is now a projectile.
    #  @param self The object pointer.
    def emergency(self):
        self.send_command('emergency', False)

    ## Command the Tello to begin streaming data and video. 
    #  @param self The object pointer.
    def streamon(self):
        self.send_command('streamon', False)

    ## Command the Tello to stop streaming data and video. 
    #  @param self The object pointer.
    def streamoff(self):
        self.send_command('streamoff', False)

    # Movement Commands

    ## Command the Tello to move up.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move up [20,500]
    def up(self, dist):
        self.send_command('up {}'.format(dist))
        return self.log[-1].get_response()

    ## Command the Tello to move down.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move down [20,500]
    def down(self, dist):
        self.send_command('down {}'.format(dist))
        return self.log[-1].get_response()

    ## Command the Tello to move left.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move left [20,500]
    def left(self, dist):
        self.send_command('left {}'.format(dist))
        return self.log[-1].get_response()

    ## Command the Tello to move right.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move right [20,500]
    def right(self, dist):
        self.send_command('right {}'.format(dist))
        return self.log[-1].get_response()

    ## Command the Tello to move forward.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move forward [20,500]
    def forward(self, dist):
        self.send_command('forward {}'.format(dist))
        return self.log[-1].get_response()

    ## Command the Tello to move back.
    #  @param self The object pointer.
    #  @param dist Distance in cm to move back [20,500]
    def back(self, dist):
        self.send_command('back {}'.format(dist))
        return self.log[-1].get_response()

    ## Command the Tello to rotate clockwise.
    #  @param self The object pointer.
    #  @param dist Distance in deg to rotate clockwise [1,360]
    def cw(self, degr):
        self.send_command('cw {}'.format(degr))
        return self.log[-1].get_response()

    ## Command the Tello to rotate counter-clockwise.
    #  @param self The object pointer.
    #  @param dist Distance in deg to rotate counter-clockwise [1,360]
    def ccw(self, degr):
        self.send_command('ccw {}'.format(degr))
        return self.log[-1].get_response()

    ## Command the Tello to flip.
    #  @param self The object pointer.
    #  @param direc Direction to flip {"l", "r", "f", "b"}.
    def flip(self, direc):
        self.send_command('flip {}'.format(direc))
        return self.log[-1].get_response()

    ## Command the Tello to go a distance at a speed.
    #  @param self The object pointer.
    #  @param x Distance in cm to move forward [-500,500]
    #  @param y Distance in cm to move left [-500,500]
    #  @param z Distance in cm to move up [-500,500]
    #  @param speed Speed at which to travel.
    def go(self, x, y, z, speed):
        self.send_command('go {} {} {} {}'.format(x, y, z, speed))
        return self.log[-1].get_response()

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
        self.send_command('curve {} {} {} {} {} {} {}'.format(x1, y1, z1, x2, y2, z2, speed))
        return self.log[-1].get_response()

    # Set Commands

    ## Set the tello speed
    #  @param self The object pointer.
    #  @param speed Speed to set
    def set_speed(self, speed):
        self.send_command('speed {}'.format(speed))
        return self.log[-1].get_response()

    ## Set the tello velocities
    #  @param self The object pointer.
    #  @param a velocity in x direction
    #  @param b velocity in y direction
    #  @param c velocity in z direction
    #  @param d angular velocity
    def rc_control(self, a, b, c, d):
        clip = lambda x: (x if x <= 100 else 100) if x >= -100 else -100
        self.send_command('rc {} {} {} {}'.format(clip(-b), clip(a), clip(c), clip(-d)), False)

    ## Set the tello wifi name and password
    #  @param self The object pointer.
    #  @param ssid name for the network
    #  @param passwrd password for the network
    def set_wifi(self, ssid, passwrd):
        self.send_command('wifi {} {}'.format(ssid, passwrd))
        return self.log[-1].get_response()

    # Read Commands

    ## Read the tello speed
    #  @param self The object pointer.
    def get_speed(self):
        self.send_command('speed?', True)
        return self.log[-1].get_response()

    ## Read the tello battery
    #  @param self The object pointer.
    def get_battery(self):
        self.send_command('battery?', True)
        return self.log[-1].get_response()

    ## Read the tello time
    #  @param self The object pointer.
    def get_time(self):
        self.send_command('time?', True)
        return self.log[-1].get_response()

    ## Read the tello height
    #  @param self The object pointer.
    def get_height(self):
        self.send_command('height?', True)
        return self.log[-1].get_response()

    ## Read the tello temperature
    #  @param self The object pointer.
    def get_temp(self):
        self.send_command('temp?', True)
        return self.log[-1].get_response()

    ## Read the tello attitude
    #  @param self The object pointer.
    def get_attitude(self):
        self.send_command('attitude?', True)
        return self.log[-1].get_response()

    ## Read the tello barometer
    #  @param self The object pointer.
    def get_baro(self):
        self.send_command('baro?', True)
        return self.log[-1].get_response()

    ## Read the tello acceleration
    #  @param self The object pointer.
    def get_acceleration(self):
        self.send_command('acceleration?', True)
        return self.log[-1].get_response()

    ## Read the tello time of flight
    #  @param self The object pointer.
    def get_tof(self):
        self.send_command('tof?', True)
        return self.log[-1].get_response()

    ## Read the tello wifi name
    #  @param self The object pointer.
    def get_wifi(self):
        self.send_command('wifi?', True)
        return self.log[-1].get_response()
