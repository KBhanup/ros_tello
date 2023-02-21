import socket
import time
from enum import Enum
import numpy as np
from .model import Model


class State(Enum):
    GROUND = 0
    EMERGENCY = 1
    FLYING = 2
    NOTREADY = 3


## Instantiate a tello Simulator object
#  Only specify a state port if you have a single drone. All drones will broadcast to the local ip 0.0.0.0 on port 8890
class Simulator:

    ## Create the instance of the tello object.
    #  Opens communication to the tello and begins a listening thread.
    #  @param self This object pointer.
    #  @param tello_ip Ip address of the drone on teh network (default: '192.168.10.1')
    #  @param local_port Port to connect on from the local computer (default: 8889)
    #  @param local_ip Ip address of the host computer (default: '0.0.0.0')
    def __init__(self):
        # command interface
        self.state = State.NOTREADY

        A = np.eye(4)
        B = np.eye(4) * .05
        self.model = Model(A, B)
        self.Ts = .05
        self.curr_ctrl = np.zeros((4, 1))

    # Control Commands
    ## Command the tello to take off.
    #  @param self The object pointer.
    def takeoff(self):
        self.state = State.FLYING
        while self.model.z < 1:
            self.model.update(np.array([0, 0, 1, 0]).reshape(-1, 1))
            time.sleep(self.Ts)

    ## Command the Tello to land
    #  @param self The object pointer.
    def land(self):
        while self.model.z > 0:
            self.model.update(np.array([0, 0, -1, 0]).reshape(-1, 1))
            time.sleep(self.Ts)
        self.model.set_state()
        self.state = State.GROUND

    ## Command the Tello to hover.
    #  @param self The object pointer.
    def stop(self):
        self.curr_ctrl = np.zeros((4, 1))

    ## Command the Tello to immediately power off motors.
    #  This is an emergency function (hence the name). The motors will immediately power off whether in flight or on the
    #  ground. If it has velocity, your drone is now a projectile.
    #  @param self The object pointer.
    def emergency(self):
        self.state = State.EMERGENCY
        while self.model.z() > 0:
            self.model.projectile()
        self.model.set_state()

    ## Set the tello velocities
    #  @param self The object pointer.
    #  @param a velocity in x direction
    #  @param b velocity in y direction
    #  @param c velocity in z direction
    #  @param d angular velocity
    def rc_control(self, a, b, c, d):
        if self.state == State.FLYING:
            self.model.control(np.array([a, b, c, d]).reshape((4, 1)))
            if self.model.z() < 0:
                self.state = GROUND
