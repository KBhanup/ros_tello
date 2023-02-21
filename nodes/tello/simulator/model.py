import numpy as np
from math import pi, sin, cos


class Model():
    @staticmethod
    def R(th):
        return np.array([[cos(th), -sin(th)], [sin(th), cos(th)]])

    def __init__(self, A: np.array, B: np.array):
        self.pos = np.zeros((4, 1))
        self.vel = np.zeros((4, 1))
        self.A = A
        self.B = B
        self.Ts = .05

    def control(self, ctrl):
        dp = self.B.dot(ctrl)
        wp = dp
        wp[0:2] = R(-self.pos[3]).dot(wp[0:2])

        self.pos = self.pos + wp
        self.vel = wp / Ts

        # check the yaw is in (-pi,pi]
        self.pos[3] = self.pos[3] % (2 * pi)
        if self.pos[3] > pi:
            self.pos[3] = self.pos[3] - 2 * pi

    def projectile(self):
        self.pos = self.pos + self.vel * Ts + np.array([0, 0, -9.8, 0]).reshape((4, 1)) * Ts ** 2
        self.vel = self.vel + p.array([0, 0, -9.8, 0]).reshape((4, 1)) * Ts

    def x(self):
        return self.pos[0]

    def y(self):
        return self.pos[1]

    def z(self):
        return self.pos[2]

    def w(self):
        return self.pos[3]

    def set_state(self, pos=np.zeros((4, 1)), vel=np.zeros((4, 1))):
        self.pos = pos
        self.vel = vel
