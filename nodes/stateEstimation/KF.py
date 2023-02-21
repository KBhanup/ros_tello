import numpy as np

from math import pi


class KF():


    def __init__(self, x_0: list):
        pNoise = 1e-2
        wNoise = 1e-2
        pdNoise = 1e1
        wdNoise = 1e1
        self.W = np.diag([pNoise, pNoise, pNoise, wNoise, pdNoise, pdNoise, pdNoise, wdNoise])

        puNoise = 1e-1
        wuNoise = 1e-1
        self.V = np.diag([puNoise, puNoise, puNoise, wuNoise])

        assert(x_0.shape == (8,1))
        self.x = x_0
        self.P = np.ones(len(x_0))
        self.H = np.eye(4,8)

    def update(self, obs):
        y = obs - self.H.dot(self.x)
        y[3] = y[3] % (2 * pi)

        if y[3] > pi:
            y[3] = y[3] - (2 * pi)

        S = self.H.dot(self.P).dot(self.H.transpose()) + self.V
        K = self.P.dot(self.H.transpose()).dot(np.linalg.inv(S))

        self.x = self.x + K.dot(y)

        self.P = (np.eye(8) - K.dot(self.H)).dot(self.P)

        self.x[3] = self.x[3] % (2 * pi)

        if self.x[3] > pi:
            self.x[3] = self.x[3] - (2 * pi)

    def predict(self, dt):
        F = self.F(dt)
        self.x = F.dot(self.x)
        self.P = F.dot(self.P).dot(F.transpose()) + self.W
        if self.x[3] > pi:
            self.x[3] = self.x[3] - (2 * pi)
        elif self.x[3] < pi:
            self.x[3] = self.x[3] + (2 * pi)

    @staticmethod
    def F(u: np.array):
        return np.array([
            [1, 0, 0, 0, u, 0, 0, 0],
            [0, 1, 0, 0, 0, u, 0, 0],
            [0, 0, 1, 0, 0, 0, u, 0],
            [0, 0, 0, 1, 0, 0, 0, u],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ])
