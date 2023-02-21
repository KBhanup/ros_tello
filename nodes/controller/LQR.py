import numpy as np


def rot_mat_2(th: float):
    return np.array([[math.cos(th), -math.sin(th)],
                     [math.sin(th), math.cos(th)]])


class LQR():
    def __init__(self, ctrl_csv):
        self.ctrl = np.genfromtxt(ctrl_csv, delimiter=",")

    def get_ctrl(self, goal_W, pose_W):
        err = goal_W - pose_W
        R = rot_mat_2(-pose_W[3])
        err[0:2] = R.dot(err[0:2])
        return np.clip(self.ctrl.dot(err), -1, 1)
