#!python3
import numpy as np
import matplotlib.pyplot as plt
import argparse

from KF import KF
import Utilities

parser = argparse.ArgumentParser(description='Filter a trajectory.')
parser.add_argument('input_file', metavar='input_file', type=str, nargs=1,
                    help='a file to filter')
parser.add_argument('--output_file', metavar='output_file', type=str, nargs=1,
                    help='a file to output filtered traj to')
parser.add_argument('--plot', help='plot output',
                    action='store_true')
args = parser.parse_args()
if_path = args.input_file[0]
of_path = None
if args.output_file:
    of_path = args.output_file[0]
plot = False
if args.plot:
    plot = True

Ts = 0.05

traj_orig = np.loadtxt(open(if_path, "rb"), delimiter=",", skiprows=1)

traj = Utilities.tum_to_xyzw(traj_orig)

filt = KF(np.hstack((traj[0, 1:5], 0, 0, 0, 0)).reshape((-1,1)))

traj_filt = np.zeros((len(traj), 9))
traj_filt[0] = np.hstack((traj[0], 0, 0, 0, 0))
for i in range(1, len(traj)):
    filt.predict(traj[i, 0] - traj[i - 1, 0])
    filt.update(traj[i, 1:5].reshape(-1,1))
    traj_filt[i] = np.hstack((traj[i, 0], filt.x.reshape((-1))))

if (plot):
    plt.figure(1)
    plt.title("Trajectory")
    for i in range(4):
        plt.subplot(4, 1, i + 1)
        plt.plot(traj[:, 0], traj[:, i + 1])
        plt.plot(traj_filt[:, 0], traj_filt[:, i + 1],linewidth=1)

    plt.figure(2)
    plt.title("Velocities")
    for i in range(4):
        plt.subplot(4, 1, i + 1)
        plt.plot(traj_filt[:, 0], traj_filt[:, i + 5])
    plt.show()
