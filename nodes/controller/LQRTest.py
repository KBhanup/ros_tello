from LQR import LQR
import numpy as np

lqr = LQR("../../scripts/controller.csv")

goal = np.array([.1, 0, 0, 0, 0, 0, 0, 0]).reshape((-1, 1))
pose = np.array([.1, 0, 0, 0, 0, 0, 0, 0]).reshape(-1, 1)
control = lqr.get_ctrl(goal,pose)
print(control)
