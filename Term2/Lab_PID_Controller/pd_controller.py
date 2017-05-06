# -----------
# User Instructions
#
# Implement a PD controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau_p and tau_d so that:
#
# steering = -tau_p * CTE - tau_d * diff_CTE
# where differential crosstrack error (diff_CTE)
# is given by CTE(t) - CTE(t-1)
#
#
# Only modify code at the bottom! Look for the TODO
# ------------
 
from robot import Robot
import numpy as np
import matplotlib.pyplot as plt

def run(robot, tau_p, tau_d, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    prev_cte = robot.y
    for i in range(n):
        cte = robot.y
        steer = -tau_p * cte - tau_d * (cte - prev_cte)
        prev_cte = cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        print(robot)
    return x_trajectory, y_trajectory


if __name__ == '__main__':

    robot = Robot()
    robot.set(0, 1, 0)
    x_trajectory, y_trajectory = run(robot, 0.2, 3.0)
    n = len(x_trajectory)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
    ax1.plot(x_trajectory, y_trajectory, 'g', label='PD controller')
    ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
