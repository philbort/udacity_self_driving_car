# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
#
# where the integrated crosstrack error (int_CTE) is
# the sum of all the previous crosstrack errors.
# This term works to cancel out steering drift.
#
# Only modify code at the bottom! Look for the TODO.
# ------------

from robot import Robot
import numpy as np
import matplotlib.pyplot as plt

def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    prev_cte = robot.y
    sum_cte = 0
    for i in range(n):
        cte = robot.y
        sum_cte += cte
        steer = - tau_p * cte \
                - tau_d * (cte - prev_cte) \
                - tau_i * sum_cte
        prev_cte = cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        print(robot)
    return x_trajectory, y_trajectory

if __name__ == '__main__':

    robot = Robot()
    robot.set(0, 1, 0)

    x_trajectory, y_trajectory = run(robot, 0.2, 3.0, 0.004)
    n = len(x_trajectory)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,8))
    ax1.plot(x_trajectory, y_trajectory, 'g', label='PID controller')
    ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
