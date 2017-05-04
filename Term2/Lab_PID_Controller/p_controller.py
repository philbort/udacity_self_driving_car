# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The desired trajectory for the 
# robot is the x-axis. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau * crosstrack_error
#
# You'll only need to modify the `run` function at the bottom.
# ------------
 
from robot import Robot
import numpy as np
import matplotlib.pyplot as plt

def run(robot, tau, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    for i in range(n):
        cte = robot.y
        steer = -tau * cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        print(robot)
    return x_trajectory, y_trajectory

if __name__ == '__main__':

    robot = Robot()
    robot.set(0, 1, 0)
    x_trajectory, y_trajectory = run(robot, 0.1)
    n = len(x_trajectory)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
    ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
    ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
