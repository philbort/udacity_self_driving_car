from math import pi
from robot import robot

myrobot = robot()
myrobot.set_noise(5.0, 0.1, 5.0)
myrobot.set(30, 50, pi/2)
myrobot.move(-pi/2, 15.0)
myrobot.sense()
print myrobot
myrobot.move(pi/2,10.0)
myrobot.sense()
print myrobot