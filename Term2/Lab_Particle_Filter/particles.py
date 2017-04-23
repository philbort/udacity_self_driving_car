from robot import robot
import random
from pprint import pprint

myrobot = robot()
myrobot = myrobot.move(0.1, 5.0)
Z = myrobot.sense()


N = 1000
p = []

# Initialize 1000 particles
for i in range(N):
    x = robot()
    x.set_noise(0.05, 0.05, 5.0)
    p.append(x)
print len(p)

# Move all particles
p2 = []
for i in range(N):
    p2.append(p[i].move(0.1, 5.0))

p = p2

# Importance weight
w = []
for i in range(N):
    w.append(p[i].measurement_prob(Z))

# Resampling
p3 = []
index = int(random.random() * N)
beta = 0.0
mw = max(w)
for i in range(N):
    beta += random.random() * 2.0 * mw
    while beta > w[index]:
        beta -= w[index]
        index = (index + 1) % N
    p3.append(p[index])
p = p3

pprint(p)