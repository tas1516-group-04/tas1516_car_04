import numpy as np
import matplotlib.pyplot as plt
import time

def calcDistance(x, y):
    return np.sqrt(x**2 + y**2)

z = np.arange(np.pi/2, 0, -0.05);
y = +np.sin(z)-1
x = 10*np.cos(z)

plt.plot(x, y, 'bo')

start_time = time.time()

vx_prev = x[1] - x[0]
vy_prev = y[1] - y[0]
dist_prev = calcDistance(vx_prev, vy_prev)

acc_angle = 0
acc_dist = 0

for i in range(1, len(z)):
    vx = x[i] - x[i-1]
    vy = y[i] - y[i-1]
    dist = calcDistance(vx, vy)
    print((vx * vx_prev + vy * vy_prev) / (dist * dist_prev))
    angle = np.arccos(np.clip((vx * vx_prev + vy * vy_prev) / (dist * dist_prev), 0.0, 1.0))*180/np.pi
    print("angle: ", angle)

    acc_angle += np.abs(angle)
    acc_dist += dist

    if acc_dist > 3.0:
        break

    dist_prev = dist
    vx_prev = vx
    vy_prev = vy

print(acc_angle)
print(acc_dist)

print("--- %s seconds ---" % (time.time() - start_time))
plt.show()