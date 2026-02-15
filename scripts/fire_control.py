import numpy as np
import matplotlib.pyplot as plt

projectile_velocity = 10

def TOF(v: np.ndarray, g: np.ndarray, t: float) -> float:
    offset_g = g - (v * t)
    distance = np.linalg.norm(offset_g)

    return distance / projectile_velocity

v = np.array([12, 12])
g = np.array([0, 10])

speed = np.linalg.norm(v)

t_values = np.linspace(0, 5, 100)
tof_values = [TOF(v, g, t) for t in t_values]

max_iter = 10
prev_t = 0
t = 0
c = 0.8

# fixed-point iteration method
for i in range(max_iter):
    new_t = TOF(v, g, t)

    dt = (new_t - t) / (t - prev_t) if t != prev_t else 0 # prevent division by zero

    print("iteration {}: t = {}, dt = {}".format(i + 1, t, abs(dt)))

    prev_t = t
    t = new_t

    if abs(dt) > c:
        print()
        print("dt is too high for fixed-point iteration - failed to converge after {} iterations.".format(i + 1))
        break

    if abs(t - prev_t) < 0.01:
        print()
        print("t has been found - converged after {} iterations.".format(i + 1))
        break

plt.plot(t, TOF(v, g, t), 'ro', markersize=8, label='Solution')
plt.plot(t_values, tof_values, label='TOF')
plt.plot(t_values, t_values, label='y=t')

plt.xlabel('t')
plt.ylabel('TOF')
plt.title('TOF vs t')

plt.grid(True)
plt.legend()

plt.show()
