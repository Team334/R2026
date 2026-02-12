import numpy as np
import matplotlib.pyplot as plt

# Constants
projectile_speed = 5.0  # m/s, can be changed

# TOF function (expects scalar t)
def TOF(x, v, g, t):
    """
    Compute time-of-flight for a 2D vector target.

    x: 2-element iterable (initial relative position)
    v: 2-element iterable (relative velocity)
    g: 2-element iterable (target position)
    t: scalar time guess

    Returns: scalar TOF value
    """
    x = np.asarray(x, dtype=float).ravel()[:2]
    v = np.asarray(v, dtype=float).ravel()[:2]
    g = np.asarray(g, dtype=float).ravel()[:2]

    moved_target = g - v * float(t)
    distance = np.linalg.norm(moved_target - x)

    return distance / projectile_speed

# Example vectors
x = np.array([0, 0])  # initial relative position
v = np.array([1.0, -2])  # robot velocity
g = np.array([10, 0])  # target position

# Time guesses
t_values = np.linspace(0, 5, 200)  # 0 to 5 seconds

# Compute TOF by calling TOF for each scalar t in the linspace
tof_values = np.array([TOF(x, v, g, tt) for tt in t_values])

# Plot y = t
plt.figure(figsize=(6, 6))
plt.plot(t_values, t_values, label='y = t', color='blue')
plt.plot(t_values, tof_values, label='TOF', color='red')
plt.xlabel('t')
plt.ylabel('y')
plt.title('t vs TOF')
plt.legend()
plt.grid(True)
plt.show()
