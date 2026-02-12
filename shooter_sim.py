import numpy as np
import matplotlib.pyplot as plt

# Constants
projectile_speed = 5.0  # m/s, can be changed

# TOF function
def TOF(x, v, g, t):
    """
    Compute time-of-flight for a 2D vector target.

    x: 2-element iterable (initial relative position)
    v: 2-element iterable (relative velocity)
    t: scalar or 1D array of time guesses

    Returns: scalar or 1D numpy array of TOF values
    """
    x = np.asarray(x, dtype=float).ravel()[:2]
    v = np.asarray(v, dtype=float).ravel()[:2]
    t_arr = np.asarray(t)

    if t_arr.ndim == 0:
        moved_target = g - v * float(t_arr)
        distance = np.linalg.norm(moved_target - x)
        return distance / projectile_speed
    else:
        return np.array([TOF(x, v, g, tt) for tt in t_arr])

# Example vectors
x = np.array([0, 0])  # initial relative position
v = np.array([1.0, 0.5])  # robot velocity
g = np.array([10, 0])  # target position

# Time guesses
t_values = np.linspace(0, 5, 200)  # 0 to 5 seconds

# Compute TOF
tof_values = TOF(x, v, g, t_values)

t = 0.0
tol = 0.1
max_iter = 100

for i in range(max_iter):
    tof_t = float(TOF(x, v, g, t))
    print(f"iter {i}: t={t:.8f}, TOF(t)={tof_t:.8f}")
    if abs(tof_t - t) < tol:
        print(f"converged after {i} iterations: t={tof_t:.8f}")
        break
    t = tof_t
else:
    print(f"did not converge after {max_iter} iterations, last t={t:.8f}")

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
