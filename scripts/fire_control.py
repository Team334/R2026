import numpy as np
import matplotlib.pyplot as plt
import random

class LookupTable:
    def __init__(self, data: dict):
        """
        Initialize with a dictionary of key to values.
        """
        self.keys = sorted(data.keys())
        self.values = [data[k] for k in self.keys]
    
    def get(self, key: float) -> float:
        """
        Get value at key using linear interpolation.
        Returns boundary values if key is outside range.
        """
        if key <= self.keys[0]:
            return self.values[0]
        if key >= self.keys[-1]:
            return self.values[-1]
        
        # Find the two points to interpolate between
        for i in range(len(self.keys) - 1):
            if self.keys[i] <= key <= self.keys[i + 1]:
                x0, x1 = self.keys[i], self.keys[i + 1]
                y0, y1 = self.values[i], self.values[i + 1]
                # Linear interpolation
                return y0 + (key - x0) * (y1 - y0) / (x1 - x0)
        
        return self.values[-1]

def TOF(v: np.ndarray, g: np.ndarray, t: float) -> float:
    virtual_goal = g - (v * t)
    distance = np.linalg.norm(virtual_goal)

    return projectile_tof_lookup.get(distance)

def dTOF_dt(v: np.ndarray, g: np.ndarray, t: float) -> float:
    virtual_goal = g - (v * t)
    distance = np.linalg.norm(virtual_goal)

    if distance == 0:
        return 0

    return -np.dot(v, virtual_goal) / (distance * projectile_velocity)


def FPI(max_iter: int):
    print("\nFPI")

    prev_t = 0
    t = 0

    for i in range(max_iter):
        new_t = TOF(v, g, t)
        dT_dt = dTOF_dt(v, g, t)

        print("fpi iteration {}: t = {}, dT/dt = {}".format(i + 1, t, abs(dT_dt)))

        prev_t = t
        t = new_t

        if abs(t - prev_t) < 0.01:
            print("t has been found - converged after {} iterations.".format(i + 1))
            break

    axs = plt.subplots(1, 1, figsize=(6, 4))[1]

    axs.plot(t_values, tof_values, label='TOF(t)')
    axs.plot(t_values, t_values, label='y = t')
    axs.plot(t_values, dtof_dt_values, label="TOF'(t)", linewidth=2, linestyle='dashed')
    axs.plot(t, TOF(v, g, t), 'ro', markersize=8, label='Fixed-Point Solution')
    axs.set_xlabel('t')
    axs.set_ylabel('TOF / TOF\'')
    axs.set_title('Fixed-Point Method ({} iterations)'.format(i + 1))
    plt.gcf().canvas.manager.set_window_title('Fixed-Point Method')
    axs.grid(True)
    axs.legend()


def Newton(max_iter: int):
    print("\nNewton")

    t = 0

    for i in range(max_iter):
        E = t - TOF(v, g, t)
        dT_dt = dTOF_dt(v, g, t)
        dE_dt = 1 - dT_dt

        print("newton iteration {}: t = {}, dT/dt = {}".format(i + 1, t, abs(dT_dt)))

        t = t - (E / dE_dt)

        if abs(E) < 0.01:
            print("t has been found - converged after {} iterations.".format(i + 1))
            break

    axs = plt.subplots(1, 1, figsize=(6, 4))[1]

    axs.plot(t_values, tof_values, label='TOF(t)')
    axs.plot(t_values, t_values, label='y = t')
    axs.plot(t_values, dtof_dt_values, label="TOF'(t)", linewidth=2, linestyle='dashed')
    axs.plot(t, TOF(v, g, t), 'go', markersize=8, label='Newton Solution')
    axs.set_xlabel('t')
    axs.set_ylabel('TOF / TOF\'')
    axs.set_title('Newton Method ({} iterations)'.format(i + 1))
    plt.gcf().canvas.manager.set_window_title('Newton Method')
    axs.grid(True)
    axs.legend()


# good scenario
# v = <0, 4>
# g = <0, 10>
# projectile_velocity = 30

# bad scenario
# v = <0, 28>
# g = <0, 10>
# projectile_velocity = 30

v = np.array([5, 25])
g = np.array([0, 15])

projectile_velocity = 30

max_iter = 200

projectile_tof_lookup = LookupTable({
    i: i / (projectile_velocity * random.uniform(0.8, 1.2))
    for i in range(1, 21)  # 1 through 20 inclusive
})

t_values = np.linspace(0, 20, 100)
tof_values = [TOF(v, g, t) for t in t_values]
dtof_dt_values = [dTOF_dt(v, g, t) for t in t_values]

FPI(max_iter)
Newton(max_iter)

plt.show()
