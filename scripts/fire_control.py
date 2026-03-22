import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

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

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

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


# def FPI(max_iter: int):
#     print("\nFPI")

#     prev_t = 0
#     t = 0

#     for i in range(max_iter):
#         new_t = TOF(v, g, t)
#         dT_dt = dTOF_dt(v, g, t)

#         print("fpi iteration {}: t = {}, dT/dt = {}".format(i + 1, t, abs(dT_dt)))

#         prev_t = t
#         t = new_t

#         if abs(t - prev_t) < 0.01:
#             print("t has been found - converged after {} iterations.".format(i + 1))
#             break

#     axs = plt.subplots(1, 1, figsize=(6, 4))[1]

#     axs.plot(t_values, tof_values, label='TOF(t)')
#     axs.plot(t_values, t_values, label='y = t')
#     axs.plot(t_values, dE_dt_values, label="TOF'(t)", linewidth=2, linestyle='dashed')
#     axs.plot(t, TOF(v, g, t), 'ro', markersize=8, label='Fixed-Point Solution')
#     axs.set_xlabel('t')
#     axs.set_ylabel('TOF / TOF\'')
#     axs.set_title('Fixed-Point Method ({} iterations)'.format(i + 1))
#     plt.gcf().canvas.manager.set_window_title('Fixed-Point Method')
#     axs.grid(True)
#     axs.legend()


def Newton(max_iter: int):
    print("\nNewton")

    t = np.linalg.norm(g) / (np.dot(g, v) / np.linalg.norm(g) + projectile_velocity)

    virtual_targets = []
    t_guesses = []

    for i in range(max_iter):
        virtual_targets.append(g - (v * t))
        t_guesses.append(t)

        T = TOF(g, v, t)
        dT_dt = dTOF_dt(v, g, t)

        if np.linalg.norm(g - v * t) != clamp(np.linalg.norm(g - v * t), min_shot_distance, max_shot_distance):
            print(f"out of bounds iteration {i+1}, D {np.linalg.norm(g - v * t) != clamp(np.linalg.norm(g - v * t), min_shot_distance, max_shot_distance)}")
            dT_dt = 0

        E = t - T
        dE_dt = 1 - dT_dt

        print(f"newton iteration {i+1}: t = {t}, E={E}, E'={dE_dt}")

        # input()

        t = t - (E / dE_dt)

        if abs(E) < 0.1:
            print("t has been found - converged after {} iterations.".format(i + 1))
            break

    # plot everything
    _, axs_vec = plt.subplots(figsize=(6, 6))

    axs_vec.set_xlim(-20, 20)
    axs_vec.set_ylim(-20, 20)

    axs_vec.spines['left'].set_position('center')
    axs_vec.spines['bottom'].set_position('center')
    axs_vec.spines['right'].set_color('none')
    axs_vec.spines['top'].set_color('none')

    axs_vec.set_aspect('equal', adjustable='box')
    axs_vec.grid(True, linestyle=':', linewidth=0.5)
    axs_vec.set_title("Newton Method Virtual Targets")

    axs_vec.plot(g[0], g[1], 'o', color='blue', markersize=8)

    n = len(virtual_targets)
    
    alphas = np.linspace(0.01, 1.0, n)
    
    for i, vt in enumerate(virtual_targets):
        axs_vec.plot(vt[0], vt[1], 'o', color='green', alpha=alphas[i], markersize=6)

    axs = plt.subplots(1, 1, figsize=(6, 4))[1]

    axs.plot(t_values, E_values, label='E(t)')
    axs.plot(t_values, dE_dt_values, label='E\'(t)')

    for i, tg in enumerate(t_guesses):
        axs.plot(tg, tg - TOF(v, g, tg), 'o', color='green', alpha=alphas[i], markersize=6)

    axs.set_xlabel('t')
    axs.set_ylabel('E / E\'')
    axs.set_title('Newton Method ({} iterations)'.format(i + 1))
    axs.grid(True)
    axs.legend()


v = np.array([0, -2])
g = np.array([0, 5])

projectile_velocity = 2.722

max_iter = 100

projectile_tof_lookup = LookupTable({
    1.89: 0.955,
    2.665: 1.08,
    3.768: 1.38,
    4.574: 1.53,
    5.252: 1.51
})

min_shot_distance = 1.89
max_shot_distance = 5.252

t_values = np.linspace(-20, 20, 100)

E_values = [t - TOF(v, g, t) for t in t_values]
dE_dt_values = []
dE_dt_values_wrong = [1 - dTOF_dt(v, g, t) for t in t_values]

for t in t_values:
    if np.linalg.norm(g - v * t) != clamp(np.linalg.norm(g - v * t), min_shot_distance, max_shot_distance):
        dE_dt_values.append(1)
        continue

    dE_dt_values.append(1 - dTOF_dt(v, g, t))

# Newton(max_iter)

fig, ax = plt.subplots(figsize=(8, 6))
ax.plot(t_values, E_values, label='E(t)', linewidth=2)
ax.plot(t_values, dE_dt_values, label="E'(t)", linewidth=2)
ax.plot(t_values, dE_dt_values_wrong, label="E'(t) wrong", linewidth=2)
ax.set_xlabel('t')
ax.set_ylabel('Value')
ax.set_title('E(t) and E\'(t)')
ax.grid(True) 
ax.legend()

plt.show()
