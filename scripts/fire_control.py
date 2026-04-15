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
            dT_dt = 0

        E = t - T
        dE_dt = 1 - dT_dt

        print(f"newton iteration {i+1}: t = {t}, E={E}, E'={dE_dt}, D = {np.linalg.norm(g - v * t)}")

        t = t - (E / dE_dt)

        if abs(E) < 0.1:
            print(f"t has been found - converged after {i + 1} iterations")
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
    
    alphas = np.linspace(0.2, 1.0, len(virtual_targets)) # light to dark
    
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


v = np.array([-2, 2])
g = np.array([-1.5, 1.5])

projectile_velocity = 2.722

max_iter = 15

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

# build dE_dt CORRECTLY
for t in t_values:
    if np.linalg.norm(g - v * t) != clamp(np.linalg.norm(g - v * t), min_shot_distance, max_shot_distance):
        dE_dt_values.append(1)
        continue

    dE_dt_values.append(1 - dTOF_dt(v, g, t))

Newton(max_iter)

plt.show()
