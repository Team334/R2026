import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

class LookupTable:
    def __init__(self, data: dict):
        """
        Initialize with a dictionary of key to values.
        """
        self.data = data
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
    
    def diff(self):
        thing = LookupTable(self.data)

        thing.values = [
            (self.values[i] - self.values[i - 1]) / (self.keys[i] - self.keys[i - 1]) for i in range(1, len(self.keys))
        ]

        return thing

def TOF(v: np.ndarray, g: np.ndarray, t: float) -> float:
    virtual_goal = g - (v * t)
    distance = np.linalg.norm(virtual_goal)

    return projectile_tof_lookup.get(distance)

def dTOF_dt(v: np.ndarray, g: np.ndarray, t: float) -> float:
    virtual_goal = g - (v * t)
    distance = np.linalg.norm(virtual_goal)

    if distance == 0:
        return 0


    return -np.dot(v, virtual_goal) / (distance * projectile_EPRIME_lookup.get(distance))


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
    axs.plot(t_values, dE_dt_values, label="TOF'(t)", linewidth=2, linestyle='dashed')
    axs.plot(t, TOF(v, g, t), 'ro', markersize=8, label='Fixed-Point Solution')
    axs.set_xlabel('t')
    axs.set_ylabel('TOF / TOF\'')
    axs.set_title('Fixed-Point Method ({} iterations)'.format(i + 1))
    plt.gcf().canvas.manager.set_window_title('Fixed-Point Method')
    axs.grid(True)
    axs.legend()


def Newton(max_iter: int):
    print("\nNewton")

    t = np.linalg.norm(g) / (np.dot(g, v) / np.linalg.norm(g) + projectile_velocity)

    # t = abs(t)

    virtual_targets = []
    t_guesses = []

    for i in range(max_iter):
        # print(t)

        virtual_targets.append(g - (v * t))
        t_guesses.append(t)

        E = t - TOF(v, g, t)
        # dT_dt = dTOF_dt(v, g, t)
        # dE_dt = 1 - dT_dt

        virtual_goal = g - (v * t)
        distance = np.linalg.norm(virtual_goal)

        dE_dt = 1 + (virtual_goal[0] * v[0] + virtual_goal[1] * v[1]) / (projectile_EPRIME_lookup.get(distance) * distance)

        # print(1 - dT_dt == dE_dt)

        # print(f"newton iteration {i+1}: t = {t}, dT/dt = {abs(dT_dt)} E={E}")

        # input()

        t = t - (E / dE_dt)

        if abs(E) < 0.1:
            print("t has been found - converged after {} iterations.".format(i + 1))
            break

    _, ax_vec = plt.subplots(figsize=(6, 6))

    ax_vec.set_xlim(-20, 20)
    ax_vec.set_ylim(-20, 20)

    ax_vec.spines['left'].set_position('center')
    ax_vec.spines['bottom'].set_position('center')
    ax_vec.spines['right'].set_color('none')
    ax_vec.spines['top'].set_color('none')

    ax_vec.set_aspect('equal', adjustable='box')
    ax_vec.grid(True, linestyle=':', linewidth=0.5)
    ax_vec.set_title("Newton Method Virtual Targets")

    ax_vec.plot(g[0], g[1], 'o', color='C0', markersize=8)

    n = len(virtual_targets)
    
    # Build red -> yellow -> green gradient
    colors = []
    for i in range(n):
        t = i / max(n - 1, 1)
        if t <= 0.5:
            # Red to yellow
            colors.append((1.0, 2 * t, 0.0))
        else:
            # Yellow to green
            colors.append((2 * (1 - t), 1.0, 0.0))
    
    # Clamp color values to valid range [0, 1]
    colors = [tuple(np.clip(c, 0, 1) for c in color) for color in colors]

    for i, vt in enumerate(virtual_targets):
        ax_vec.plot(vt[0], vt[1], 'o', color=colors[i], markersize=6)

    # axs = plt.subplots(1, 1, figsize=(6, 4))[1]

    axs.plot(t_values, E_values, label='E(t)')
    axs.plot(t_values, dE_dt_values, label='E\'(t)')

    # for i, tg in enumerate(t_guesses):
    #     axs.plot(tg, tg - TOF(v, g, tg), 'o', color=colors[i], markersize=6)

    # axs.set_xlabel('t')
    # axs.set_ylabel('E / E\'')
    # axs.set_title('Newton Method ({} iterations)'.format(i + 1))
    # plt.gcf().canvas.manager.set_window_title('Newton Method')
    # axs.grid(True)
    # axs.legend()


v = np.array([0, -2])
g = np.array([0, 5])

projectile_velocity = 2.722

max_iter = 1000

projectile_tof_lookup = LookupTable({
    0: 0,
    1.89: 0.955,
    2.665: 1.08,
    3.768: 1.38,
    4.574: 1.53,
    5.252: 1.7
})

projectile_EPRIME_lookup = projectile_tof_lookup.diff()
print(projectile_EPRIME_lookup.get(1.89))

t_values = np.linspace(-20, 20, 100)
E_values = [t - TOF(v, g, t) for t in t_values]
dE_dt_values = [1 - dTOF_dt(v, g, t) for t in t_values]

Newton(max_iter)

plt.show()
