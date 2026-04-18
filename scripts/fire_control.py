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

def Newton(max_iter: int, plot: bool):
    t = np.linalg.norm(g) / (np.dot(g, v) / np.linalg.norm(g) + projectile_velocity)

    virtual_targets = []
    t_guesses = []

    for i in range(max_iter):
        virtual_targets.append(g - v * t)
        t_guesses.append(t)

        T = TOF(g, v, t)
        dT_dt = dTOF_dt(v, g, t)

        if np.linalg.norm(g - v * t) != clamp(np.linalg.norm(g - v * t), min_shot_distance, max_shot_distance):
            dT_dt = 0

        E = t - T
        dE_dt = 1 - dT_dt

        if abs(E) < 0.1:
            break

        t = t - (E / dE_dt)

    if plot:
        _, axs_vec = plt.subplots(figsize=(6, 6))

        axs_vec.set_xlim(-5, 5)
        axs_vec.set_ylim(-5, 5)

        axs_vec.spines['left'].set_position('center')
        axs_vec.spines['bottom'].set_position('center')
        axs_vec.spines['right'].set_color('none')
        axs_vec.spines['top'].set_color('none')

        axs_vec.set_aspect('equal', adjustable='box')
        axs_vec.grid(True, linestyle=':', linewidth=0.5)
        axs_vec.set_title("newton method virtual targets")

        axs_vec.plot(g[0], g[1], 'o', color='blue', markersize=8)
        
        alphas = np.linspace(0.2, 1.0, len(virtual_targets)) # LIGHT TO DARK
        
        for i, vt in enumerate(virtual_targets):
            axs_vec.plot(vt[0], vt[1], 'o', color='green', alpha=alphas[i], markersize=6)

        axs = plt.subplots(1, 1, figsize=(6, 4))[1]

        t_values = np.linspace(-5, 5, 50)

        E_values = [t - TOF(v, g, t) for t in t_values]
        dE_dt_values = []

        # build dE_dt values CORRECTLY
        for t in t_values:
            if np.linalg.norm(g - v * t) != clamp(np.linalg.norm(g - v * t), min_shot_distance, max_shot_distance):
                dE_dt_values.append(1)
                continue

            dE_dt_values.append(1 - dTOF_dt(v, g, t))

        axs.plot(t_values, E_values, label='E(t)')
        axs.plot(t_values, dE_dt_values, label='E\'(t)')

        for i, tg in enumerate(t_guesses):
            axs.plot(tg, tg - TOF(v, g, tg), 'o', color='green', alpha=alphas[i], markersize=6)

        axs.set_xlabel('t')
        axs.set_ylabel('E / E\'')
        axs.set_title('newton method ({} iterations)'.format(i + 1))
        axs.grid(True)
        axs.legend()

    return t, i < max_iter

ang = 90
s = 1.5

G = np.array([0.5, 0])
V = np.array([np.cos(np.deg2rad(ang)), np.sin(np.deg2rad(ang))]) * s

g = G
v = V

projectile_velocity = 2.722

max_iter = 10

projectile_tof_lookup = LookupTable({
    2.0: 0.885,
    2.97: 1.05,
    4.26: 1.275,
    5.012: 1.41
})

min_shot_distance = 2.0
max_shot_distance = 5.012

x_values = []
shot_heading_values = []

dshot_heading_dx_values = []

robot_poses = []

for x in np.linspace(-1, 1, 100):
    g = G - v * x

    t, suc = Newton(max_iter, False)

    vg = g - v * t

    shot_heading = np.rad2deg(np.arctan2(vg[1], vg[0]))

    if not suc:
        print("newton's method failed to converge at x = " + str(x))

    dshot_heading_dx = 0

    if len(x_values) > 0:
        dshot_heading_dx = (shot_heading - shot_heading_values[-1]) / (x - x_values[-1])

    dshot_heading_dx_values.append(dshot_heading_dx)

    x_values.append(x)
    shot_heading_values.append(shot_heading)

    robot_poses.append(v * x)

dshot_heading_dx_values[0] = dshot_heading_dx_values[1]

_, axs_vec = plt.subplots(figsize=(6, 6))

axs_vec.set_xlim(-2, 2)
axs_vec.set_ylim(-2, 2)

axs_vec.spines['left'].set_position('center')
axs_vec.spines['bottom'].set_position('center')
axs_vec.spines['right'].set_color('none')
axs_vec.spines['top'].set_color('none')

axs_vec.set_aspect('equal', adjustable='box')
axs_vec.grid(True, linestyle=':', linewidth=0.5)
axs_vec.set_title("Robot Poses")

axs_vec.plot(G[0], G[1], 'o', color='blue', markersize=8)

for robot_pose in robot_poses:
    axs_vec.plot(robot_pose[0], robot_pose[1], 'o', color='green', markersize=4)

axs = plt.subplots(1, 1, figsize=(6, 4))[1]

axs.plot(x_values, shot_heading_values)
axs.plot(x_values, dshot_heading_dx_values)

plt.show()
