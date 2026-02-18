import numpy as np
import matplotlib.pyplot as plt

def TOF(v: np.ndarray, g: np.ndarray, t: float) -> float:
    offset_g = g - (v * t)
    distance = np.linalg.norm(offset_g)

    return distance / projectile_velocity

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

    global projectile_velocity

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
# g = <0, 500>
# projectile_velocity = 30

# bad scenario
# v = <0, 28>
# g = <0, 500>
# projectile_velocity = 30

v = np.array([0, 4])
g = np.array([0, 500])

projectile_velocity = 30

max_iter = 200

t_values = np.linspace(0, 40, 100)
tof_values = [TOF(v, g, t) for t in t_values]
dtof_dt_values = [dTOF_dt(v, g, t) for t in t_values]

FPI(max_iter)
Newton(max_iter)

plt.show()
