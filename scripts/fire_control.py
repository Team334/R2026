import numpy as np
import matplotlib.pyplot as plt

projectile_velocity = 11

def TOF(v: np.ndarray, g: np.ndarray, t: float) -> float:
    offset_g = g - (v * t)
    distance = np.linalg.norm(offset_g)

    return distance / projectile_velocity

def FPI(max_iter: int):
    prev_t = 0
    t = 0
    dt_history = []

    for i in range(max_iter):
        new_t = TOF(v, g, t)

        dt = (new_t - t) / (t - prev_t) if t != prev_t else 0 # prevent division by zero

        print("iteration {}: t = {}, dt = {}".format(i + 1, t, abs(dt)))

        prev_t = t
        t = new_t

        dt_history.append(abs(t - prev_t))

        if abs(dt) > c:
            print()
            print("dt is too high for fixed-point iteration - failed to converge after {} iterations.".format(i + 1))
            break

        if abs(t - prev_t) < 0.01:
            print()
            print("t has been found - converged after {} iterations.".format(i + 1))
            break

    axs = plt.subplots(1, 1, figsize=(6, 4))[1]

    # Calculate numerical derivative of TOF
    tof_derivative = np.gradient(tof_values, t_values)

    axs.plot(t_values, tof_values, label='TOF(t)')
    axs.plot(t_values, t_values, label='y = t')
    axs.plot(t, TOF(v, g, t), 'ro', markersize=8, label='Fixed-Point Solution')
    axs.plot(t_values, tof_derivative, label="TOF'(t)", linewidth=2, linestyle='dashed')
    axs.set_xlabel('t')
    axs.set_ylabel('TOF / TOF\'')
    axs.set_title('Fixed-Point Method')
    axs.grid(True)
    axs.legend()


def Newton(max_iter: int):
    t = 0
    dt_history = []

    for i in range(max_iter):
        displacement = g - (v * t)
        distance = np.linalg.norm(displacement)

        E = t - (distance / projectile_velocity)
        dE = 1 + (np.dot(displacement, v) /  (distance * projectile_velocity))

        new_t = t - (E / dE)

        print(f"Newton iteration {i+1}: t = {t}")

        dt_history.append(abs(new_t - t))

        if abs(new_t - t) < 0.01:
            print()
            print("t has been found - converged after {} iterations.".format(i + 1))
            t = new_t
            break

        t = new_t

    axs = plt.subplots(1, 1, figsize=(6, 4))[1]

    # Calculate numerical derivative of TOF
    tof_derivative = np.gradient(tof_values, t_values)

    axs.plot(t_values, tof_values, label='TOF(t)')
    axs.plot(t_values, t_values, label='y = t')
    axs.plot(t, TOF(v, g, t), 'go', markersize=8, label='Newton Solution')
    axs.plot(t_values, tof_derivative, label="TOF'(t)", linewidth=2, linestyle='dashed')
    axs.set_xlabel('t')
    axs.set_ylabel('TOF / TOF\'')
    axs.set_title('Newton Method')
    axs.grid(True)
    axs.legend()


v = np.array([10, 10])
g = np.array([0, 500])

speed = np.linalg.norm(v)

t_values = np.linspace(0, 40, 100)
tof_values = [TOF(v, g, t) for t in t_values]

c = 0.8

FPI(25)
Newton(10)

plt.show()
