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


def fixed_point_solver(x, v, g, t_initial, max_iterations=100, tolerance=1e-6):
    """
    Solve for TOF using fixed-point iteration: t_next = TOF(t_current)
    
    Args:
        x: robot position (2D)
        v: robot velocity (2D)
        g: target position (2D)
        t_initial: initial time guess
        max_iterations: maximum number of iterations
        tolerance: convergence threshold
    
    Returns:
        (t_solution, iterations, converged, history)
    """
    x = np.asarray(x, dtype=float).ravel()[:2]
    v = np.asarray(v, dtype=float).ravel()[:2]
    g = np.asarray(g, dtype=float).ravel()[:2]
    
    t = float(t_initial)
    history = [t]
    
    for i in range(max_iterations):
        t_next = TOF(x, v, g, t)
        history.append(t_next)
        
        if abs(t_next - t) < tolerance:
            return t_next, i + 1, True, history
        
        t = t_next
    
    return t, max_iterations, False, history


def newton_method_solver(x, v, g, t_initial, max_iterations=20, tolerance=1e-6):
    """
    Solve for TOF using Newton's method
    
    Newton's method: t_next = t - E(t)/E'(t)
    where E(t) = t - TOF(t)
    and E'(t) = 1 + (d·v)/(v_p·D)
    
    Args:
        x: robot position (2D)
        v: robot velocity (2D)
        g: target position (2D)
        t_initial: initial time guess
        max_iterations: maximum number of iterations
        tolerance: convergence threshold
    
    Returns:
        (t_solution, iterations, converged, history)
    """
    x = np.asarray(x, dtype=float).ravel()[:2]
    v = np.asarray(v, dtype=float).ravel()[:2]
    g = np.asarray(g, dtype=float).ravel()[:2]
    
    t = float(t_initial)
    history = [t]
    
    for i in range(max_iterations):
        # Calculate virtual target displacement at current t
        d = (g - x) - v * t  # displacement to virtual target
        D = np.linalg.norm(d)  # distance to virtual target
        
        # Error function: E(t) = t - TOF(t) = t - D/v_p
        E = t - D / projectile_speed
        
        # Check convergence
        if abs(E) < tolerance:
            return t, i + 1, True, history
        
        # Error derivative: E'(t) = 1 + (d·v)/(v_p·D)
        # Using proxy derivative (works even with empirical tables)
        dot_product = np.dot(d, v)
        E_prime = 1.0 + dot_product / (projectile_speed * D)
        
        # Check for singularity
        if abs(E_prime) < 1e-9:
            print(f"Warning: Singularity detected at iteration {i}")
            return t, i + 1, False, history
        
        # Newton update
        t_next = t - E / E_prime
        
        # Prevent negative time
        if t_next < 0:
            t_next = t / 2.0
        
        history.append(t_next)
        t = t_next
    
    return t, max_iterations, False, history


# Example vectors
x = np.array([0, 0])  # initial relative position
v = np.array([1.0, -2])  # robot velocity
g = np.array([10, 0])  # target position

# Initial guess (static distance / projectile speed)
static_distance = np.linalg.norm(g - x)
t_initial = static_distance / projectile_speed

print("=" * 60)
print("SOLVER COMPARISON")
print("=" * 60)
print(f"Robot position: {x}")
print(f"Robot velocity: {v}")
print(f"Target position: {g}")
print(f"Projectile speed: {projectile_speed} m/s")
print(f"Initial guess: {t_initial:.4f} s")
print()

# Solve with both methods
t_fixed, iter_fixed, conv_fixed, history_fixed = fixed_point_solver(x, v, g, t_initial)
t_newton, iter_newton, conv_newton, history_newton = newton_method_solver(x, v, g, t_initial)

print("-" * 60)
print("FIXED-POINT ITERATION:")
print(f"  Solution: {t_fixed:.6f} s")
print(f"  Iterations: {iter_fixed}")
print(f"  Converged: {conv_fixed}")
print()

print("NEWTON'S METHOD:")
print(f"  Solution: {t_newton:.6f} s")
print(f"  Iterations: {iter_newton}")
print(f"  Converged: {conv_newton}")
print()

# Verify solution
virtual_target_fixed = g - v * t_fixed
virtual_target_newton = g - v * t_newton
print("VERIFICATION:")
print(f"  Fixed-point virtual target: {virtual_target_fixed}")
print(f"  Newton virtual target: {virtual_target_newton}")
print(f"  Distance (fixed): {np.linalg.norm(virtual_target_fixed - x):.6f} m")
print(f"  Distance (newton): {np.linalg.norm(virtual_target_newton - x):.6f} m")
print("=" * 60)

# ============================================================================
# VISUALIZATION
# ============================================================================

# Time guesses for plotting
t_values = np.linspace(0, 5, 200)  # 0 to 5 seconds
tof_values = np.array([TOF(x, v, g, tt) for tt in t_values])

# Create figure with multiple subplots
fig, axes = plt.subplots(2, 2, figsize=(14, 12))

# ============================================================================
# Plot 1: Fixed-point iteration on the curve
# ============================================================================
ax = axes[0, 0]
ax.plot(t_values, t_values, label='y = t', color='blue', linewidth=2)
ax.plot(t_values, tof_values, label='TOF(t)', color='red', linewidth=2)

# Plot fixed-point iteration path
for i in range(len(history_fixed) - 1):
    t_curr = history_fixed[i]
    t_next = history_fixed[i + 1]
    
    # Vertical line from (t_curr, t_curr) to (t_curr, TOF(t_curr))
    ax.plot([t_curr, t_curr], [t_curr, t_next], 'g--', alpha=0.5, linewidth=1)
    # Horizontal line from (t_curr, TOF(t_curr)) to (TOF(t_curr), TOF(t_curr))
    ax.plot([t_curr, t_next], [t_next, t_next], 'g--', alpha=0.5, linewidth=1)

ax.plot(history_fixed[0], history_fixed[0], 'go', markersize=10, label='Start')
ax.plot(t_fixed, t_fixed, 'g*', markersize=15, label=f'Solution ({iter_fixed} iter)')
ax.set_xlabel('t (seconds)', fontsize=12)
ax.set_ylabel('y (seconds)', fontsize=12)
ax.set_title('Fixed-Point Iteration: Convergence Path', fontsize=14, fontweight='bold')
ax.legend()
ax.grid(True, alpha=0.3)

# ============================================================================
# Plot 2: Newton's method on the curve
# ============================================================================
ax = axes[0, 1]
ax.plot(t_values, t_values, label='y = t', color='blue', linewidth=2)
ax.plot(t_values, tof_values, label='TOF(t)', color='red', linewidth=2)

# Plot Newton's method jumps
for i in range(len(history_newton) - 1):
    t_curr = history_newton[i]
    t_next = history_newton[i + 1]
    
    # Draw arrow from current guess to next guess
    ax.annotate('', xy=(t_next, t_next), xytext=(t_curr, t_curr),
                arrowprops=dict(arrowstyle='->', color='purple', lw=2, alpha=0.6))

ax.plot(history_newton[0], history_newton[0], 'mo', markersize=10, label='Start')
ax.plot(t_newton, t_newton, 'm*', markersize=15, label=f'Solution ({iter_newton} iter)')
ax.set_xlabel('t (seconds)', fontsize=12)
ax.set_ylabel('y (seconds)', fontsize=12)
ax.set_title("Newton's Method: Convergence Path", fontsize=14, fontweight='bold')
ax.legend()
ax.grid(True, alpha=0.3)

# ============================================================================
# Plot 3: Convergence comparison (iteration vs error)
# ============================================================================
ax = axes[1, 0]

# Calculate errors for each iteration
errors_fixed = [abs(t - t_fixed) for t in history_fixed]
errors_newton = [abs(t - t_newton) for t in history_newton]

ax.semilogy(range(len(errors_fixed)), errors_fixed, 'g-o', 
            label=f'Fixed-Point ({iter_fixed} iter)', linewidth=2, markersize=6)
ax.semilogy(range(len(errors_newton)), errors_newton, 'm-s', 
            label=f"Newton's Method ({iter_newton} iter)", linewidth=2, markersize=6)

ax.set_xlabel('Iteration', fontsize=12)
ax.set_ylabel('Absolute Error (log scale)', fontsize=12)
ax.set_title('Convergence Speed Comparison', fontsize=14, fontweight='bold')
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3, which='both')

# ============================================================================
# Plot 4: Iteration history (value vs iteration)
# ============================================================================
ax = axes[1, 1]

ax.plot(range(len(history_fixed)), history_fixed, 'g-o', 
        label=f'Fixed-Point', linewidth=2, markersize=4)
ax.plot(range(len(history_newton)), history_newton, 'm-s', 
        label=f"Newton's Method", linewidth=2, markersize=6)
ax.axhline(y=t_fixed, color='gray', linestyle='--', linewidth=1, label='True solution')

ax.set_xlabel('Iteration', fontsize=12)
ax.set_ylabel('t (seconds)', fontsize=12)
ax.set_title('Time Guess Evolution', fontsize=14, fontweight='bold')
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# ============================================================================
# Additional test: Try different scenarios
# ============================================================================
print("\n" + "=" * 60)
print("TESTING DIFFERENT SCENARIOS")
print("=" * 60)

test_cases = [
    {"name": "Moving toward target", "v": np.array([2.0, 0.0])},
    {"name": "Moving away from target", "v": np.array([-1.5, 0.0])},
    {"name": "Stationary robot", "v": np.array([0.0, 0.0])},
    {"name": "Fast perpendicular motion", "v": np.array([0.0, 4.0])},
]

for case in test_cases:
    v_test = case["v"]
    t_init = static_distance / projectile_speed
    
    t_fp, iter_fp, conv_fp, _ = fixed_point_solver(x, v_test, g, t_init, max_iterations=200)
    t_nm, iter_nm, conv_nm, _ = newton_method_solver(x, v_test, g, t_init)
    
    print(f"\n{case['name']}: v = {v_test}")
    print(f"  Fixed-point: {iter_fp:3d} iterations, converged: {conv_fp}")
    print(f"  Newton:      {iter_nm:3d} iterations, converged: {conv_nm}")
    print(f"  Solutions match: {abs(t_fp - t_nm) < 1e-4}")