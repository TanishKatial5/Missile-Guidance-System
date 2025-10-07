import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# ==================== PARAMETERS ====================
class Params:
    def __init__(self):
        self.Vm = 300.0      # Missile speed (m/s)
        self.Vt = 200.0      # Target speed (m/s)
        self.N = 4.0         # PNG navigation constant
        self.K = 50.0        # SMC control gain
        self.alpha = 2.0     # SMC sliding surface parameter

# ==================== GUIDANCE LAWS ====================
def png_control(lambda_val, lambda_dot, Vm, Vt, theta_m, theta_t, N):
    """Proportional Navigation Guidance"""
    Vc = Vm * np.cos(theta_m - lambda_val) + Vt * np.cos(theta_t - lambda_val)
    am = N * Vc * lambda_dot
    return np.clip(am, -100, 100)  # Limit acceleration

def smc_control(lambda_val, lambda_dot, K, alpha):
    """Sliding Mode Control"""
    s = lambda_dot + alpha * lambda_val
    am = -K * np.sign(s)
    return np.clip(am, -100, 100)  # Limit acceleration

# ==================== DYNAMICS ====================
def missile_target_dynamics(state, t, params, control_type='PNG'):
    """
    State: [xm, ym, theta_m, xt, yt, theta_t]
    """
    xm, ym, theta_m, xt, yt, theta_t = state
    
    # Relative position
    dx = xt - xm
    dy = yt - ym
    r = np.sqrt(dx**2 + dy**2)
    
    # Avoid division by zero
    if r < 1.0:
        return [0, 0, 0, 0, 0, 0]
    
    # Line-of-sight angle
    lambda_val = np.arctan2(dy, dx)
    
    # LOS rate
    dr_dt = -params.Vm * np.cos(theta_m - lambda_val) - params.Vt * np.cos(theta_t - lambda_val)
    lambda_dot = (params.Vm * np.sin(theta_m - lambda_val) + params.Vt * np.sin(theta_t - lambda_val)) / r
    
    # Control law selection
    if control_type == 'PNG':
        am = png_control(lambda_val, lambda_dot, params.Vm, params.Vt, theta_m, theta_t, params.N)
    else:  # SMC
        am = smc_control(lambda_val, lambda_dot, params.K, params.alpha)
    
    # Target acceleration (non-maneuvering for simplicity)
    at = 0.0
    
    # State derivatives
    dxm_dt = params.Vm * np.cos(theta_m)
    dym_dt = params.Vm * np.sin(theta_m)
    dtheta_m_dt = am / params.Vm
    
    dxt_dt = params.Vt * np.cos(theta_t)
    dyt_dt = params.Vt * np.sin(theta_t)
    dtheta_t_dt = at / params.Vt
    
    return [dxm_dt, dym_dt, dtheta_m_dt, dxt_dt, dyt_dt, dtheta_t_dt]

# ==================== SIMULATION ====================
def run_simulation(control_type='PNG'):
    """Run missile guidance simulation"""
    params = Params()
    
    # Initial conditions
    xm0, ym0 = 0.0, 0.0
    theta_m0 = np.deg2rad(45)
    
    xt0, yt0 = 4000.0, 3000.0
    theta_t0 = np.deg2rad(135)
    
    state0 = [xm0, ym0, theta_m0, xt0, yt0, theta_t0]
    
    # Time span
    t = np.linspace(0, 20, 2000)
    
    # Solve ODE
    solution = odeint(missile_target_dynamics, state0, t, args=(params, control_type))
    
    # Extract trajectories
    xm, ym = solution[:, 0], solution[:, 1]
    xt, yt = solution[:, 3], solution[:, 4]
    
    # Calculate miss distance
    distances = np.sqrt((xt - xm)**2 + (yt - ym)**2)
    miss_distance = np.min(distances)
    intercept_idx = np.argmin(distances)
    intercept_time = t[intercept_idx]
    
    return t, solution, miss_distance, intercept_time, params

# ==================== PLOTTING ====================
def plot_results(t_png, sol_png, t_smc, sol_smc, miss_png, miss_smc, time_png, time_smc):
    """Create comparison plots"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # PNG Trajectory
    ax = axes[0, 0]
    ax.plot(sol_png[:, 0], sol_png[:, 1], 'b-', linewidth=2, label='Missile')
    ax.plot(sol_png[:, 3], sol_png[:, 4], 'r--', linewidth=2, label='Target')
    ax.plot(sol_png[0, 0], sol_png[0, 1], 'bo', markersize=10, label='Start')
    ax.plot(sol_png[-1, 3], sol_png[-1, 4], 'ro', markersize=10, label='End')
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'PNG Trajectory\nMiss: {miss_png:.2f}m, Time: {time_png:.2f}s', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # SMC Trajectory
    ax = axes[0, 1]
    ax.plot(sol_smc[:, 0], sol_smc[:, 1], 'b-', linewidth=2, label='Missile')
    ax.plot(sol_smc[:, 3], sol_smc[:, 4], 'r--', linewidth=2, label='Target')
    ax.plot(sol_smc[0, 0], sol_smc[0, 1], 'bo', markersize=10, label='Start')
    ax.plot(sol_smc[-1, 3], sol_smc[-1, 4], 'ro', markersize=10, label='End')
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'SMC Trajectory\nMiss: {miss_smc:.2f}m, Time: {time_smc:.2f}s', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Range vs Time
    ax = axes[1, 0]
    r_png = np.sqrt((sol_png[:, 3] - sol_png[:, 0])**2 + (sol_png[:, 4] - sol_png[:, 1])**2)
    r_smc = np.sqrt((sol_smc[:, 3] - sol_smc[:, 0])**2 + (sol_smc[:, 4] - sol_smc[:, 1])**2)
    ax.plot(t_png, r_png, 'b-', linewidth=2, label='PNG')
    ax.plot(t_smc, r_smc, 'r--', linewidth=2, label='SMC')
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Range (m)', fontsize=12)
    ax.set_title('Relative Distance vs Time', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Performance Comparison
    ax = axes[1, 1]
    methods = ['PNG', 'SMC']
    miss_distances = [miss_png, miss_smc]
    colors = ['blue', 'red']
    bars = ax.bar(methods, miss_distances, color=colors, alpha=0.7, edgecolor='black', linewidth=2)
    ax.set_ylabel('Miss Distance (m)', fontsize=12)
    ax.set_title('Miss Distance Comparison', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for bar, val in zip(bars, miss_distances):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{val:.2f}m', ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    plt.tight_layout()
    plt.show()

# ==================== MAIN EXECUTION ====================
print("🚀 Missile Guidance System Simulation")
print("=" * 50)
print("\nRunning PNG simulation...")
t_png, sol_png, miss_png, time_png, params = run_simulation('PNG')
print(f"✓ PNG Complete - Miss Distance: {miss_png:.2f}m")

print("\nRunning SMC simulation...")
t_smc, sol_smc, miss_smc, time_smc, _ = run_simulation('SMC')
print(f"✓ SMC Complete - Miss Distance: {miss_smc:.2f}m")

print("\n" + "=" * 50)
print("RESULTS SUMMARY")
print("=" * 50)
print(f"\nProportional Navigation Guidance (PNG):")
print(f"  • Miss Distance: {miss_png:.2f} m")
print(f"  • Intercept Time: {time_png:.2f} s")
print(f"\nSliding Mode Control (SMC):")
print(f"  • Miss Distance: {miss_smc:.2f} m")
print(f"  • Intercept Time: {time_smc:.2f} s")
print(f"\n🎯 Winner: {'SMC' if miss_smc < miss_png else 'PNG'} (Lower miss distance)")
print("=" * 50)

# Generate plots
plot_results(t_png, sol_png, t_smc, sol_smc, miss_png, miss_smc, time_png, time_smc)