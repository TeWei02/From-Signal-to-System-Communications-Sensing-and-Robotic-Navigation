"""
plot_latency_vs_nav_error.py
-----------------------------
Simulate and plot the impact of communication latency on navigation error.

Scenario:
  A remote-operated or edge-assisted robot receives velocity commands / waypoint
  updates with a certain round-trip latency.
  
  Variables:
    - Robot forward speed (0.5-2.0 m/s)
    - Communication latency / RTT (0-500 ms)
    - Dead-band error accumulates during the latency period
  
  Output: Error vs latency curves for different speeds
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false

import numpy as np
import matplotlib.pyplot as plt


def compute_deadband_error(robot_speed_ms: float, rtt_latency_ms: float) -> float:
    """
    Compute position error due to communication latency.
    
    During latency period, the robot is "blind" to updates and accumulates
    dead-reckoning error (assuming worst case: continues moving in the same
    direction until update arrives).
    
    Args:
        robot_speed_ms: Robot forward speed (m/s)
        rtt_latency_ms: Round-trip latency (milliseconds)
        
    Returns:
        Dead-band error distance (meters)
    """
    latency_s = rtt_latency_ms / 1000.0
    error_m = robot_speed_ms * latency_s
    return error_m


def plot_latency_vs_error(output_file: str = "latency_vs_nav_error.pdf"):
    """
    Plot navigation error as a function of communication latency.
    
    X-axis: Round-trip latency (milliseconds)
    Y-axis: Dead-band position error (meters)
    Lines: Different robot speeds
    """
    latencies_ms = np.linspace(0, 500, 100)
    robot_speeds = [0.5, 1.0, 1.5, 2.0]  # m/s
    
    _, ax = plt.subplots(figsize=(10, 7))
    
    colors = plt.get_cmap("viridis")(np.linspace(0, 1, len(robot_speeds)))  # pyright: ignore[reportAttributeAccessIssue]
    
    for speed, color in zip(robot_speeds, colors):
        errors = [compute_deadband_error(speed, lat) for lat in latencies_ms]
        ax.plot(latencies_ms, errors, 'o-', linewidth=2.5, markersize=5,
                label=f"Speed {speed} m/s", color=color)
    
    # Add reference lines for safety / accuracy constraints
    ax.axhline(y=0.1, color='red', linestyle='--', alpha=0.5, linewidth=2,
               label="Acceptable error (0.1 m)")
    ax.axhline(y=0.5, color='orange', linestyle='--', alpha=0.5, linewidth=2,
               label="Warning threshold (0.5 m)")
    ax.axvline(x=100, color='gray', linestyle=':', alpha=0.5, linewidth=1.5,
               label="Typical WiFi latency (~100 ms)")
    
    ax.set_xlabel("Round-Trip Latency (ms)", fontsize=12)
    ax.set_ylabel("Dead-Band Position Error (m)", fontsize=12)
    ax.set_title("Navigation Error vs. Communication Latency\n(position uncertainty during control update delay)", 
                 fontsize=13)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10, loc='upper left')
    ax.set_xlim(0, 500)
    ax.set_ylim(0, 1.2)
    
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=150)
    print(f"Saved latency-error plot to {output_file}")
    plt.close()


def plot_latency_with_jitter(output_file: str = "latency_with_jitter.pdf"):
    """
    Plot latency impact with uncertainty bands (jitter/variance).
    
    Assumes latency follows a distribution (e.g., Gaussian with std = 20 ms).
    """
    latencies_ms = np.linspace(0, 500, 100)
    robot_speed = 1.0  # m/s
    jitter_std = 20    # Standard deviation of latency (ms)
    
    _, ax = plt.subplots(figsize=(10, 7))
    
    # Mean error
    errors_mean = [compute_deadband_error(robot_speed, lat) for lat in latencies_ms]
    
    # Error bounds (±1 std of jitter)
    errors_lower = np.array([compute_deadband_error(robot_speed, max(0, lat - jitter_std)) for lat in latencies_ms])
    errors_upper = np.array([compute_deadband_error(robot_speed, lat + jitter_std) for lat in latencies_ms])
    
    ax.plot(latencies_ms, errors_mean, 'b-', linewidth=2.5, label='Mean latency')
    ax.fill_between(latencies_ms, errors_lower, errors_upper, alpha=0.3, color='blue', label=f'Jitter band (±{jitter_std} ms)')  # pyright: ignore[reportArgumentType]
    
    # Acceptable error region
    ax.axhline(y=0.1, color='green', linestyle='--', alpha=0.7, linewidth=2,
               label="Acceptable error (0.1 m)")
    ax.axvspan(0, 100, alpha=0.1, color='green', label="Low-risk latency")
    
    ax.set_xlabel("Mean Round-Trip Latency (ms)", fontsize=12)
    ax.set_ylabel("Dead-Band Error (m)", fontsize=12)
    ax.set_title(f"Navigation Error with Latency Jitter\n(robot speed: {robot_speed} m/s, jitter σ = {jitter_std} ms)",
                 fontsize=13)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11, loc='upper left')
    ax.set_xlim(0, 500)
    ax.set_ylim(0, 1.0)
    
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=150)
    print(f"Saved jitter plot to {output_file}")
    plt.close()


def main():
    """Generate latency plots."""
    plot_latency_vs_error(output_file="latency_vs_nav_error.pdf")
    plot_latency_with_jitter(output_file="latency_with_jitter.pdf")
    print("\nPlots generated in current directory.")


if __name__ == "__main__":
    main()
