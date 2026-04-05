"""
simulate_latency_vs_error.py
------------------------------
Generates a figure showing how round-trip latency (RTT) affects the
dead-band navigation error of a robot following a command computed remotely.

The model: a robot moving at constant speed v receives control commands
over a wireless link with RTT = tau.  During tau the robot travels v*tau
from the position assumed when the command was generated, leading to a
position tracking error proportional to tau (linear regime) with added
noise modelling network jitter.

Figure produced: figures/latency_vs_error.pdf
"""

import numpy as np
import matplotlib.pyplot as plt
import os


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "figures")

ROBOT_SPEEDS_MS = [0.2, 0.5, 1.0]   # m/s — slow, medium, fast robot
RNG_SEED = 42


# ---------------------------------------------------------------------------
# Core model
# ---------------------------------------------------------------------------

def dead_band_error(latency_s: float, speed_ms: float) -> float:
    """
    Compute the deterministic dead-band displacement error.

    Parameters
    ----------
    latency_s : float
        Round-trip latency in seconds.
    speed_ms : float
        Robot speed in m/s.

    Returns
    -------
    float
        Expected position error in metres.
    """
    return speed_ms * latency_s


def simulate_latency_sweep(
    latency_range_ms: np.ndarray,
    speeds_ms: list,
    jitter_std_ms: float = 5.0,
    n_samples: int = 100,
    rng: np.random.Generator = None,
) -> dict:
    """
    For each speed and each latency value, compute mean position error
    averaged over random jitter realisations.

    Parameters
    ----------
    latency_range_ms : np.ndarray
        Array of mean RTT values in milliseconds.
    speeds_ms : list of float
        Robot speeds in m/s.
    jitter_std_ms : float
        Standard deviation of per-packet jitter in milliseconds.
    n_samples : int
        Number of jitter samples per (latency, speed) pair.
    rng : np.random.Generator
        Random number generator (created if None).

    Returns
    -------
    dict
        Mapping from speed (float) to dict with keys 'mean' and 'std'
        (both np.ndarray of length == len(latency_range_ms)).
    """
    if rng is None:
        rng = np.random.default_rng(RNG_SEED)

    results = {}
    for speed in speeds_ms:
        mean_errors = []
        std_errors = []
        for lat_ms in latency_range_ms:
            # Sample jitter-perturbed latency values
            # TODO: replace with measured jitter distribution from a real testbed
            jittered_latency_s = (
                lat_ms + rng.normal(0.0, jitter_std_ms, n_samples)
            ).clip(min=0.0) / 1000.0

            errors = dead_band_error(jittered_latency_s, speed)
            mean_errors.append(float(np.mean(errors)))
            std_errors.append(float(np.std(errors)))

        results[speed] = {
            "mean": np.array(mean_errors),
            "std": np.array(std_errors),
        }
    return results


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_results(latency_range_ms: np.ndarray, results: dict) -> None:
    """
    Plot mean navigation error (with ±1σ band) vs. latency for each speed.

    Parameters
    ----------
    latency_range_ms : np.ndarray
        Latency values used in the sweep.
    results : dict
        Output of simulate_latency_sweep().
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7, 4))

    colors = ["tab:blue", "tab:orange", "tab:green"]
    for (speed, data), color in zip(results.items(), colors):
        mean = data["mean"]
        std = data["std"]
        label = f"v = {speed} m/s"
        ax.plot(latency_range_ms, mean, color=color, linewidth=2, label=label)
        ax.fill_between(
            latency_range_ms,
            mean - std,
            mean + std,
            color=color,
            alpha=0.15,
        )

    ax.set_xlabel("Round-Trip Latency (ms)")
    ax.set_ylabel("Position Error (m)")
    ax.set_title("Dead-Band Navigation Error vs. Network Latency")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    out_path = os.path.join(OUTPUT_DIR, "latency_vs_error.pdf")
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"Saved figure to {out_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    latency_range = np.linspace(0.0, 500.0, 200)  # 0–500 ms RTT
    results = simulate_latency_sweep(latency_range, ROBOT_SPEEDS_MS)
    plot_results(latency_range, results)
