"""
simulate_bandwidth_vs_update_rate.py
-------------------------------------
Generates a figure showing how the achievable sensor-data update rate
varies with available wireless bandwidth, for different data
representations (raw point cloud, compressed map, sparse diff).

Figure produced: figures/bandwidth_vs_update_rate.pdf
"""

import numpy as np
import matplotlib.pyplot as plt
import os


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "figures")

# Payload sizes in bytes for different representation schemes
PAYLOAD_BYTES = {
    "Raw LiDAR (100k pts)": 100_000 * 3 * 4,   # 100k points × 3 coords × 4 bytes
    "Full occupancy map (compressed)": 20_000,   # ~20 kB after lossless compression
    "Sparse diff update": 1_000,                 # ~1 kB for changed cells only
}


# ---------------------------------------------------------------------------
# Core simulation
# ---------------------------------------------------------------------------

def compute_max_update_rate(bandwidth_bps: float, payload_bytes: int) -> float:
    """
    Return the maximum update rate (Hz) achievable for a given bandwidth
    and fixed payload size, accounting for a simple protocol overhead.

    Parameters
    ----------
    bandwidth_bps : float
        Available channel bandwidth in bits per second.
    payload_bytes : int
        Size of a single data packet in bytes.

    Returns
    -------
    float
        Maximum achievable update rate in Hz.
    """
    # TODO: optionally model protocol overhead (headers, ACKs)
    protocol_overhead_fraction = 0.1  # 10% overhead
    effective_bps = bandwidth_bps * (1.0 - protocol_overhead_fraction)
    payload_bits = payload_bytes * 8
    return effective_bps / payload_bits


def simulate_bandwidth_sweep(
    bandwidth_range_mbps: np.ndarray,
    payload_sizes: dict,
) -> dict:
    """
    For each representation scheme and each bandwidth value, compute the
    maximum achievable update rate.

    Parameters
    ----------
    bandwidth_range_mbps : np.ndarray
        Array of bandwidth values in Mbps.
    payload_sizes : dict
        Mapping from scheme name (str) to payload size in bytes (int).

    Returns
    -------
    dict
        Mapping from scheme name to array of update rates (Hz).
    """
    results = {}
    for name, payload_bytes in payload_sizes.items():
        rates = []
        for bw_mbps in bandwidth_range_mbps:
            bw_bps = bw_mbps * 1e6
            rate = compute_max_update_rate(bw_bps, payload_bytes)
            # TODO: add an upper bound corresponding to sensor hardware max rate
            rate = min(rate, 20.0)  # cap at 20 Hz (typical LiDAR max)
            rates.append(rate)
        results[name] = np.array(rates)
    return results


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_results(bandwidth_range_mbps: np.ndarray, results: dict) -> None:
    """
    Plot update rate vs. bandwidth for all representation schemes.

    Parameters
    ----------
    bandwidth_range_mbps : np.ndarray
        Bandwidth values used in the sweep.
    results : dict
        Output of simulate_bandwidth_sweep().
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7, 4))

    linestyles = ["-", "--", ":"]
    for (name, rates), ls in zip(results.items(), linestyles):
        ax.plot(bandwidth_range_mbps, rates, linestyle=ls, linewidth=2, label=name)

    ax.set_xlabel("Available Bandwidth (Mbps)")
    ax.set_ylabel("Max Update Rate (Hz)")
    ax.set_title("Sensor-Data Update Rate vs. Wireless Bandwidth")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    out_path = os.path.join(OUTPUT_DIR, "bandwidth_vs_update_rate.pdf")
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"Saved figure to {out_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    bandwidth_range = np.linspace(0.1, 10.0, 200)  # 0.1–10 Mbps
    results = simulate_bandwidth_sweep(bandwidth_range, PAYLOAD_BYTES)
    plot_results(bandwidth_range, results)
