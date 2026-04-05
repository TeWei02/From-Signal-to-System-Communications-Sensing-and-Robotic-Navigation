"""
plot_bandwidth_vs_update_rate.py
---------------------------------
Simulate and plot the trade-off between sensor update rate and wireless bandwidth.

Scenario:
  A single robot streams LiDAR data to an edge server.
  
  Variables:
    - Sensor update rate (1 Hz to 20 Hz)
    - Compression ratio (raw vs lossy vs very aggressive)
    - Available bandwidth (1 Mbps to 50 Mbps)
  
  Calculate: Maximum achievable sensor rate for each (bandwidth, compression) pair
  
  Output: Pareto frontier showing achievable operating points
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false

import numpy as np
import matplotlib.pyplot as plt
from enum import Enum


class LiDARCompression(Enum):
    """LiDAR data representation strategies."""
    RAW = "raw"               # Full 3D point cloud (~50 MB/s at 10 Hz)
    DOWNSAMPLED = "downsampled"    # 0.5× resolution (~5 MB/s)
    SPARSE = "sparse"          # Sparse occupancy diff (~0.5 MB/s)


def estimate_lidar_bandwidth(update_rate_hz: int, compression: LiDARCompression) -> float:
    """
    Estimate required bandwidth for streaming LiDAR.
    
    Args:
        update_rate_hz: LiDAR scan frequency
        compression: Compression strategy
        
    Returns:
        Required bandwidth in Mbps
    """
    # Base rates (rough estimates from sensor specs)
    base_rates_mbps = {
        LiDARCompression.RAW: 50.0,        # 3D point cloud, full res
        LiDARCompression.DOWNSAMPLED: 5.0, # Medium resolution
        LiDARCompression.SPARSE: 0.5,      # Sparse occupancy / differential
    }
    
    base = base_rates_mbps[compression]
    return base * (update_rate_hz / 10.0)  # Scale by rate (assume 10 Hz baseline)


def plot_bandwidth_vs_rate(output_file: str = "bandwidth_vs_update_rate.pdf"):
    """
    Generate Pareto frontier plot.
    
    X-axis: Sensor update rate (Hz)
    Y-axis: Required bandwidth (Mbps)
    Lines: One per compression strategy
    """
    update_rates = np.linspace(1, 20, 20)
    
    _, ax = plt.subplots(figsize=(10, 7))
    
    colors = {
        LiDARCompression.RAW: 'red',
        LiDARCompression.DOWNSAMPLED: 'orange',
        LiDARCompression.SPARSE: 'green',
    }
    
    for compression in LiDARCompression:
        bandwidths = [estimate_lidar_bandwidth(int(rate), compression) for rate in update_rates]
        ax.plot(update_rates, bandwidths, 'o-', linewidth=2.5,
                label=compression.value.upper(), color=colors[compression], markersize=6)
    
    # Add reference lines for available bandwidths
    ax.axhline(y=2.0, color='gray', linestyle='--', alpha=0.5, label="Typical WiFi")
    ax.axhline(y=10.0, color='blue', linestyle='--', alpha=0.5, label="Good WiFi / 4G")
    ax.axhline(y=30.0, color='darkgreen', linestyle='--', alpha=0.5, label="5G")
    
    ax.set_xlabel("LiDAR Update Rate (Hz)", fontsize=12)
    ax.set_ylabel("Required Bandwidth (Mbps)", fontsize=12)
    ax.set_title("Sensor Data Rate vs. Wireless Bandwidth\n(LiDAR streaming to edge server)", fontsize=13)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11, loc='upper left')
    ax.set_ylim(0, 120)
    
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=150)
    print(f"Saved bandwidth plot to {output_file}")
    plt.close()


def plot_achievable_rates(available_bandwidth_mbps: list[float] | None = None, 
                          output_file: str = "achievable_rates.pdf"):
    """
    Create a table showing achievable sensor rates for given bandwidths.
    
    Args:
        available_bandwidth_mbps: List of bandwidth options (Mbps)
        output_file: Output filename
    """
    if available_bandwidth_mbps is None:
        available_bandwidth_mbps = [0.5, 1.0, 2.0, 5.0, 10.0, 30.0]
    
    _, ax = plt.subplots(figsize=(10, 6))
    ax.axis('tight')
    ax.axis('off')
    
    # Build table: rows=bandwidths, columns=compression strategies
    rows = []
    for bw in available_bandwidth_mbps:
        row = [f"{bw:.1f}"]
        for compression in LiDARCompression:
            # Binary search for max achievable rate
            max_rate = 1
            for rate in range(1, 31):
                required_bw = estimate_lidar_bandwidth(rate, compression)
                if required_bw <= bw:
                    max_rate = rate
                else:
                    break
            row.append(f"{max_rate} Hz")
        rows.append(row)
    
    cols = ["Bandwidth"] + [c.value.title() for c in LiDARCompression]
    
    table = ax.table(cellText=rows, colLabels=cols, cellLoc='center', loc='center',
                     colColours=['lightgray']*len(cols))
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)
    
    plt.title("Maximum Achievable LiDAR Update Rate\nvs. Bandwidth & Compression", fontsize=12)
    plt.savefig(output_file, format='pdf', bbox_inches='tight', dpi=150)
    print(f"Saved achievable rates table to {output_file}")
    plt.close()


def main():
    """Generate bandwidth vs rate plots."""
    plot_bandwidth_vs_rate(output_file="bandwidth_vs_update_rate.pdf")
    plot_achievable_rates(output_file="achievable_rates.pdf")
    print("\nPlots generated in current directory.")


if __name__ == "__main__":
    main()
