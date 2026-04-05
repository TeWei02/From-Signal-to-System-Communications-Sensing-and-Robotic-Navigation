"""
coverage_analysis.py
---------------------
Analyze spatial coverage metrics from simulation runs.

Computes:
  - Coverage fraction (% of environment explore)
  - Frontier length (uncovered-explored boundary)
  - Exploration efficiency (area covered per byte transmitted)
  - Per-robot contribution to coverage
  - Coverage growth rate / asymptotic behavior

Input: .npz or .bag log file from simulation
Output: Plots of coverage vs time, efficiency metrics
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Optional, Tuple
from dataclasses import dataclass


@dataclass
class CoverageMetrics:
    """Aggregated coverage statistics from a run."""
    total_area_m2: float
    covered_area_m2: float
    coverage_fraction: float          # [0, 1]
    frontier_length_m: float
    per_robot_area: Dict[str, float]  # Area each robot covered
    efficiency_m2_per_megabyte: float
    total_bytes_transmitted: int
    exploration_time_s: float


class CoverageAnalyzer:
    """
    Analyzes and visualizes coverage data from simulation.
    """
    
    def __init__(self, grid_resolution_m: float = 0.1):
        """
        Args:
            grid_resolution_m: Resolution of coverage grid for calculation
        """
        self.grid_resolution = grid_resolution_m
        self.coverage_map: Optional[np.ndarray] = None
        self.frontier_map: Optional[np.ndarray] = None
    
    def load_from_npz(self, npz_file: str) -> CoverageMetrics:
        """
        Load coverage data from .npz file.
        
        Args:
            npz_file: Path to .npz data file (output from sim_runner.py)
            
        Returns:
            CoverageMetrics object
        """
        data = np.load(npz_file, allow_pickle=True)
        
        # TODO: Extract coverage grid and robot contributions from npz
        # Example structure:
        #   - coverage_grid: (H, W) binary array of explored cells
        #   - robot_01_grid, robot_02_grid, ...
        #   - comm_stats: total bytes, latency, etc.
        
        metrics = CoverageMetrics(
            total_area_m2=100.0,  # TODO: compute from grid size
            covered_area_m2=0.0,
            coverage_fraction=0.0,
            frontier_length_m=0.0,
            per_robot_area={},
            efficiency_m2_per_megabyte=0.0,
            total_bytes_transmitted=int(data.get('bytes_sent', 0)),
            exploration_time_s=float(data.get('sim_time', 0.0))
        )
        
        return metrics
    
    def compute_coverage_fraction(self, occupancy_grid: np.ndarray) -> Tuple[float, float]:
        """
        Compute coverage fraction and frontier length from an occupancy grid.
        
        Args:
            occupancy_grid: 2D array where:
                            -1 = unexplored
                            0-100 = occupancy probability
        
        Returns:
            (coverage_fraction, frontier_length_m)
        """
        # Explored cells: anything that's not -1
        explored = (occupancy_grid != -1).astype(int)
        coverage_fraction = np.sum(explored) / explored.size
        
        # Frontier: boundary between explored and unexplored
        # (Detectable as cells adjacent to unexplored)
        unexplored_mask = (occupancy_grid == -1)
        frontier = np.zeros_like(explored)
        
        for i in range(unexplored_mask.shape[0]):
            for j in range(unexplored_mask.shape[1]):
                if unexplored_mask[i, j]:
                    # Check if any neighbor is explored
                    neighbors = [
                        (i-1, j), (i+1, j), (i, j-1), (i, j+1)
                    ]
                    for ni, nj in neighbors:
                        if 0 <= ni < unexplored_mask.shape[0] and 0 <= nj < unexplored_mask.shape[1]:
                            if not unexplored_mask[ni, nj]:  # Neighbor is explored
                                frontier[i, j] = 1
        
        frontier_length = np.sum(frontier) * self.grid_resolution
        
        return coverage_fraction, frontier_length
    
    def plot_coverage_vs_time(self, coverage_log: Dict[float, float], output_file: str = "coverage_vs_time.pdf"):
        """
        Plot coverage fraction as a function of simulated time.
        
        Args:
            coverage_log: Dict {sim_time: coverage_fraction}
            output_file: Output PDF filename
        """
        times = sorted(coverage_log.keys())
        fractions = [coverage_log[t] for t in times]
        
        plt.figure(figsize=(10, 6))
        plt.plot(times, fractions, 'b-', linewidth=2, label="Coverage")
        plt.xlabel("Simulation Time (s)")
        plt.ylabel("Coverage Fraction")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_file, format='pdf')
        print(f"Saved coverage plot to {output_file}")
    
    def plot_efficiency(self, metrics: CoverageMetrics, output_file: str = "efficiency.pdf"):
        """
        Plot coverage efficiency (area per megabyte) vs time.
        
        Args:
            metrics: CoverageMetrics from analysis
            output_file: Output filename
        """
        mb_transmitted = metrics.total_bytes_transmitted / 1e6
        if mb_transmitted == 0:
            print("No data transmitted; skipping efficiency plot")
            return
        
        efficiency = metrics.covered_area_m2 / mb_transmitted
        
        _, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Coverage bar
        ax1.barh(['Coverage'], [metrics.coverage_fraction * 100], color='steelblue')
        ax1.set_xlabel("Coverage (%)")
        ax1.set_xlim([0, 100])
        
        # Efficiency bar
        ax2.barh(['Efficiency'], [efficiency], color='seagreen')
        ax2.set_xlabel("Area per MB (m²/MB)")
        
        plt.tight_layout()
        plt.savefig(output_file, format='pdf')
        print(f"Saved efficiency plot to {output_file}")


def main():
    """Example: run coverage analysis on a data file."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Analyze coverage from simulation run")
    parser.add_argument("data_file", type=str, help="NPZ or log file from simulation")
    parser.add_argument("--output", type=str, default="analysis/", help="Output directory")
    
    args = parser.parse_args()
    
    analyzer = CoverageAnalyzer()
    metrics = analyzer.load_from_npz(args.data_file)
    
    print(f"\nCoverage Analysis Results:")
    print(f"  Coverage: {metrics.coverage_fraction * 100:.1f}%")
    print(f"  Area covered: {metrics.covered_area_m2:.1f} m²")
    print(f"  Bytes transmitted: {metrics.total_bytes_transmitted / 1e6:.2f} MB")
    print(f"  Efficiency: {metrics.efficiency_m2_per_megabyte:.2f} m²/MB")
    print(f"  Exploration time: {metrics.exploration_time_s:.1f} s")


if __name__ == "__main__":
    main()
