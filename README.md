# From Signal to System: Communications, Sensing, and Robotic Navigation

A mini research report and companion Python experiment code exploring the intersection
of wireless communications engineering and autonomous robotic navigation.

---

## Repository Layout

```
.
├── report/
│   └── report.tex          # LaTeX report body — paste directly into Overleaf
│                           # (no preamble; use \documentclass{article} or IEEEtran)
└── code/
    ├── requirements.txt                      # Python dependencies (numpy, matplotlib)
    ├── simulate_bandwidth_vs_update_rate.py  # Bandwidth vs. sensor-update-rate figure
    ├── simulate_latency_vs_error.py          # Latency vs. navigation-error figure
    └── multi_robot_exploration_sim.py        # Multi-robot grid-world exploration sim
```

---

## Report (`report/report.tex`)

The LaTeX file contains the full body of the mini-paper with the following sections:

1. **Introduction** — motivation for a systems-level view of robotic navigation
2. **Sensing and Signal Acquisition** — LiDAR, IMU, camera; SNR; time-synchronisation
3. **Perception and Mapping** — SLAM, occupancy grid, costmap, sensor fusion, EKF
4. **Local Planning, Control, and On-Board Computation** — DWA, MPC, embedded SoCs
5. **Communication in Robotic Navigation Systems** — Wi-Fi / 4G / 5G; bandwidth, latency, packet loss
6. **Edge/Cloud Offloading and System-Level Trade-offs** — compute–communication trade-off, rate-distortion
7. **Case Study: Multi-Robot Exploration with Limited Bandwidth** — full-map vs. sparse-diff strategies
8. **Conclusion and Future Work** — semantic comms, learned compression, 5G/6G outlook

### Using in Overleaf

Create a new project and paste the following minimal preamble, then copy the body directly:

```latex
\documentclass[11pt]{article}
\usepackage{amsmath}
\usepackage{graphicx}
\usepackage{hyperref}
\begin{document}
% paste contents of report/report.tex here
\end{document}
```

---

## Python Experiments (`code/`)

### Setup

```bash
cd code
pip install -r requirements.txt
```

### Scripts

| Script | What it produces |
|--------|-----------------|
| `simulate_bandwidth_vs_update_rate.py` | `figures/bandwidth_vs_update_rate.pdf` — max sensor-data update rate vs. available wireless bandwidth for three data representations (raw LiDAR, compressed map, sparse diff) |
| `simulate_latency_vs_error.py` | `figures/latency_vs_error.pdf` — dead-band position error vs. round-trip latency for three robot speeds, with jitter uncertainty bands |
| `multi_robot_exploration_sim.py` | `figures/multi_robot_coverage.pdf` — coverage fraction over time comparing full-map upload (Strategy A) vs. sparse differential updates (Strategy B) in a 30×30 grid world |

Run all three:

```bash
cd code
python simulate_bandwidth_vs_update_rate.py
python simulate_latency_vs_error.py
python multi_robot_exploration_sim.py
```

Figures are saved to `code/figures/`.

---

## Key Ideas

- Robotic navigation is an end-to-end **communication system**: information is acquired,
  encoded, transmitted, decoded, and acted upon under resource constraints at every stage.
- **Bandwidth** limits the rate at which sensor data can be offloaded to edge/cloud.
- **Latency** introduces dead-band error proportional to robot speed × RTT.
- **Sparse differential map updates** can achieve near-full coverage at a fraction of
  the bandwidth cost of full-map uploads.
