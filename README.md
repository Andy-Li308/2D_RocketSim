# 2D Rocket Simulation

A real-time 2D self-landing rocket simulation with full-state feedback LQR control, using SFML for visualization, Eigen for linear algebra, and matplotlib for data plots. The simulation demonstrates how different LQR gain profiles affect rocket landing behavior.

## Overview

- **Physics:** Simulates a 2D rigid-body rocket with 6-DOF (x, y, θ and their derivatives), gravity, thrust, and a variable-thrust gimbaled engine.
- **Control:** Uses a Linear Quadratic Regulator (LQR) for full-state feedback to stabilize the rocket at a desired state equilibrium point. Two profiles are demonstrated:
  - **Slow:** A slower, more gentle response profile that prioritizes minimizing control actuation effort, characterized by a high control cost.
  - **Fast:** A faster, more aggressive response that stabilizes the rocket more rapidly at the expense of greater control effort. 
- **Visualization:** Real-time graphics with SFML, displaying rocket state, control actions, and system behavior over time.

## How to run the simulation: 

1. **LQR Gain Generation:**  
   Run the Python script to generate LQR gain matrices for both profiles:
   ```
   python scripts/compute_lqr_gains.py
   ```
   This produces `lqr_gains_slow.json` and `lqr_gains_fast.json`.

2. **Build the Project:**  
   - Prerequisites: CMake 3.16+, GCC 14+, SFML 3.0+, Eigen 5.0+.
   - Configure and build:
     ```
     cd build
     cmake ..
     cmake --build .
     ```

3. **Run the Simulation:**  
   From the `build` directory:
   ```
   ./RocketSim.exe slow   # Gentle controller
   ./RocketSim.exe fast   # Aggressive controller
   ```
   The controller profile determines which gain file is loaded.
