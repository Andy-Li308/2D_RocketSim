# 2D Rocket Simulation

A real-time 2D rocket landing simulation with full-state feedback LQR control, using SFML for visualization and Eigen for linear algebra. The simulation demonstrates how different LQR gain profiles affect rocket landing behavior.

## Overview

- **Physics:** Simulates a 2D rigid-body rocket with 6-DOF (x, y, θ and their derivatives), gravity, thrust, and gimbaled engine.
- **Control:** Uses a Linear Quadratic Regulator (LQR) for full-state feedback. Two profiles are available:
  - **Slow:** Gentle, low-effort control.
  - **Fast:** Aggressive, rapid stabilization.
- **Visualization:** Real-time graphics with SFML, showing rocket state and control actions.

## How It Works

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

## Project Structure

- **src/**: C++ source files for simulation, control, and rendering.
- **include/**: C++ headers.
- **scripts/**: Python script for generating LQR gains.
- **lqr_gains_*.json**: Controller gain files (auto-generated).
- **build/**: Build output directory.
