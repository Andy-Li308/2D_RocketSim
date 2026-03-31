#!/usr/bin/env python3
"""
Compute LQR gains for 2D rocket full-state feedback controller.
Uses linearized system dynamics around hovering equilibrium.

State: [x, xdot, y, ydot, theta, thetadot]
Control: [T (thrust), alpha (gimbal angle)]

Usage:
  python compute_lqr_gains.py --profile slow    -> lqr_gains_slow.json
  python compute_lqr_gains.py --profile fast    -> lqr_gains_fast.json
  python compute_lqr_gains.py                   -> both files
"""

import argparse
import numpy as np
from scipy.linalg import solve_continuous_are
import json


# Q/R weight profiles
PROFILES = {
    "slow": {
        "Q": [2.0, 0.5, 2.0, 0.5, 1.0, 0.1],
        "R": [5.0, 5.0],
        "description": "Slow controller - low control effort, gentle response",
    },
    "fast": {
        "Q": [2.0, 0.5, 20.0, 5.0, 1.0, 0.1],
        "R": [0.5, 5.0],
        "description": "Aggressive controller - fast vertical response, same gentle lateral",
    },
}


def compute_lqr_gains(m=1.0, L=0.5, g=9.81, profile="default"):
    cfg = PROFILES[profile]
    Q_diag = cfg["Q"]
    R_diag = cfg["R"]

    l = L / 2.0
    I = (1.0 / 12.0) * m * L**2

    print(f"Profile: {profile} — {cfg['description']}")
    print(f"System: m={m}, L={L}, l={l}, I={I:.6f}, g={g}")

    # Linearized A matrix (6x6) around hover equilibrium (T=mg, alpha=0, theta=0)
    A = np.array([
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, g, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0],
    ], dtype=float)

    # Linearized B matrix (6x2)
    B = np.array([
        [0, 0],
        [0, g],
        [0, 0],
        [1.0/m, 0],
        [0, 0],
        [0, l*g/I],
    ], dtype=float)

    Q = np.diag(Q_diag)
    R = np.diag(R_diag)

    print(f"Q diag: {Q_diag}")
    print(f"R diag: {R_diag}")

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P

    print(f"\nK matrix (2x6):")
    print(f"  Thrust row: {K[0]}")
    print(f"  Gimbal row: {K[1]}")

    eigs = np.linalg.eigvals(A - B @ K)
    print(f"\nClosed-loop eigenvalues:")
    for i, e in enumerate(eigs):
        print(f"  eig[{i}] = {e:.4f}")
    stable = all(e.real < 0 for e in eigs)
    print(f"Stability: {'STABLE' if stable else 'UNSTABLE!'}\n")

    return K


def save_gains(K, filename, m=1.0, L=0.5, g=9.81, profile="default"):
    data = {
        "description": f"LQR gains — {profile} profile",
        "profile": profile,
        "state_order": ["x", "xdot", "y", "ydot", "theta", "thetadot"],
        "control_order": ["thrust", "alpha"],
        "dimensions": {"K_matrix": [2, 6], "format": "row-major (2x6)"},
        "k_matrix": K.tolist(),
        "system_parameters": {"mass_kg": m, "length_m": L, "gravity": g},
    }
    with open(filename, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Saved gains to {filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=["slow", "fast"], default=None)
    args = parser.parse_args()

    if args.profile:
        profiles = [args.profile]
    else:
        profiles = ["slow", "fast"]

    for profile in profiles:
        K = compute_lqr_gains(profile=profile)
        save_gains(K, f"lqr_gains_{profile}.json", profile=profile)
