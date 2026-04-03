

import argparse
import numpy as np
from scipy.linalg import solve_continuous_are
import json


# Q/R weight profiles
PROFILES = {
    "slow": {
        "Q": [2.0, 0.5, 2.0, 6.5, 1.0, 0.1],
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


    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

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
