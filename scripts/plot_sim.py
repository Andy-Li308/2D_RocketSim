import numpy as np
import matplotlib.pyplot as plt
import sys
import os

csv_path = os.path.join(os.path.dirname(__file__), "..", "sim_log.csv")
if len(sys.argv) > 1:
    csv_path = sys.argv[1]

data = np.genfromtxt(csv_path, delimiter=",", names=True)

# Cut off data once all errors reach ~zero (hover equilibrium)
settled = (
    (np.abs(data["x_err"]) < 0.05)
    & (np.abs(data["y_err"]) < 0.05)
    & (np.abs(data["xdot"]) < 0.05)
    & (np.abs(data["ydot"]) < 0.05)
    & (np.abs(data["theta"]) < 0.02)
    & (np.abs(data["thetadot"]) < 0.02)
)
settled_indices = np.where(settled)[0]
if len(settled_indices) > 0:
    cutoff = settled_indices[0]
else:
    cutoff = len(data)
data = data[:cutoff]

t = data["time"]

fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
fig.suptitle("Rocket Sim Telemetry", fontsize=14)

# X position error
axes[0, 0].plot(t, data["x_err"], color="tab:blue")
axes[0, 0].axhline(0, color="gray", linewidth=0.5, linestyle="--")
axes[0, 0].set_ylabel("X Error (m)")
axes[0, 0].set_title("X Position Error")
axes[0, 0].grid(True, alpha=0.3)

# Y position error
axes[0, 1].plot(t, data["y_err"], color="tab:orange")
axes[0, 1].axhline(0, color="gray", linewidth=0.5, linestyle="--")
axes[0, 1].set_ylabel("Y Error (m)")
axes[0, 1].set_title("Y Position Error")
axes[0, 1].grid(True, alpha=0.3)

# Thrust
axes[1, 0].plot(t, data["thrust"], color="tab:red")
axes[1, 0].set_ylabel("Thrust (N)")
axes[1, 0].set_title("Thrust")
axes[1, 0].grid(True, alpha=0.3)

# Gimbal angle
axes[1, 1].plot(t, np.degrees(data["alpha"]), color="tab:green")
axes[1, 1].axhline(0, color="gray", linewidth=0.5, linestyle="--")
axes[1, 1].set_ylabel("Gimbal Angle (deg)")
axes[1, 1].set_title("Gimbal Angle")
axes[1, 1].grid(True, alpha=0.3)

# Theta (body angle)
axes[2, 0].plot(t, np.degrees(data["theta"]), color="tab:purple")
axes[2, 0].axhline(0, color="gray", linewidth=0.5, linestyle="--")
axes[2, 0].set_ylabel("Theta (deg)")
axes[2, 0].set_title("Body Angle")
axes[2, 0].set_xlabel("Time (s)")
axes[2, 0].grid(True, alpha=0.3)

# Velocities
axes[2, 1].plot(t, data["xdot"], label="vx", color="tab:blue")
axes[2, 1].plot(t, data["ydot"], label="vy", color="tab:orange")
axes[2, 1].axhline(0, color="gray", linewidth=0.5, linestyle="--")
axes[2, 1].set_ylabel("Velocity (m/s)")
axes[2, 1].set_title("Velocities")
axes[2, 1].set_xlabel("Time (s)")
axes[2, 1].legend()
axes[2, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
