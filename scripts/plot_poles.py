import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import sys
import os
import json

sys.path.insert(0, os.path.dirname(__file__))
from compute_lqr_gains import PROFILES

profile = "fast"
meta_path = os.path.join(os.path.dirname(__file__), "..", "data", "sim_meta.txt")
if len(sys.argv) > 1:
    profile = sys.argv[1]
elif os.path.exists(meta_path):
    with open(meta_path) as f:
        profile = f.read().strip()
if profile not in PROFILES:
    print(f"Unknown profile '{profile}'. Choose: {list(PROFILES.keys())}")
    sys.exit(1)

# Load K from JSON
gains_path = os.path.join(os.path.dirname(__file__), "..", "data", f"lqr_gains_{profile}.json")
with open(gains_path) as f:
    gains_data = json.load(f)

K = np.array(gains_data["k_matrix"])
p = gains_data["system_parameters"]
m, L, g = p["mass_kg"], p["length_m"], p["gravity"]
l = L / 2.0
I = (1.0 / 12.0) * m * L**2

# Reconstruct linearized system matrices
A = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, g, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
], dtype=float)

B = np.array([
    [0,      0      ],
    [0,      g      ],
    [0,      0      ],
    [1.0/m,  0      ],
    [0,      0      ],
    [0,      l*g/I  ],
], dtype=float)

A_cl = A - B @ K
poles = np.linalg.eigvals(A_cl)

cfg = PROFILES[profile]
Q = np.diag(cfg["Q"])
R = np.diag(cfg["R"])

# ── Layout ────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(15, 7))
fig.suptitle(f"Closed-Loop Analysis — {profile.upper()} profile", fontsize=13)

ax = fig.add_axes([0.05, 0.08, 0.52, 0.84])   # poles plot
ax_text = fig.add_axes([0.60, 0.0, 0.40, 1.0]) # matrix text panel
ax_text.axis("off")

# ── Poles plot ────────────────────────────────────────────────────────────────
real_max = max(np.abs(poles.real).max() * 1.4, 1.0)
imag_max = max(np.abs(poles.imag).max() * 1.4, 1.0)

# Shade stable (LHP) and unstable (RHP) regions
ax.axvspan(-real_max, 0, color="tab:green", alpha=0.06, label="Stable (LHP)")
ax.axvspan(0, real_max, color="tab:red", alpha=0.06, label="Unstable (RHP)")
ax.axhline(0, color="black", linewidth=0.6)
ax.axvline(0, color="black", linewidth=0.6)

for p_val in poles:
    color = "tab:red" if p_val.real > 1e-8 else "tab:blue"
    ax.plot(p_val.real, p_val.imag, "x", markersize=10, markeredgewidth=2, color=color)
    label = f"  {p_val.real:.3f}{'+' if p_val.imag >= 0 else ''}{p_val.imag:.3f}j"
    ax.annotate(label, (p_val.real, p_val.imag), fontsize=7.5,
                xytext=(4, 4), textcoords="offset points")

ax.set_xlim(-real_max, real_max)
ax.set_ylim(-imag_max, imag_max)
ax.set_xlabel("Real Axis")
ax.set_ylabel("Imaginary Axis")
ax.set_title("Closed-Loop Poles  (A − BK)")
ax.legend(loc="upper right", fontsize=8)
ax.grid(True, alpha=0.25)

# ── Matrix text panel ─────────────────────────────────────────────────────────
state_labels = ["x", "ẋ", "y", "ẏ", "θ", "θ̇"]
control_labels = ["T", "α"]

def fmt_row(row, width=9):
    return "  ".join(f"{v:>{width}.4f}" for v in row)

def fmt_diag_matrix(mat, labels, name):
    n = mat.shape[0]
    lines = [f"{name}  (diagonal shown)"]
    for i in range(n):
        lines.append(f"  [{labels[i]}]  {mat[i, i]:.4f}")
    return "\n".join(lines)

def fmt_K(K, row_labels, col_labels):
    header = "K  (2×6)"
    col_header = "       " + "  ".join(f"{l:>9}" for l in col_labels)
    rows = []
    for i, rl in enumerate(row_labels):
        rows.append(f"  [{rl}]  " + fmt_row(K[i]))
    return "\n".join([header, col_header] + rows)

text_lines = []
text_lines.append(fmt_diag_matrix(Q, state_labels, "Q  (6×6)"))
text_lines.append("")
text_lines.append(fmt_diag_matrix(R, control_labels, "R  (2×2)"))
text_lines.append("")
text_lines.append(fmt_K(K, control_labels, state_labels))
text_lines.append("")
text_lines.append("Poles:")
for p_val in sorted(poles, key=lambda p: p.real):
    text_lines.append(f"  {p_val.real:+.4f}  {p_val.imag:+.4f}j")

ax_text.text(0.02, 0.97, "\n".join(text_lines),
             transform=ax_text.transAxes,
             fontsize=9, family="monospace",
             verticalalignment="top",
             bbox=dict(boxstyle="round,pad=0.5", facecolor="whitesmoke", edgecolor="lightgray"))

plt.show()
