import matplotlib.pyplot as plt
import pandas as pd
import argparse
import sys
import os
import re

# === ARGUMENTS ===
parser = argparse.ArgumentParser(description="Plot log in 'label:value' format.")
parser.add_argument('--nome', type=str, required=True, help="Arquivo de log no formato t:..., r:..., etc.")
args = parser.parse_args()

FILENAME = args.nome

if not os.path.exists(FILENAME):
    print(f"[ERROR] Arquivo '{FILENAME}' não encontrado.")
    sys.exit(1)

# === REGEX ===
pattern = re.compile(
    r"t:(?P<t>[\d\.\-eE]+),\s*"
    r"r:(?P<r>[\d\.\-eE]+),\s*"
    r"omega:(?P<omega>[\d\.\-eE]+),\s*"
    r"omega_m:(?P<omega_m>[\d\.\-eE]+),\s*"
    r"u:(?P<u>[\d\.\-eE]+),\s*"
    r"theta1:(?P<theta1>[\d\.\-eE]+),\s*"
    r"theta2:(?P<theta2>[\d\.\-eE]+),\s*"
    r"e:(?P<e>[\d\.\-eE]+)"
)

# === READ AND PARSE LOG FILE ===
data = {
    "t": [], "r": [], "omega": [], "omega_m": [],
    "u": [], "theta1": [], "theta2": [], "e": []
}

with open(FILENAME, 'r') as f:
    for line in f:
        match = pattern.match(line.strip())
        if match:
            for key, value in match.groupdict().items():
                data[key].append(float(value))

# === CONVERT TO DATAFRAME ===
df = pd.DataFrame(data)

# === PLOT ===
PLOT_VARS = ['omega', 'u', 'theta1', 'theta2', 'e']
TIME_VAR = 't'
OVERLAYS = {'omega': 'omega_m'}  # Tracejada

fig, axs = plt.subplots(len(PLOT_VARS), 1, figsize=(10, 2.5 * len(PLOT_VARS)), sharex=True)
if len(PLOT_VARS) == 1:
    axs = [axs]

for ax, var in zip(axs, PLOT_VARS):
    ax.plot(df[TIME_VAR], df[var], label=var, linewidth=1.8)
    
    if var in OVERLAYS:
        overlay = OVERLAYS[var]
        if overlay in df.columns:
            ax.plot(df[TIME_VAR], df[overlay], '--', label=overlay, linewidth=1.5)

    ax.set_ylabel(var)
    ax.grid(True)
    ax.legend(loc="upper right")

axs[-1].set_xlabel("Time [s]")
plt.suptitle(f"Plot from '{FILENAME}'", fontsize=14)
plt.tight_layout()
plt.show()
