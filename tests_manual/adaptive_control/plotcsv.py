import matplotlib.pyplot as plt
import pandas as pd

# === CONFIGURATION ===
CSV_FILE = 'log_output_14.csv'  # Altere para o nome do seu arquivo .csv
PLOT_VARS = ['r', 'ommega', 'ommega_m', 'u', 'theta1', 'theta2', 'e']  # Variáveis a serem plotadas
TIME_VAR = 't'  # Nome da variável de tempo no CSV

# === LOAD CSV ===
df = pd.read_csv(CSV_FILE)

# === PLOT EACH VARIABLE ===
fig, axs = plt.subplots(len(PLOT_VARS), 1, figsize=(10, 2.5 * len(PLOT_VARS)), sharex=True)
if len(PLOT_VARS) == 1:
    axs = [axs]

for ax, var in zip(axs, PLOT_VARS):
    ax.plot(df[TIME_VAR], df[var], label=var)
    ax.set_ylabel(var)
    ax.grid(True)
    ax.legend(loc="upper right")

axs[-1].set_xlabel("Time [s]")
plt.tight_layout()
plt.show()
