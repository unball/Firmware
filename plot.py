import pandas as pd
import matplotlib.pyplot as plt
import re
import os

# === Arquivos de log e rótulos ===
log_files = [
    "log_output.csv"
]

labels = [
    "MRAC"
]

# === Variáveis presentes no CSV gerado pelo script de log ===
all_variables = [
    "t", "v_ref", "w_ref", "v", "w", "e_v", "e_w",
    "v_d", "w_d", "omega_L_d", "omega_R_d", "omega_L", "omega_R"
]

# === Variáveis que deseja visualizar ===
variables_to_plot = ["v", "v_ref", "e_v", "omega_L", "omega_L_d"]

# === Cores para as curvas ===
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']

# === Subplots ===
fig, axs = plt.subplots(len(variables_to_plot), 1, figsize=(10, 3 * len(variables_to_plot)), sharex=True)
fig.subplots_adjust(hspace=0.4)
if len(variables_to_plot) == 1:
    axs = [axs]  # Garante lista mesmo para um único subplot

for idx, (file, label) in enumerate(zip(log_files, labels)):
    if not os.path.exists(file):
        print(f"[!] Arquivo não encontrado: {file}")
        continue

    try:
        df = pd.read_csv(file)
    except Exception as e:
        print(f"[!] Erro ao ler {file}: {e}")
        continue

    for i, var in enumerate(variables_to_plot):
        if var not in df.columns:
            print(f"[!] Variável '{var}' não encontrada em {file}")
            continue
        axs[i].plot(df["t"], df[var], label=label, color=colors[idx % len(colors)])

# Estética dos gráficos
for i, var in enumerate(variables_to_plot):
    axs[i].set_ylabel(var)
    axs[i].grid(True)
    axs[i].legend()

axs[-1].set_xlabel("Tempo [s]")
fig.suptitle("Gráfico dos Logs - State Space + MRAC", fontsize=14)
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()
