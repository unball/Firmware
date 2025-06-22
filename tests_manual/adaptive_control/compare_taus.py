import pandas as pd
import matplotlib.pyplot as plt
import re

# === Arquivos e rótulos ===
log_files = [
    "log_gamma_n5c.csv",
    "log_gamma_0051.csv",
    "log_err.csv"
]

labels = [
    "no control",
    "control",
    "control"
]

# === Variáveis disponíveis no log ===
all_variables = ["t", "r", "y", "ym", "u", "theta1", "theta2", "e"]

# === Variáveis a serem exibidas (altere aqui) ===
variables_to_plot = ["y", "ym", "u", "theta1", "theta2", "e"]  # <- escolha só as que quiser

# === Cores (ajustadas automaticamente)
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']

# === Subplots
fig, axs = plt.subplots(len(variables_to_plot), 1, figsize=(10, 3 * len(variables_to_plot)), sharex=True)
fig.subplots_adjust(hspace=0.4)
if len(variables_to_plot) == 1:
    axs = [axs]  # Garante lista mesmo para um único subplot

for idx, (file, label) in enumerate(zip(log_files, labels)):
    data = []
    try:
        with open(file, 'r') as f:
            for line in f:
                values = re.findall(r'-?\d+\.\d+', line)
                if len(values) == 8:
                    data.append([float(v) for v in values])
    except FileNotFoundError:
        print(f"[!] Arquivo não encontrado: {file}")
        continue

    if not data:
        print(f"[!] Nenhum dado válido em {file}")
        continue

    df = pd.DataFrame(data, columns=all_variables)

    # Plotar cada variável selecionada
    for i, var in enumerate(variables_to_plot):
        axs[i].plot(df["t"], df[var], label=label, color=colors[idx % len(colors)])

# Estética dos gráficos
for i, var in enumerate(variables_to_plot):
    axs[i].set_ylabel(var)
    axs[i].grid(True)
    axs[i].legend()
axs[-1].set_xlabel("Tempo [s]")
fig.suptitle("Comparação de Resposta - MRAC", fontsize=14)
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()
