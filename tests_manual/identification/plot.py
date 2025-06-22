import pandas as pd
import matplotlib.pyplot as plt

# Nome do arquivo com os dados
CSV_FILE = 'motor_log.csv'

# Lê o CSV em um DataFrame
df = pd.read_csv(CSV_FILE)

# Converte colunas para tipos numéricos, se necessário
df['time'] = pd.to_numeric(df['time'], errors='coerce')
df['w'] = pd.to_numeric(df['w'], errors='coerce')

# Remove linhas inválidas
df.dropna(subset=['time', 'w'], inplace=True)

# Plot
plt.figure(figsize=(10, 5))
plt.plot(df['time'], df['w'], label='w (rad/s)')
plt.xlabel('Time (s)')
plt.ylabel('Angular speed (rad/s)')
plt.title('Motor response to step input')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
