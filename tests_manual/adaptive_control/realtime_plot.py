import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import matplotlib.ticker as ticker


# === CONFIGURAÇÕES DO USUÁRIO ===
SERIAL_PORT = 'COM4'       # Altere para sua porta serial
BAUD_RATE = 115200
PLOT_VARS = ['ommega', 'u', 'theta1', 'theta2', 'e']  # Variáveis a serem exibidas
WINDOW_SIZE = 900  # Número de amostras visíveis por gráfico

# === CONFIGURAÇÃO SERIAL ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# === DADOS EM MEMÓRIA ===
data = {var: deque(maxlen=WINDOW_SIZE) for var in ['t'] + PLOT_VARS}

# === CONFIGURAÇÃO DOS SUBPLOTS ===
num_vars = len(PLOT_VARS)
fig, axs = plt.subplots(num_vars, 1, figsize=(8, 3 * num_vars), sharex=True)
if num_vars == 1:
    axs = [axs]  # Garante que axs seja sempre uma lista

lines = {}
for ax, var in zip(axs, PLOT_VARS):
    line, = ax.plot([], [], label=var)
    lines[var] = line
    ax.set_ylabel(var)
    ax.legend(loc="upper right")

    # Format tick labels and density
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
    ax.yaxis.set_major_locator(ticker.MaxNLocator(nbins=8))
    ax.xaxis.set_major_locator(ticker.MaxNLocator(nbins=10))


def parse_line(line):
    parsed = {}
    try:
        line = line.strip().replace(' ', '')
        parts = line.split(',')
        for p in parts:
            if ':' in p:
                key, val = p.split(':')
                parsed[key] = float(val)
    except:
        pass
    return parsed

def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore')
        parsed = parse_line(line)
        if 't' in parsed:
            data['t'].append(parsed['t'])
            for var in PLOT_VARS:
                data[var].append(parsed.get(var, 0.0))  # Se não houver valor, adiciona 0

    for ax, var in zip(axs, PLOT_VARS):
        lines[var].set_data(data['t'], data[var])
        if data['t']:
            ax.set_xlim(max(0, data['t'][0]), data['t'][-1])
            ax.relim()
            ax.autoscale_view()

    return list(lines.values())

ani = FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()

ser.close()
