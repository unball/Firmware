import serial
import matplotlib.pyplot as plt
import time

# Configurar a porta serial (ajuste conforme necessário)
serial_port = '/dev/ttyUSB0'  # ou 'COM3' no Windows
baud_rate = 115200

# Inicializar a comunicação serial
ser = serial.Serial(serial_port, baud_rate)
time.sleep(2)  # Aguarde a inicialização do Arduino

# Listas para armazenar os valores lidos
data1 = []
data2 = []
data3 = []

plt.ion()  # Habilitar o modo interativo

fig, ax = plt.subplots()
line1, = ax.plot(data1, label='Variável 1')
line2, = ax.plot(data2, label='Variável 2')
line3, = ax.plot(data3, label='Variável 3')
plt.legend()

# Função para atualizar o gráfico
def update_plot():
    line1.set_ydata(data1)
    line2.set_ydata(data2)
    line3.set_ydata(data3)
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.01)

buffer_size = 50  # Tamanho do buffer para acumular leituras antes de atualizar o gráfico

try:
    while True:
        if ser.in_waiting > 0:
            # Ler linha do Arduino
            line = ser.readline().decode('utf-8').strip()
            # Dividir a linha em três variáveis float
            values = line.split(',')
            if len(values) == 3:
                try:
                    var1 = float(values[0])
                    var2 = float(values[1])
                    var3 = float(values[2])
                    
                    # Adicionar os valores às listas
                    data1.append(var1)
                    data2.append(var2)
                    data3.append(var3)
                    
                    # Manter as listas com tamanho fixo
                    if len(data1) > 100:
                        data1.pop(0)
                        data2.pop(0)
                        data3.pop(0)
                    
                    # Atualizar o gráfico a cada 'buffer_size' leituras
                    if len(data1) % buffer_size == 0:
                        update_plot()
                except ValueError:
                    print(f"Erro ao converter os valores: {line}")

except KeyboardInterrupt:
    print("Leitura interrompida pelo usuário")

finally:
    ser.close()
    print("Porta serial fechada")
