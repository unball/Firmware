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

# Duração da coleta de dados em segundos
duration = 10
start_time = time.time()

try:
    while time.time() - start_time < duration:
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
                except ValueError:
                    print(f"Erro ao converter os valores: {line}")

    # Fechar a porta serial após a coleta de dados
    ser.close()

    # Plotar os dados coletados
    plt.figure(figsize=(10, 5))
    plt.plot(data1, label='Variável 1')
    plt.plot(data2, label='Variável 2')
    plt.plot(data3, label='Variável 3')
    plt.legend()
    plt.xlabel('Amostras')
    plt.ylabel('Valores')
    plt.title('Dados Coletados do Arduino')
    plt.show()

except KeyboardInterrupt:
    print("Leitura interrompida pelo usuário")
    ser.close()
    print("Porta serial fechada")
