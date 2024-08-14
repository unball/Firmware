import serial
import time
import csv

def collect_and_save_data(port, baudrate, duration, filename):
    ser = serial.Serial(port, baudrate)
    time.sleep(2)  # Aguarde a inicialização do Arduino
    start_time = time.time()
    data1 = []

    try:
        while time.time() - start_time < duration:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                values = line.split(',')
                if len(values) == 1:
                    try:
                        var1 = float(values[0])
                        data1.append(var1)
                    except ValueError:
                        print(f"Erro ao converter os valores: {line}")
        ser.close()

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Data1'])
            for value in data1:
                writer.writerow([value])

        print(f"Data saved to {filename}")

    except KeyboardInterrupt:
        print("Leitura interrompida pelo usuário")
        ser.close()
        print("Porta serial fechada")

# Use the function to collect data twice
collect_and_save_data('/dev/ttyUSB0', 115200, 10, 'data3.csv')
