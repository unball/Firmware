import matplotlib.pyplot as plt
import csv

def read_data(filename):
    data1 = []
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # skip the header
        for row in reader:
            data1.append(float(row[0]))
    return data1

# Read the data
data1 = read_data('data3.csv')
data2 = read_data('data1.csv')

# Plot the data
plt.figure(figsize=(10, 5))
plt.plot(data1, label='Coleta com Timer')
plt.plot(data2, label='Coleta com timer primeira vez')
plt.legend()
plt.xlabel('Amostras')
plt.ylabel('Velocidade (rad/s)')
plt.title('Dados Coletados do ESP32 em Duas Sessões')
plt.show()
