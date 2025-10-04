import serial
import re
import csv
import os
from datetime import datetime
import time
import threading
import msvcrt  # para capturar teclas no Windows

# === Configuration ===
PORT = 'COM3'           # Change to your serial port (e.g. '/dev/ttyUSB0' on Linux)
BAUDRATE = 115200
BASE_FILENAME = 'log_output'
EXTENSION = '.csv'

# === Generate unique output filename ===
def generate_unique_filename(base, ext):
    counter = 1
    filename = f"{base}{ext}"
    while os.path.exists(filename):
        filename = f"{base}_{counter}{ext}"
        counter += 1
    return filename

# === Ensure "logs" folder exists and generate output file inside it ===
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)  # cria a pasta se não existir
OUTPUT_FILE = os.path.join(LOG_DIR, generate_unique_filename(BASE_FILENAME, EXTENSION))


# === Log regex pattern ===
log_pattern = re.compile(
    r"t:(?P<t>[\d.]+)\s+\| \[REF\] v:(?P<v_ref>[\d.-]+) \| w:(?P<w_ref>[\d.-]+)\s+\|\| \[MEAS\] v:(?P<v>[\d.-]+) \| w:(?P<w>[\d.-]+)\s+\|\| \[e_v, e_w\]: (?P<e_v>[\d.-]+), (?P<e_w>[\d.-]+)\s+\|\| \[u\] v_d:(?P<v_d>[\d.-]+) \| w_d:(?P<w_d>[\d.-]+)\s+\|\| \[ωL_r, ωR_r\]:(?P<omega_L_d>[\d.-]+),(?P<omega_R_d>[\d.-]+)\s+\|\| \[ωL_meas, ωR_meas\]:(?P<omega_L>[\d.-]+),(?P<omega_R>[\d.-]+)"
)

log_pattern_transmitter = re.compile(
    r"t:(?P<t>\d+), v_ref:(?P<v_ref>[-\d.]+), w_ref:(?P<w_ref>[-\d.]+), v:(?P<v>[-\d.]+), w:(?P<w>[-\d.]+), "
    r"omega_L:(?P<omega_L>[-\d.]+), omega_R:(?P<omega_R>[-\d.]+), u_L:(?P<u_L>[-\d.]+), u_R:(?P<u_R>[-\d.]+), "
    r"w_L:(?P<w_L>[-\d.]+), w_R:(?P<w_R>[-\d.]+), theta1_L:(?P<theta1_L>[-\d.]+), theta2_L:(?P<theta2_L>[-\d.]+), "
    r"theta1_R:(?P<theta1_R>[-\d.]+), theta2_R:(?P<theta2_R>[-\d.]+), e_L:(?P<e_L>[-\d.]+), e_R:(?P<e_R>[-\d.]+)"
)


# === CSV header ===
fields = [
    "t", "v_ref", "w_ref", "v", "w", "e_v", "e_w",
    "v_d", "w_d", "omega_L_d", "omega_R_d", "omega_L", "omega_R"
]

fields_transmitter = [
    "t", "v_ref", "w_ref", "v", "w",
    "omega_L", "omega_R", "u_L", "u_R",
    "w_L", "w_R",
    "theta1_L", "theta2_L", "theta1_R", "theta2_R",
    "e_L", "e_R"
]

# === Variáveis globais para controle ===
v_ref = 0.0
w_ref = 0.0
step_v = 0.05
step_w = 0.2

# === Thread de controle por teclado (WASD) ===
def keyboard_control(ser):
    global v_ref, w_ref
    print("🔁 Keyboard control enabled. Use W A S D (space = stop, Q = quit)")

    while True:
        if msvcrt.kbhit():  # verifica se uma tecla foi pressionada
            key = msvcrt.getch().decode('utf-8').lower()

            if key == 'q':
                print("[INFO] Exiting by user command.")
                os._exit(0)
            elif key == 'w':
                v_ref += step_v
                v_ref = max(min(v_ref, 1.5), -1.5)   # limita v_ref entre -1.5 e +1.5
            elif key == 's':
                v_ref -= step_v
                v_ref = max(min(v_ref, 1.5), -1.5)
            elif key == 'a':
                w_ref += step_w
                w_ref = max(min(w_ref, 30.0), -30.0) # limita w_ref entre -30 e +30
            elif key == 'd':
                w_ref -= step_w
                w_ref = max(min(w_ref, 30.0), -30.0)
            elif key == ' ':
                v_ref = 0.0
                w_ref = 0.0             

            checksum = int(v_ref + w_ref)
            message = f"{v_ref:.2f},{w_ref:.2f}\n"
            try:
                ser.write(message.encode())
                print(f"[TX] {message.strip()}")
            except Exception as e:
                print(f"[ERROR] Failed to send: {e}")
                break

def send_stop(ser):
    try:
        message = "0.00,0.00\n"
        ser.write(message.encode())
        print("[TX] STOP command sent (v=0, w=0)")
    except Exception as e:
        print(f"[ERROR] Failed to send stop command: {e}")


# === Main Logging Loop ===
def main():
    print(f"[{datetime.now()}] Waiting for port {PORT}...")

    ser = None
    while True:
        try:
            ser = serial.Serial(PORT, BAUDRATE, timeout=1)
            print(f"[{datetime.now()}] Connected to {PORT}")
            break
        except serial.SerialException:
            pass  # Continua tentando até conectar

    # Inicia a thread do teclado
    # threading.Thread(target=keyboard_control, args=(ser,), daemon=True).start()

    with ser, open(OUTPUT_FILE, mode='w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fields_transmitter)
        writer.writeheader()

        print(f"[{datetime.now()}] Logging started. Saving to '{OUTPUT_FILE}'...")

        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                match = log_pattern_transmitter.match(line)
                if match:
                    writer.writerow(match.groupdict())
                    print(line)

            except serial.SerialException:
                print(f"\n[{datetime.now()}] Logging stopped by disconnection.")
                send_stop(ser)
                break

            except KeyboardInterrupt:
                print("\nLogging stopped by user.")
                send_stop(ser)
                break

            except Exception as e:
                send_stop(ser)
                print(f"Error: {e}")

        print(f"[{datetime.now()}] Log file saved to '{OUTPUT_FILE}'.")


if __name__ == '__main__':
    main()
