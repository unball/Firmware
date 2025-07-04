import serial
import re
import csv
import os
import time
from datetime import datetime

# === Configuration ===
PORT = 'COM4'           # Set this to your serial port
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

OUTPUT_FILE = generate_unique_filename(BASE_FILENAME, EXTENSION)

# === CSV header ===
fields = [
    "t", "r", "ommega", "ommega_m", "u", "theta1", "theta2", "e"
]

# === Regex pattern to match the logging format ===
log_pattern = re.compile(
    r"t:(?P<t>[\d\.\-eE]+),\s*"
    r"r:(?P<r>[\d\.\-eE]+),\s*"
    r"ommega:(?P<ommega>[\d\.\-eE]+),\s*"
    r"ommega_m:(?P<ommega_m>[\d\.\-eE]+),\s*"
    r"u:(?P<u>[\d\.\-eE]+),\s*"
    r"theta1:(?P<theta1>[\d\.\-eE]+),\s*"
    r"theta2:(?P<theta2>[\d\.\-eE]+),\s*"
    r"e:(?P<e>[\d\.\-eE]+)"
)

# === Main Logging Loop ===
def main():
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser, open(OUTPUT_FILE, mode='w', newline='') as csvfile:
        # # === Force ESP reset ===
        # ser.setRTS(True)    # IO0 HIGH → não entrar em bootloader
        # ser.setDTR(False)   # EN LOW   → força reset
        # time.sleep(1)

        # ser.setDTR(True)    # EN HIGH → libera reset
        # time.sleep(0.1)
        writer = csv.DictWriter(csvfile, fieldnames=fields)
        writer.writeheader()

        print(f"[{datetime.now()}] Logging started. Saving to '{OUTPUT_FILE}'...")

        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                match = log_pattern.match(line)
                if match:
                    writer.writerow(match.groupdict())
                    print(line)
            except KeyboardInterrupt:
                print("\nLogging stopped by user.")
                break
            except Exception as e:
                print(f"Error: {e}")

if __name__ == '__main__':
    main()
