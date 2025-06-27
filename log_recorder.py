import serial
import re
import csv
import os
from datetime import datetime

# === Configuration ===
PORT = 'COM4'           # Change to your serial port (e.g. '/dev/ttyUSB0' on Linux)
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

# === Log regex pattern ===
log_pattern = re.compile(
    r"t:(?P<t>[\d.]+)\s+\| \[REF\] v:(?P<v_ref>[\d.-]+) \| w:(?P<w_ref>[\d.-]+)\s+\|\| \[MEAS\] v:(?P<v>[\d.-]+) \| w:(?P<w>[\d.-]+)\s+\|\| \[e_v, e_w\]: (?P<e_v>[\d.-]+), (?P<e_w>[\d.-]+)\s+\|\| \[u\] v_d:(?P<v_d>[\d.-]+) \| w_d:(?P<w_d>[\d.-]+)\s+\|\| \[ωL_r, ωR_r\]:(?P<omega_L_d>[\d.-]+),(?P<omega_R_d>[\d.-]+)\s+\|\| \[ωL_meas, ωR_meas\]:(?P<omega_L>[\d.-]+),(?P<omega_R>[\d.-]+)"
)

# === CSV header ===
fields = [
    "t", "v_ref", "w_ref", "v", "w", "e_v", "e_w",
    "v_d", "w_d", "omega_L_d", "omega_R_d", "omega_L", "omega_R"
]

# === Main Logging Loop ===
def main():
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser, open(OUTPUT_FILE, mode='w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fields)
        writer.writeheader()

        print(f"[{datetime.now()}] Logging started. Saving to '{OUTPUT_FILE}'...")

        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
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
