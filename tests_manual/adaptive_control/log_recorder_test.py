import serial
import csv
import time
import os

# CONFIGURE HERE
PORT = 'COM4'           # Make sure this is the correct COM port
BAUDRATE = 115200
NUM_SAMPLES = 5000       # Change if necessary
# OUTPUT_FILE = 'motor_log212.csv'
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

def main():
    try:
        print(f"[INFO] Opening serial port {PORT} at {BAUDRATE} bps...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(1)  # ESP32 resets when the serial port is opened
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open {PORT}: {e}")
        return

    print("[INFO] Waiting for data...")
    lines = []
    header_found = False

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode('utf-8').strip()
            if not line:
                continue

            if not header_found:
                if line.startswith("pwm"):
                    header_found = True
                    lines.append(line.split(','))
                    print("[INFO] Header found.")
            else:
                parts = line.split(',')
                if len(parts) == 2:
                    lines.append(parts)
                    print(f"{parts}")
        except KeyboardInterrupt:
                ser.close()
                print("\nLogging stopped by user.")
                print(f"[INFO] Reading complete. Total: {len(lines)-1} samples.")
                break
        except Exception as e:
            print("[ERROR] Failed to read/decode line:", e)


    # Saving to CSV
    try:
        with open(OUTPUT_FILE, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(lines)
        print(f"[OK] File saved as '{OUTPUT_FILE}'.")
    except Exception as e:
        print("[ERROR] Failed to save file:", e)

if __name__ == '__main__':
    main()
