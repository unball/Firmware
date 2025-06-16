import serial
import csv
import time

# CONFIGURE HERE
PORT = 'COM3'           # Make sure this is the correct COM port
BAUDRATE = 115200
NUM_SAMPLES = 500       # Change if necessary
OUTPUT_FILE = 'motor_log.csv'

def main():
    try:
        print(f"[INFO] Opening serial port {PORT} at {BAUDRATE} bps...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # ESP32 resets when the serial port is opened
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open {PORT}: {e}")
        return

    print("[INFO] Waiting for data...")
    lines = []
    header_found = False

    while len(lines) < NUM_SAMPLES + 1:  # +1 to include the header
        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode('utf-8').strip()
            if not line:
                continue

            if not header_found:
                if line.startswith("time_s"):
                    header_found = True
                    lines.append(line.split(','))
                    print("[INFO] Header found.")
            else:
                parts = line.split(',')
                if len(parts) == 3:
                    lines.append(parts)
                    print(f"[{len(lines)-1}/{NUM_SAMPLES}] {parts}")

        except Exception as e:
            print("[ERROR] Failed to read/decode line:", e)

    ser.close()
    print(f"[INFO] Reading complete. Total: {len(lines)-1} samples.")

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
