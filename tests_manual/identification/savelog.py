import serial

port = 'COM3'           # Change as needed
baud = 115200
output_file = 'motor_log.csv'

ser = serial.Serial(port, baud, timeout=1)

start_logging = False

with open(output_file, 'w') as f:
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            print(line)

            # Wait for CSV header to start logging
            if not start_logging:
                if line.startswith("time_us,pwm,w"):
                    start_logging = True
                    f.write(line + '\n')
                continue

            if "Missed samples" in line:
                break

            f.write(line + '\n')

        except KeyboardInterrupt:
            break

ser.close()
