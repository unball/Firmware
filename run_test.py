import os
import shutil
import subprocess
import sys
import time

def find_pio_executable():
    try:
        subprocess.run(["pio", "--version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return "pio"
    except FileNotFoundError:
        pass
    user_home = os.path.expanduser("~")
    fallback_path = os.path.join(user_home, ".platformio", "penv", "Scripts", "pio.exe")
    if os.path.exists(fallback_path):
        return fallback_path
    print("❌ PlatformIO CLI (pio) not found.")
    sys.exit(1)

def list_available_tests():
    print("📋 Available tests:\n")
    test_dir = os.path.join("tests_manual")
    for name in os.listdir(test_dir):
        path = os.path.join(test_dir, name)
        if os.path.isdir(path) and os.path.isfile(os.path.join(path, "main.cpp")):
            print(f"• {name}")
    print()

def read_serial_output(timeout=15):
    try:
        print(f"🔍 Reading serial output for {timeout} seconds (manual timeout)...")

        monitor = subprocess.Popen(
            ["pio", "device", "monitor", "--rts", "0", "--dtr", "1"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        start = time.time()
        output = ""
        while time.time() - start < timeout:
            line = monitor.stdout.readline()
            if not line:
                continue  # keep waiting
            print(line.strip())
            output += line
            if "[TEST RESULT]" in line:
                break  # found result early

        monitor.terminate()

        if "[TEST RESULT] PASS" in output:
            print("✅ Test PASSED")
        else:
            print("❌ Test FAILED")

    except Exception as e:
        print("⚠️ Failed to read serial output:", e)

if len(sys.argv) < 2:
    print("Usage: python run_test.py <test_name>\n")
    list_available_tests()
    sys.exit(1)

TEST_NAME = sys.argv[1]
TEST_PATH = os.path.join("tests_manual", TEST_NAME, "main.cpp")
SRC_PATH = os.path.join("src", "main.cpp")
BACKUP_PATH = os.path.join("tests_manual", "backup", "main.cpp")

if not os.path.exists(TEST_PATH):
    print(f"❌ Test '{TEST_NAME}' not found!\n")
    list_available_tests()
    sys.exit(1)

os.makedirs(os.path.dirname(BACKUP_PATH), exist_ok=True)
print("🔄 Backing up src/main.cpp...")
shutil.copyfile(SRC_PATH, BACKUP_PATH)

print(f"🧪 Running test: {TEST_NAME}")
shutil.copyfile(TEST_PATH, SRC_PATH)

pio_cmd = find_pio_executable()
print("🚀 Uploading to device...")
upload = subprocess.run([pio_cmd, "run", "-t", "upload"])

print("♻️ Restoring original main.cpp...")
shutil.copyfile(BACKUP_PATH, SRC_PATH)
os.remove(BACKUP_PATH)

if upload.returncode == 0:
    print("✅ Upload successful. Reading output...")
    read_serial_output(timeout=10)
else:
    print("⚠️ Upload failed. main.cpp was restored anyway.")
