import os
import shutil
import subprocess
import sys

def find_pio_executable():
    # Try using default command
    try:
        subprocess.run(["pio", "--version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return "pio"
    except FileNotFoundError:
        pass

    # Try default PlatformIO path on Windows
    user_home = os.path.expanduser("~")
    fallback_path = os.path.join(user_home, ".platformio", "penv", "Scripts", "pio.exe")
    if os.path.exists(fallback_path):
        return fallback_path

    print("❌ PlatformIO CLI (pio) not found.")
    print("Please make sure PlatformIO is installed and available in your PATH.")
    sys.exit(1)

# Get test name from argument
if len(sys.argv) < 2:
    print("Usage: python run_test.py <test_name>")
    sys.exit(1)

TEST_NAME = sys.argv[1]
TEST_PATH = os.path.join("tests_manual", TEST_NAME, "main.cpp")
SRC_PATH = os.path.join("src", "main.cpp")
BACKUP_PATH = os.path.join("tests_manual", "backup", "main.cpp")

if not os.path.exists(TEST_PATH):
    print(f"❌ Test '{TEST_NAME}' not found!")
    sys.exit(1)

# Backup current main.cpp
os.makedirs(os.path.dirname(BACKUP_PATH), exist_ok=True)
print("🔄 Backing up src/main.cpp...")
shutil.copyfile(SRC_PATH, BACKUP_PATH)

# Copy test file to src/
print(f"🧪 Running test: {TEST_NAME}")
shutil.copyfile(TEST_PATH, SRC_PATH)

# Upload to device using PlatformIO
pio_cmd = find_pio_executable()
print("🚀 Uploading to device...")
result = subprocess.run([pio_cmd, "run", "-t", "upload"])

# Restore original main.cpp
print("♻️ Restoring original main.cpp...")
shutil.copyfile(BACKUP_PATH, SRC_PATH)
os.remove(BACKUP_PATH)

if result.returncode == 0:
    print("✅ Upload successful. main.cpp restored.")
else:
    print("⚠️ Upload failed. main.cpp was restored anyway.")
