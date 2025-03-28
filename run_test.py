import os
import shutil
import subprocess
import sys

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

os.makedirs(os.path.dirname(BACKUP_PATH), exist_ok=True)

# Backup
print("🔄 Backing up src/main.cpp...")
shutil.copyfile(SRC_PATH, BACKUP_PATH)

# Substitui por teste
print(f"🧪 Running test: {TEST_NAME}")
shutil.copyfile(TEST_PATH, SRC_PATH)

# Upload
print("🚀 Uploading to device...")
result = subprocess.run(["pio", "run", "-t", "upload"])

# Restaurar main.cpp original
print("♻️ Restoring original main.cpp...")
shutil.copyfile(BACKUP_PATH, SRC_PATH)
os.remove(BACKUP_PATH)

if result.returncode == 0:
    print("✅ Upload successful. main.cpp restored.")
else:
    print("⚠️ Upload failed. main.cpp still restored, but test may not have run.")
