import os
import shutil
import subprocess
import sys
import time

AUTOMATED_TESTS = [
    "test_esp32",
    "test_dip_switch",
    "test_motors_and_encoders",
    "test_imu"
]

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
            text=True,
            encoding='utf-8',
            errors='replace'
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
            return True
        else:
            print("❌ Test FAILED")
            return False

    except Exception as e:
        print("⚠️ Failed to read serial output:", e)
        return False

def run_test(test_name):
    TEST_PATH = os.path.join("tests_manual", test_name, "main.cpp")
    SRC_PATH = os.path.join("src", "main.cpp")
    BACKUP_PATH = os.path.join("tests_manual", "backup", "main.cpp")

    if not os.path.exists(TEST_PATH):
        print(f"❌ Test '{test_name}' not found!\n")
        list_available_tests()
        return False

    os.makedirs(os.path.dirname(BACKUP_PATH), exist_ok=True)
    print("🔄 Backing up src/main.cpp...")
    shutil.copyfile(SRC_PATH, BACKUP_PATH)

    print(f"🧪 Running test: {test_name}")
    shutil.copyfile(TEST_PATH, SRC_PATH)

    pio_cmd = find_pio_executable()
    print("🚀 Uploading to device...")
    upload = subprocess.run([pio_cmd, "run", "-t", "upload"])

    print("♻️ Restoring original main.cpp...")
    shutil.copyfile(BACKUP_PATH, SRC_PATH)
    os.remove(BACKUP_PATH)

    if upload.returncode == 0:
        print("✅ Upload successful. Reading output...")
        return read_serial_output(timeout=15)
    else:
        print("⚠️ Upload failed. main.cpp was restored anyway.")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python run_test.py <test_name>")
        print("  python run_test.py --all\n")
        list_available_tests()
        sys.exit(1)

    if sys.argv[1] in ["--all", "all"]:
        print("🔁 Running all automated tests...\n")
        results = {}
        for test in AUTOMATED_TESTS:
            passed = run_test(test)
            results[test] = passed
        print("\n📋 Summary:")
        for test, passed in results.items():
            print(f"{test}: {'PASS' if passed else 'FAIL'}")
    else:
        run_test(sys.argv[1])
