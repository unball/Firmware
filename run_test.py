import os
import shutil
import subprocess
import sys

FIRMWARE_SRC = "src/main.cpp"
BACKUP_SRC = "src/main.backup.cpp"
TESTS_DIR = "tests_manual"
AUTOMATED_TESTS = [
    "test_esp32",
    "test_dip_switch",
    "test_battery_voltage",
    "test_motors_and_encoders",
    "test_imu"
]

def run_single_test(test_name):
    test_path = os.path.join(TESTS_DIR, test_name, "main.cpp")
    if not os.path.exists(test_path):
        print(f"[SKIP] Test '{test_name}' not found.")
        return False

    print(f"\n=== Running test: {test_name} ===")

    shutil.copy(FIRMWARE_SRC, BACKUP_SRC)
    shutil.copy(test_path, FIRMWARE_SRC)

    try:
        result = subprocess.run(["pio", "run", "-t", "upload"], capture_output=True, text=True)
        print(result.stdout)
        if "[TEST RESULT] PASS" in result.stdout:
            print(f"[PASS] {test_name}")
            return True
        else:
            print(f"[FAIL] {test_name}")
            return False
    finally:
        shutil.copy(BACKUP_SRC, FIRMWARE_SRC)
        os.remove(BACKUP_SRC)

def run_all_tests():
    print("== Running full automated test suite ==")
    results = {}
    for test in AUTOMATED_TESTS:
        result = run_single_test(test)
        results[test] = result

    print("\n=== TEST SUMMARY ===")
    for test, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"{test}: {status}")

if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] in ["--all", "all"]:
        run_all_tests()
    elif len(sys.argv) == 2:
        run_single_test(sys.argv[1])
    else:
        print("Usage:")
        print("  python run_test.py test_name")
        print("  python run_test.py --all")
