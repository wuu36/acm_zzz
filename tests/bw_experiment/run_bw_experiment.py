import os
import sys
import yaml
import shutil
import subprocess

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
C_DIR = os.path.join(PROJECT_ROOT, "frameworkCodes", "c")
DAT_DIR = os.path.join(PROJECT_ROOT, "frameworkCodes", "dat")
RESULT_DIR = os.path.join(PROJECT_ROOT, "dat", "bw_experiment")
CONFIG_FILE = os.path.join(PROJECT_ROOT, "user_config.yaml")

CASES = {
    "1a_bw5":    {"speed_bw": 5,   "current_bw": 100, "label": "BW_speed=5, BW_current=100"},
    "1b_bw10":   {"speed_bw": 10,  "current_bw": 100, "label": "BW_speed=10, BW_current=100"},
    "1c_bw20":   {"speed_bw": 20,  "current_bw": 100, "label": "BW_speed=20, BW_current=100"},
    "1d_bw50":   {"speed_bw": 50,  "current_bw": 100, "label": "BW_speed=50, BW_current=100"},
    "2a_bw100":  {"speed_bw": 100, "current_bw": 100, "label": "BW_speed=100, BW_current=100 [violate]"},
    "2b_bw150":  {"speed_bw": 150, "current_bw": 100, "label": "BW_speed=150, BW_current=100 [violate]"},
    "3a_curr50": {"speed_bw": 20,  "current_bw": 50,  "label": "BW_speed=20, BW_current=50"},
}

def load_config():
    with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def save_config(config):
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

def run_one_case(case_name, case_params):
    print(f"\n{'='*60}")
    print(f"Running case: {case_name} - {case_params['label']}")
    print(f"{'='*60}")

    config = load_config()
    config['simulation']['sim.NUMBER_OF_STEPS'] = 8000
    config['control']['FOC.SPEED_BW'] = float(case_params['speed_bw'])
    config['control']['FOC.CURRENT_BW'] = float(case_params['current_bw'])
    config['test']['test.TLoad'] = 0.05
    save_config(config)

    result = subprocess.run(
        [sys.executable, os.path.join(PROJECT_ROOT, "super_config.py")],
        cwd=PROJECT_ROOT, capture_output=True, text=True, timeout=30
    )
    if result.returncode != 0:
        print(f"super_config.py failed: {result.stderr}")
        return False
    print(result.stdout)

    result = subprocess.run(
        ["make", "-B", "velocity_test.exe"],
        cwd=C_DIR, capture_output=True, text=True, timeout=30
    )
    if result.returncode != 0:
        print(f"make failed: {result.stderr}")
        return False

    exe_path = os.path.join(C_DIR, "velocity_test.exe")
    result = subprocess.run(
        [exe_path],
        cwd=C_DIR, capture_output=True, text=True, timeout=60
    )
    print(result.stdout[-500:] if len(result.stdout) > 500 else result.stdout)

    src_dat = os.path.join(DAT_DIR, "test_motor_velocity.dat")
    dst_dat = os.path.join(RESULT_DIR, f"bw_exp_{case_name}.dat")
    if os.path.exists(src_dat):
        os.makedirs(RESULT_DIR, exist_ok=True)
        shutil.copy2(src_dat, dst_dat)
        print(f"Data saved: {dst_dat}")
    else:
        print(f"Warning: no dat file at {src_dat}")
        return False

    return True

def main():
    os.makedirs(RESULT_DIR, exist_ok=True)

    results = {}
    for case_name, case_params in CASES.items():
        success = run_one_case(case_name, case_params)
        results[case_name] = success

    print(f"\n{'='*60}")
    print("Summary:")
    for name, success in results.items():
        print(f"  {name}: {'OK' if success else 'FAILED'}")

    config = load_config()
    config['simulation']['sim.NUMBER_OF_STEPS'] = 4000
    config['control']['FOC.SPEED_BW'] = 10.0
    config['control']['FOC.CURRENT_BW'] = 100.0
    config['test']['test.TLoad'] = 0.0
    save_config(config)
    subprocess.run([sys.executable, os.path.join(PROJECT_ROOT, "super_config.py")], cwd=PROJECT_ROOT, capture_output=True)
    subprocess.run(["make", "-B", "velocity_test.exe"], cwd=C_DIR, capture_output=True)
    print("Config restored to defaults.")

if __name__ == "__main__":
    main()