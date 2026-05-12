# type: ignore
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from super_config import SuperConfig
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_PATH = os.path.join(os.path.dirname(__file__), "..")
SIM_DAT = os.path.join(BASE_PATH, "frameworkCodes", "dat")
OUT_PATH = os.path.join(BASE_PATH, "dat", "foc_current_analysis")
os.makedirs(OUT_PATH, exist_ok=True)

print("=" * 80)
print("FOC电流模式分析")
print("=" * 80)

results = {}

def run_foc_test(cmd_iq, steps=20000, tag=""):
    cfg = SuperConfig("SEW100W")
    cfg.load_motor_library()
    cfg.load_user_config()
    p = cfg.get_motor_params("SEW100W")
    cfg.user_config["simulation"]["sim.CL_TS"] = 5e-5
    cfg.user_config["simulation"]["sim.NUMBER_OF_STEPS"] = steps
    cfg.user_config["simulation"]["sim.MODE_SELECT"] = 3  # FOC电流模式
    cfg.user_config["test"]["test.cmd_speed"] = 0.0
    cfg.user_config["test"]["test.cmd_iQ"] = cmd_iq
    cfg.user_config["test"]["test.cmd_uD"] = 0.0
    cfg.user_config["test"]["test.cmd_uQ"] = 0.0
    cfg.user_config["test"]["test.TLoad"] = 0.0
    cfg.user_config["motor"]["motor.npp"] = p["npp"]
    cfg.user_config["motor"]["motor.IN"] = p["IN"]
    cfg.user_config["motor"]["motor.R"] = p["R"]
    cfg.user_config["motor"]["motor.Ld"] = p["Ld"]
    cfg.user_config["motor"]["motor.Lq"] = p["Lq"]
    cfg.user_config["motor"]["motor.KE"] = p["KE"]
    cfg.user_config["motor"]["motor.Js"] = p["Js"]
    cfg.user_config["motor"]["motor.Vdc"] = p["Vdc"]
    cfg.update_super_config()
    cfg.compile_simulation("foc_test")
    cfg.run_simulation("foc_test")

    df = pd.read_csv(os.path.join(SIM_DAT, "test_motor_foc.dat"))
    import shutil
    shutil.copy2(os.path.join(SIM_DAT, "test_motor_foc.dat"), os.path.join(OUT_PATH, f"{tag}.dat"))

    omega_end = df["varOmega"].iloc[-10:].mean()
    iq_end = df["iQ"].iloc[-10:].mean()
    return {"tag": tag, "omega_end": omega_end, "iq_end": iq_end, "df": df}

for cmd_iq in [0.5, 1.0, 2.0, 3.5, 5.0]:
    tag = f"iq{cmd_iq}"
    print(f"  Running FOC cmd_iQ={cmd_iq}A...")
    results[tag] = run_foc_test(cmd_iq=cmd_iq, steps=20000, tag=tag)

fig, axes = plt.subplots(4, 1, figsize=(10, 14))

for tag in sorted(results.keys()):
    r = results[tag]
    iq_val = tag.replace("iq", "")
    axes[0].plot(r["df"]["time"], r["df"]["varOmega"], linewidth=1, label=f"cmd_iQ={iq_val}A")
    axes[1].plot(r["df"]["time"], r["df"]["iQ"], linewidth=1, label=f"cmd_iQ={iq_val}A")
    axes[2].plot(r["df"]["time"], r["df"]["uD"], linewidth=1, label=f"cmd_iQ={iq_val}A")
    axes[3].plot(r["df"]["time"], r["df"]["uQ"], linewidth=1, label=f"cmd_iQ={iq_val}A")

for j in range(4):
    axes[j].legend(fontsize=8)
    axes[j].grid(True, alpha=0.3)
axes[0].set_ylabel("speed [rad/s]")
axes[0].set_title("Speed (FOC current mode)")
axes[1].set_ylabel("iQ [A]")
axes[1].set_title("Q-axis current")
axes[2].set_ylabel("uD [V]")
axes[2].set_title("D-axis voltage")
axes[3].set_ylabel("uQ [V]")
axes[3].set_xlabel("time [s]")
axes[3].set_title("Q-axis voltage")
plt.tight_layout()
plt.savefig(os.path.join(OUT_PATH, "foc_current_analysis.png"), dpi=150)
plt.close()

print(f"\n{'='*80}")
print("Summary")
print(f"{'='*80}")
for tag in sorted(results.keys()):
    r = results[tag]
    iq_val = tag.replace("iq", "")
    tem = 1.5 * 4 * 0.0107 * float(iq_val)
    print(f"  cmd_iQ={iq_val:>4s}A: 稳态ω={r['omega_end']:10.1f} rad/s,  iQ跟踪误差={abs(r['iq_end']-float(iq_val)):.4f}A,  理论Tem={tem:.4f}Nm")
