import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from super_config import SuperConfig
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

BASE_PATH = os.path.join(os.path.dirname(__file__), "..")
C_PATH = os.path.join(BASE_PATH, "frameworkCodes", "c")
SIM_DAT = os.path.join(BASE_PATH, "frameworkCodes", "dat")
OUT_PATH = os.path.join(BASE_PATH, "dat", "ud_analysis")
os.makedirs(OUT_PATH, exist_ok=True)

MOTOR = {"npp": 4, "R": 0.475, "Ld": 2.05e-3, "Lq": 2.05e-3, "KE": 0.0107, "Js": 3.5e-8}

def run_test(uD, uQ, steps=4000, tag=""):
    cfg = SuperConfig("SEW100W")
    cfg.load_motor_library()
    cfg.load_user_config()
    p = cfg.get_motor_params("SEW100W")
    cfg.user_config["simulation"]["sim.CL_TS"] = 5e-5
    cfg.user_config["simulation"]["sim.NUMBER_OF_STEPS"] = steps
    cfg.user_config["simulation"]["sim.MODE_SELECT"] = 11
    cfg.user_config["test"]["test.cmd_uD"] = uD
    cfg.user_config["test"]["test.cmd_uQ"] = uQ
    cfg.user_config["test"]["test.cmd_speed"] = 0.0
    cfg.user_config["test"]["test.cmd_iQ"] = 0.0
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
    cfg.compile_simulation("velocity_test")
    cfg.run_simulation("velocity_test")

    df = pd.read_csv(os.path.join(SIM_DAT, "test_motor_velocity.dat"))
    shutil_path = os.path.join(OUT_PATH, f"{tag}.dat")
    import shutil
    shutil.copy2(os.path.join(SIM_DAT, "test_motor_velocity.dat"), shutil_path)

    omega_end = df["varOmega"].iloc[-10:].mean()
    iD_end = df["iD"].iloc[-10:].mean()
    iD_pk = df["iD"].abs().max()
    iQ_pk = df["iQ"].abs().max()
    omega_pk = df["varOmega"].abs().max()

    return {"tag": tag, "omega_end": omega_end, "iD_end": iD_end,
            "iD_pk": iD_pk, "iQ_pk": iQ_pk, "omega_pk": omega_pk, "df": df}

print("=" * 80)
print("uD 增磁/弱磁分析")
print("=" * 80)

results = {}
for uD in range(-5, 6):
    tag = f"uD{uD}"
    print(f"  Running {tag} (uQ=5V)...")
    results[tag] = run_test(uD=uD, uQ=5.0, steps=4000, tag=tag)

fig, axes = plt.subplots(4, 1, figsize=(10, 14))

ud_values = sorted([int(float(k.replace("uD",""))) for k in results.keys()])
colors_map = {-5:"r",-4:"r",-3:"r",-2:"r",-1:"r",0:"gray",1:"b",2:"b",3:"b",4:"b",5:"b"}

for uD in ud_values:
    tag = f"uD{uD}"
    r = results[tag]
    c = colors_map[uD]
    axes[0].plot(r["df"]["time"], r["df"]["varOmega"], color=c, linewidth=1,
                  label=f"uD={uD:+d}V  omega={r['omega_end']:.1f} rad/s")
    axes[1].plot(r["df"]["time"], r["df"]["iD"], color=c, linewidth=1, label=f"uD={uD:+d}V")
    axes[2].plot(r["df"]["time"], r["df"]["iQ"], color=c, linewidth=1, label=f"uD={uD:+d}V")
    axes[3].plot(r["df"]["time"], r["df"]["Tem"], color=c, linewidth=1, label=f"uD={uD:+d}V")

for j in range(4):
    axes[j].legend(fontsize=6, loc="best")
    axes[j].grid(True, alpha=0.3)
axes[0].set_ylabel("speed [rad/s]")
axes[0].set_title("Speed vs uD (uQ=5V fixed)")
axes[1].set_ylabel("iD [A]")
axes[1].set_title("D-axis current")
axes[2].set_ylabel("iQ [A]")
axes[2].set_title("Q-axis current")
axes[3].set_ylabel("Torque [Nm]")
axes[3].set_xlabel("time [s]")
axes[3].set_title("Electromagnetic torque")
plt.tight_layout()
plt.savefig(os.path.join(OUT_PATH, "ud_analysis.png"), dpi=150)
plt.close()

print(f"\n\n{'='*80}")
print("Summary")
print(f"{'='*80}")
print(f"{'uD':>5s} | {'稳态速度':>12s} | {'稳态iD':>10s} | {'峰值iD':>10s} | {'峰值速度':>12s} | 备注")
print("-" * 80)
for uD in ud_values:
    tag = f"uD{uD}"
    r = results[tag]
    note = ""
    if uD > 0:
        note = "增磁 (flux adding)"
    elif uD < 0:
        note = "弱磁 (flux weakening)"
    else:
        note = "基准"
    print(f"{uD:+5d}V | {r['omega_end']:10.2f} rad/s | {r['iD_end']:10.4f} A | {r['iD_pk']:10.4f} A | {r['omega_pk']:10.2f} rad/s | {note}")

print(f"\n理论解释:")
print(f"  总磁链 psi = KE + Ld*iD")
print(f"  稳态速度 omega = (uQ - R*iQ) / (npp * psi)")
print(f"  无负载时 iQ≈0,  所以 omega ≈ uQ / (npp * psi)")
print(f"  uD>0 → iD>0 → psi增大 → omega减小 (增磁降速)")
print(f"  uD<0 → iD<0 → psi减小 → omega增大 (弱磁升速)")
