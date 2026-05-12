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
OUT_PATH = os.path.join(BASE_PATH, "dat", "ud_analysis")


print("=" * 80)
print("ud change analysis")
print("=" * 80)

results = {}

def run_test(ud, uq, steps=4000, tag=""):
    print("running run_test")
    cfg = SuperConfig("SEW100W")
    cfg.load_motor_library()
    cfg.load_user_config()
    p = cfg.get_motor_params("SEW100W")
    # print(p)
    cfg.user_config["simulation"]["sim.CL_TS"] = 5e-5
    cfg.user_config["simulation"]["sim.NUMBER_OF_STEPS"] = steps
    cfg.user_config["simulation"]["sim.MODE_SELECT"] = 11
    cfg.user_config["test"]["test.cmd_uD"] = ud
    cfg.user_config["test"]["test.cmd_uQ"] = uq
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
    print(shutil_path)
    import shutil
    shutil.copy2(os.path.join(SIM_DAT, "test_motor_velocity.dat"), shutil_path)
    

    omega_end = df["varOmega"].iloc[-10:].mean()
    id_end = df["iD"].abs().max()
    id_pk = df["iD"].abs().max()
    iq_pk = df["iQ"].abs().max()
    omega_pk = df["varOmega"].abs().max()

    return {"tag": tag, "omega_end": omega_end, "iD_end": id_end,
            "iD_pk": id_pk, "iQ_pk": iq_pk, "omega_pk": omega_pk, "df": df}
    # print(p["npp"])


for ud in range(-5, 6):
    tag = f"ud{ud}"
    print(f"  Running {tag} (uq = 5V)...")
    results[tag] = run_test(ud=ud, uq=5.0, steps=4000, tag=tag)

fig, axes = plt.subplots(4, 1, figsize=(10,14))

ud_values = sorted([int(float(k.replace("ud",""))) for k in results.keys()])
colors_map = {-5:"r",-4:"r",-3:"r",-2:"r",-1:"r",0:"gray",1:"b",2:"b",3:"b",4:"b",5:"b"}

for uD in ud_values:
    tag = f"ud{uD}"
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
    
