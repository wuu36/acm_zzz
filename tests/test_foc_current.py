# type: ignore
"""FOC电流模式测试

测试3种场景:
1. 不同cmd_iQ的瞬态跟踪（50us步长, 200步=10ms）
2. cmd_iQ=1A 无负载（加速到Vdc饱和）
3. cmd_iQ=2A 带负载TLoad=0.05Nm（平衡运行）
"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

import os, shutil
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from super_config import SuperConfig
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = os.path.join(os.path.dirname(__file__), "..")
SDAT = os.path.join(BASE, "frameworkCodes", "dat")

def run_foc(motor, cmd_iq, tload=0.0, cl_ts=5e-5, steps=200, target="foc_test"):
    cfg = SuperConfig(motor)
    cfg.load_motor_library()
    cfg.load_user_config()
    p = cfg.get_motor_params(motor)
    cfg.user_config["simulation"]["sim.CL_TS"] = cl_ts
    cfg.user_config["simulation"]["sim.NUMBER_OF_STEPS"] = steps
    cfg.user_config["simulation"]["sim.MODE_SELECT"] = 3
    cfg.user_config["test"]["test.cmd_speed"] = 0.0
    cfg.user_config["test"]["test.cmd_iQ"] = cmd_iq
    cfg.user_config["test"]["test.cmd_uD"] = 0.0
    cfg.user_config["test"]["test.cmd_uQ"] = 0.0
    cfg.user_config["test"]["test.TLoad"] = tload
    cfg.user_config["motor"]["motor.npp"] = p["npp"]
    cfg.user_config["motor"]["motor.IN"] = p["IN"]
    cfg.user_config["motor"]["motor.R"] = p["R"]
    cfg.user_config["motor"]["motor.Ld"] = p["Ld"]
    cfg.user_config["motor"]["motor.Lq"] = p["Lq"]
    cfg.user_config["motor"]["motor.KE"] = p["KE"]
    cfg.user_config["motor"]["motor.Js"] = p["Js"]
    cfg.user_config["motor"]["motor.Vdc"] = p["Vdc"]
    cfg.update_super_config()
    cfg.compile_simulation(target)
    cfg.run_simulation(target)
    return pd.read_csv(os.path.join(SDAT, "test_motor_foc.dat"))

print("="*70)
print("Test 1: FOC电流跟踪瞬态 (cmd_iQ=1A, 50步=2.5ms)")
print("="*70)
df1 = run_foc("SEW100W", cmd_iq=1.0, steps=50, cl_ts=5e-5)
# t90: first time iQ >= 0.9*1.0 in rising edge
iQ_90 = df1[df1["iQ"] >= 0.9 * 1.0]
t90 = df1["time"].iloc[0]
iQ_max = df1["iQ"].max()
if not iQ_90.empty:
    t90 = iQ_90.iloc[0]["time"]
print(f"  peak iQ: {iQ_max:.4f} A (reached {'no 90%' if iQ_90.empty else f't90={t90*1000:.2f}ms'})")
print(f"  稳态iQ (step 50): {df1['iQ'].iloc[-10:].mean():.4f} A")

print("\n" + "="*70)
print("Test 2: FOC无负载 (cmd_iQ=2A, 10000步=500ms)")
print("="*70)
df2 = run_foc("SEW100W", cmd_iq=2.0, tload=0.0, steps=10000)
print(f"  峰值速度: {df2['varOmega'].max():.1f} rad/s  ({df2['varOmega'].max()*60/(2*3.14159):.0f} rpm)")
print(f"  稳态速度: {df2['varOmega'].iloc[-10:].mean():.1f} rad/s")
print(f"  稳态iQ:   {df2['iQ'].iloc[-10:].mean():.4f} A")
back_emf = df2['varOmega'].iloc[-10:].mean() * 4 * 0.0107
print(f"  稳态反EMF: {back_emf:.2f} V (Vdc/2=24V)")

print("\n" + "="*70)
print("Test 3: FOC带负载 (cmd_iQ=2A, TLoad=0.05Nm, 4000步=200ms)")
print("="*70)
df3 = run_foc("SEW100W", cmd_iq=2.0, tload=0.05, steps=4000)
print(f"  稳态速度: {df3['varOmega'].iloc[-200:].mean():.1f} rad/s")
print(f"  稳态iQ:   {df3['iQ'].iloc[-200:].mean():.4f} A")
print(f"  稳态Tem:  {df3['Tem'].iloc[-200:].mean():.4f} Nm (TLoad=0.05Nm)")
iq_theory = 0.05 / (1.5 * 4 * 0.0107)
print(f"  理论iQ:   {iq_theory:.4f} A (TLoad/(1.5*npp*KE))")

# 绘图
fig, axes = plt.subplots(3, 3, figsize=(14, 10))

# T1 电流跟踪
axes[0,0].plot(df1["time"]*1000, df1["iQ"], "b-", linewidth=1)
axes[0,0].axhline(y=1.0, color="r", linestyle="--", label="cmd_iQ=1A")
axes[0,0].set_title("T1: iQ tracking transient")
axes[0,0].set_ylabel("iQ [A]")
axes[0,0].set_xlabel("time [ms]")
axes[0,0].legend(fontsize=7)
axes[0,0].grid(True, alpha=0.3)

# T1 速度
axes[0,1].plot(df1["time"]*1000, df1["varOmega"], "b-", linewidth=1)
axes[0,1].set_title("T1: speed")
axes[0,1].set_ylabel("omega [rad/s]")
axes[0,1].set_xlabel("time [ms]")
axes[0,1].grid(True, alpha=0.3)

# T2 无负载速度
axes[1,0].plot(df2["time"]*1000, df2["varOmega"], "b-", linewidth=1)
axes[1,0].axhline(y=560, color="r", linestyle="--", label="Vdc limit")
axes[1,0].set_title("T2: speed (no load, cmd_iQ=2A)")
axes[1,0].set_ylabel("omega [rad/s]")
axes[1,0].set_xlabel("time [ms]")
axes[1,0].legend(fontsize=7)
axes[1,0].grid(True, alpha=0.3)

# T2 iQ
axes[1,1].plot(df2["time"]*1000, df2["iQ"], "b-", linewidth=1)
axes[1,1].axhline(y=2.0, color="r", linestyle="--", label="cmd_iQ=2A")
axes[1,1].set_title("T2: iQ (no load)")
axes[1,1].set_ylabel("iQ [A]")
axes[1,1].set_xlabel("time [ms]")
axes[1,1].legend(fontsize=7)
axes[1,1].grid(True, alpha=0.3)

# T3 带负载速度
axes[2,0].plot(df3["time"]*1000, df3["varOmega"], "b-", linewidth=1)
axes[2,0].set_title("T3: speed (TLoad=0.05Nm)")
axes[2,0].set_ylabel("omega [rad/s]")
axes[2,0].set_xlabel("time [ms]")
axes[2,0].grid(True, alpha=0.3)

# T3 iQ
axes[2,1].plot(df3["time"]*1000, df3["iQ"], "b-", linewidth=1)
axes[2,1].axhline(y=iq_theory, color="r", linestyle="--", label=f"theory={iq_theory:.4f}A")
axes[2,1].set_title("T3: iQ (with load)")
axes[2,1].set_ylabel("iQ [A]")
axes[2,1].set_xlabel("time [ms]")
axes[2,1].legend(fontsize=7)
axes[2,1].grid(True, alpha=0.3)

# T3 转矩
axes[2,2].plot(df3["time"]*1000, df3["Tem"], "b-", linewidth=1)
axes[2,2].axhline(y=0.05, color="r", linestyle="--", label="TLoad=0.05")
axes[2,2].set_title("T3: torque (with load)")
axes[2,2].set_ylabel("Tem [Nm]")
axes[2,2].set_xlabel("time [ms]")
axes[2,2].legend(fontsize=7)
axes[2,2].grid(True, alpha=0.3)

fig.delaxes(axes[0,2])
fig.delaxes(axes[1,2])
plt.tight_layout()
out = os.path.join(os.path.dirname(__file__), "..", "dat", "foc_current_test.png")
plt.savefig(out, dpi=150)
plt.close()
print(f"\n[FIGURE] {out}")
print("All tests completed.")
