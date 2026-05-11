import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
import os, time, shutil
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from playwright.sync_api import sync_playwright

BASE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(BASE_PATH, "dat", "pw_ud_uq_test")
SIM_DAT = os.path.join(BASE_PATH, "frameworkCodes", "dat")
ST_URL = "http://localhost:8501"

os.makedirs(OUT_PATH, exist_ok=True)

MOTOR = {"SEW100W": {"npp": 4, "R": 0.475, "Ld": 2.05e-3, "Lq": 2.05e-3, "KE": 0.0107, "Js": 3.5e-8}}
TAG = "ud_uq_analysis"

def set_val(page, label, val):
    inp = page.locator(f"input[aria-label='{label}']")
    inp.click()
    inp.fill(str(val))
    inp.press("Enter")
    time.sleep(0.3)

def run_test(pw, motor, uD, uQ, steps, tag):
    browser = pw.chromium.launch(headless=False)
    page = browser.new_page(viewport={"width": 1400, "height": 900})
    page.goto(ST_URL, timeout=30000)
    time.sleep(5)
    page.locator("[data-testid='stSelectbox']").first.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if motor in opt.inner_text():
            opt.click()
            break
    time.sleep(2)
    for rl in page.locator("[data-testid='stRadio'] label").all():
        if "开环VVVF" in rl.inner_text():
            rl.click()
            break
    time.sleep(2)
    set_val(page, "仿真步数", steps)
    set_val(page, "D轴电压指令 [V]", uD)
    set_val(page, "Q轴电压指令 [V]", uQ)
    set_val(page, "负载转矩 [Nm]", 0)
    time.sleep(1)
    page.get_by_text("运行仿真").first.click()
    print(f"[RUN] {tag}: motor={motor}, uD={uD}, uQ={uQ}, steps={steps}...")
    page.wait_for_function(
        "() => document.body.innerText.includes('仿真完成')",
        timeout=120000
    )
    time.sleep(3)
    browser.close()
    shutil.copy2(os.path.join(SIM_DAT, "test_motor_velocity.dat"), os.path.join(OUT_PATH, f"{tag}.dat"))

tests = [
    ("SEW100W", 0.0, 0.0, 4000, "uD0_uQ0"),
    ("SEW100W", 2.0, 0.0, 4000, "uD2_uQ0"),
    ("SEW100W", -2.0, 0.0, 4000, "uDN2_uQ0"),
    ("SEW100W", 0.0, 2.0, 4000, "uD0_uQ2"),
    ("SEW100W", 0.0, 5.0, 4000, "uD0_uQ5"),
    ("SEW100W", 0.0, 10.0, 4000, "uD0_uQ10"),
    ("SEW100W", 2.0, 5.0, 4000, "uD2_uQ5"),
    ("SEW100W", -2.0, 5.0, 4000, "uDN2_uQ5"),
]

with sync_playwright() as pw:
    for motor, uD, uQ, steps, tag in tests:
        try:
            run_test(pw, motor, uD, uQ, steps, tag)
        except Exception as e:
            print(f"[ERROR] {tag}: {e}")

# Analysis
print(f"\n{'='*70}")
print("uQ影响分析（D轴电压变化测试）")
print(f"{'='*70}")

fig, axes = plt.subplots(4, 1, figsize=(12, 16))

uq_tests = ["uD0_uQ0", "uD0_uQ2", "uD0_uQ5", "uD0_uQ10"]
labels_q = ["uQ=0V", "uQ=2V", "uQ=5V", "uQ=10V"]
colors_q = ["gray", "orange", "b", "r"]

for i, (tag, label, c) in enumerate(zip(uq_tests, labels_q, colors_q)):
    df = pd.read_csv(os.path.join(OUT_PATH, f"{tag}.dat"))
    omega_end = df["varOmega"].iloc[-10:].mean()
    axes[0].plot(df["time"], df["varOmega"], color=c, linewidth=1.5, label=f"{label} → {omega_end:.1f} rad/s")
    axes[1].plot(df["time"], df["iQ"], color=c, linewidth=1.5, label=label)
    axes[2].plot(df["time"], df["Tem"], color=c, linewidth=1.5, label=label)
    axes[3].plot(df["time"], df["iD"], color=c, linewidth=1.5, label=label)

for j in range(4):
    axes[j].legend(fontsize=8)
    axes[j].grid(True, alpha=0.3)
axes[0].set_ylabel("speed [rad/s]")
axes[0].set_title("uQ Effect (uD=0)")
axes[1].set_ylabel("iQ [A]")
axes[2].set_ylabel("Torque [Nm]")
axes[3].set_ylabel("iD [A]")
axes[3].set_xlabel("time [s]")
plt.tight_layout()
plt.savefig(os.path.join(OUT_PATH, "uq_effect.png"), dpi=150)
plt.close()

print(f"\n{'='*70}")
print("uD影响分析（Q轴电压固定）")
print(f"{'='*70}")

fig, axes = plt.subplots(4, 1, figsize=(12, 16))

ud_tests = ["uD0_uQ5", "uD2_uQ5", "uDN2_uQ5"]
labels_d = ["uD=0V (基准)", "uD=2V (增磁)", "uD=-2V (弱磁)"]
colors_d = ["b", "r", "g"]

for tag, label, c in zip(ud_tests, labels_d, colors_d):
    df = pd.read_csv(os.path.join(OUT_PATH, f"{tag}.dat"))
    omega_end = df["varOmega"].iloc[-10:].mean()
    axes[0].plot(df["time"], df["varOmega"], color=c, linewidth=1.5, label=f"{label} → {omega_end:.1f} rad/s")
    axes[1].plot(df["time"], df["iQ"], color=c, linewidth=1.5, label=label)
    axes[2].plot(df["time"], df["Tem"], color=c, linewidth=1.5, label=label)
    axes[3].plot(df["time"], df["iD"], color=c, linewidth=1.5, label=label)

for j in range(4):
    axes[j].legend(fontsize=8)
    axes[j].grid(True, alpha=0.3)
axes[0].set_ylabel("speed [rad/s]")
axes[0].set_title("uD Effect (uQ=5V fixed)")
axes[1].set_ylabel("iQ [A]")
axes[2].set_ylabel("Torque [Nm]")
axes[3].set_ylabel("iD [A]")
axes[3].set_xlabel("time [s]")
plt.tight_layout()
plt.savefig(os.path.join(OUT_PATH, "ud_effect.png"), dpi=150)
plt.close()

# Print key metrics
print("\nuQ 影响 (uD=0 固定):")
for tag, label in zip(uq_tests, labels_q):
    df = pd.read_csv(os.path.join(OUT_PATH, f"{tag}.dat"))
    p = MOTOR["SEW100W"]
    omega = df["varOmega"].iloc[-10:].mean()
    iq_pk = df["iQ"].abs().max()
    tem_pk = df["Tem"].abs().max()
    theory = 0 if tag == "uD0_uQ0" else 0.0 / (p["npp"] * p["KE"])
    print(f"  {label}: omega={omega:.2f} rad/s, iQ_pk={iq_pk:.3f}A, Tem_pk={tem_pk:.4f}Nm")

print("\nuD 影响 (uQ=5V 固定):")
for tag, label in zip(ud_tests, labels_d):
    df = pd.read_csv(os.path.join(OUT_PATH, f"{tag}.dat"))
    omega = df["varOmega"].iloc[-10:].mean()
    iq_pk = df["iQ"].abs().max()
    id_pk = df["iD"].iloc[-10:].mean()
    print(f"  {label}: omega={omega:.2f} rad/s, iQ_pk={iq_pk:.3f}A, iD_steady={id_pk:.4f}A")
