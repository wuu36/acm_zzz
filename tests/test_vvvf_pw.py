import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

import os
import time
import math
import shutil
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from playwright.sync_api import sync_playwright

BASE_PATH = os.path.dirname(os.path.abspath(__file__))
OUT_PATH = os.path.join(BASE_PATH, "dat", "pw_test_results")
SIM_DAT = os.path.join(BASE_PATH, "frameworkCodes", "dat")
ST_URL = "http://localhost:8501"

os.makedirs(OUT_PATH, exist_ok=True)

MOTOR_PARAMS = {
    "SEW100W": {"npp": 4, "R": 0.475, "Ld": 2.05e-3, "Lq": 2.05e-3, "KE": 0.0107, "Js": 3.5e-8, "IN": 3.5, "Vdc": 48},
    "SEW200W": {"npp": 4, "R": 0.2465, "Ld": 0.7e-3, "Lq": 0.7e-3, "KE": 0.0115, "Js": 2.4e-8, "IN": 6.5, "Vdc": 48},
    "CustomPMSM": {"npp": 4, "R": 0.5, "Ld": 2.0e-3, "Lq": 2.0e-3, "KE": 0.01, "Js": 5.0e-8, "IN": 3.5, "Vdc": 48},
}

PASS_COUNT = 0
FAIL_COUNT = 0

def select_motor(page, motor_name):
    sb = page.locator("[data-testid='stSelectbox']").first
    sb.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if motor_name in opt.inner_text():
            opt.click()
            time.sleep(2)
            return

def select_mode(page, mode_name):
    for rl in page.locator("[data-testid='stRadio'] label").all():
        if mode_name in rl.inner_text():
            rl.click()
            time.sleep(2)
            return

def set_input(page, aria_label, value):
    inp = page.locator(f"input[aria-label='{aria_label}']")
    inp.click()
    inp.fill("")
    inp.fill(str(value))
    inp.press("Enter")
    time.sleep(0.5)

def verify_vvvf(df, p, uD, uQ, tload, tag):
    global PASS_COUNT, FAIL_COUNT
    npp = p["npp"]
    KE = p["KE"]
    R = p["R"]

    omega_steady = df["varOmega"].iloc[-200:].mean()
    iq_steady = abs(df["iQ"].iloc[-200:].mean())
    omega_theory_noload = uQ / (npp * KE)

    print(f"\n--- Verification: {tag} ---")
    print(f"  Theory (no load): omega = uQ/(npp*KE) = {uQ}/{npp*KE:.4f} = {omega_theory_noload:.2f} rad/s")

    checks = []

    if tload > 0:
        iq_theory = tload / (1.5 * npp * KE)
        checks.append(("steady iQ = TLoad/(1.5*npp*KE)", abs(iq_steady - iq_theory) / iq_theory * 100 < 10.0,
                       f"actual={iq_steady:.4f}, theory={iq_theory:.4f}, err={abs(iq_steady - iq_theory)/iq_theory*100:.1f}%"))

        tem_steady = df["Tem"].iloc[-200:].mean()
        checks.append(("steady Tem = TLoad", abs(tem_steady - tload) / tload < 5.0,
                       f"actual={tem_steady:.4f}, expected={tload}"))

        checks.append(("load speed < no-load speed", omega_steady < omega_theory_noload,
                       f"load={omega_steady:.2f} < noload={omega_theory_noload:.2f}"))

        checks.append(("VVVF speed drop proportional to load",
                       omega_steady > omega_theory_noload * 0.5,
                       f"omega={omega_steady:.2f} > 50% of noload={omega_theory_noload:.2f}"))
    else:
        omega_err = abs(omega_steady - omega_theory_noload) / omega_theory_noload * 100
        checks.append(("steady speed ≈ uQ/(npp*KE)", omega_err < 5.0,
                       f"actual={omega_steady:.2f}, theory={omega_theory_noload:.2f}, err={omega_err:.1f}%"))

        checks.append(("steady iQ ≈ 0", iq_steady < 0.01,
                       f"actual={iq_steady:.6f}"))

    back_emf = omega_steady * npp * KE
    checks.append(("back-EMF < uQ", back_emf < uQ * 1.05,
                   f"back_emf={back_emf:.3f}V, uQ={uQ}V"))

    checks.append(("speed converges (steady < peak)", abs(omega_steady) < df["varOmega"].abs().max() * 1.1,
                   f"steady={abs(omega_steady):.2f}, peak={df['varOmega'].abs().max():.2f}"))

    t90_idx = df[df["varOmega"] >= 0.9 * omega_steady].index
    if len(t90_idx) > 0:
        t90 = df["time"].iloc[t90_idx[0]]
        checks.append(("accel time < 0.5s", t90 < 0.5, f"t90={t90:.3f}s"))

    for name, passed, detail in checks:
        status = "PASS" if passed else "FAIL"
        if passed:
            PASS_COUNT += 1
        else:
            FAIL_COUNT += 1
        print(f"  [{status}] {name}: {detail}")

    fig_path = os.path.join(OUT_PATH, f"{tag}_curve.png")
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))

    axes[0].plot(df["time"], df["varOmega"], "b-", linewidth=1)
    axes[0].axhline(y=omega_theory_noload, color="r", linestyle="--", label=f"no_load_theory={omega_theory_noload:.1f}")
    if tload > 0:
        axes[0].axhline(y=omega_steady, color="g", linestyle=":", label=f"steady={omega_steady:.1f}")
    axes[0].set_ylabel("speed [rad/s]")
    axes[0].set_title(f"Speed - {tag}")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(df["time"], df["iQ"], "r-", linewidth=1)
    if tload > 0:
        iq_theory = tload / (1.5 * npp * KE)
        axes[1].axhline(y=iq_theory, color="orange", linestyle="--", label=f"theory_iQ={iq_theory:.3f}")
        axes[1].legend(fontsize=8)
    axes[1].set_ylabel("iQ [A]")
    axes[1].set_title("Q-axis current")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(df["time"], df["Tem"], "g-", linewidth=1)
    if tload > 0:
        axes[2].axhline(y=tload, color="r", linestyle="--", label=f"TLoad={tload}")
        axes[2].legend(fontsize=8)
    axes[2].set_ylabel("Torque [Nm]")
    axes[2].set_xlabel("time [s]")
    axes[2].set_title("Torque")
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(fig_path, dpi=150)
    plt.close()
    print(f"  [CURVE] {fig_path}")

def run_single_test(pw, motor, uD, uQ, tload, steps, tag):
    browser = pw.chromium.launch(headless=False)
    page = browser.new_page(viewport={"width": 1400, "height": 900})
    page.goto(ST_URL, timeout=30000)
    time.sleep(5)

    select_motor(page, motor)
    set_input(page, "仿真步数", steps)
    select_mode(page, "开环VVVF")
    set_input(page, "D轴电压指令 [V]", uD)
    set_input(page, "Q轴电压指令 [V]", uQ)
    set_input(page, "负载转矩 [Nm]", tload)

    run_btn = page.get_by_text("运行仿真").first
    run_btn.click()
    print(f"\n{'='*50}")
    print(f"[RUN] {tag}: motor={motor}, uD={uD}, uQ={uQ}, TLoad={tload}")
    print(f"{'='*50}")

    try:
        page.wait_for_function(
            "() => document.body.innerText.includes('仿真完成') || document.body.innerText.includes('仿真失败') || document.body.innerText.includes('编译失败')",
            timeout=180000
        )
        time.sleep(3)
    except Exception as e:
        print(f"[TIMEOUT] {tag}: {e}")
        browser.close()
        return

    browser.close()

    data_file = os.path.join(SIM_DAT, "test_motor_velocity.dat")
    if not os.path.exists(data_file):
        print(f"[FAIL] {tag}: data file not found")
        return

    df = pd.read_csv(data_file)
    shutil.copy2(data_file, os.path.join(OUT_PATH, f"{tag}.dat"))

    verify_vvvf(df, MOTOR_PARAMS[motor], uD, uQ, tload, tag)

def main():
    global PASS_COUNT, FAIL_COUNT

    tests = [
        ("SEW100W", 0.0, 5.0, 0.0, 50000, "SEW100W_uQ5_TL0"),
        ("SEW200W", 0.0, 5.0, 0.0, 50000, "SEW200W_uQ5_TL0"),
        ("CustomPMSM", 0.0, 5.0, 0.0, 50000, "CustomPMSM_uQ5_TL0"),
        ("SEW100W", 0.0, 2.0, 0.0, 50000, "SEW100W_uQ2_TL0"),
        ("SEW100W", 0.0, 10.0, 0.0, 50000, "SEW100W_uQ10_TL0"),
        ("SEW100W", 0.0, 5.0, 0.05, 50000, "SEW100W_uQ5_TL0.05"),
        ("SEW100W", 0.0, 5.0, 0.1, 50000, "SEW100W_uQ5_TL0.1"),
    ]

    with sync_playwright() as pw:
        for motor, uD, uQ, tload, steps, tag in tests:
            try:
                run_single_test(pw, motor, uD, uQ, tload, steps, tag)
            except Exception as e:
                print(f"[ERROR] {tag}: {e}")
                FAIL_COUNT += 3

    print(f"\n{'='*50}")
    print(f"FINAL: {PASS_COUNT} PASS, {FAIL_COUNT} FAIL")
    print(f"{'='*50}")

if __name__ == "__main__":
    main()