import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
import os, time
import pandas as pd
from playwright.sync_api import sync_playwright

MOTOR_PARAMS = {
    "SEW100W": {"npp": 4, "R": 0.475, "KE": 0.0107},
}
SIM_DAT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frameworkCodes", "dat")

with sync_playwright() as pw:
    browser = pw.chromium.launch(headless=False)
    page = browser.new_page(viewport={"width": 1400, "height": 900})
    page.goto("http://localhost:8501", timeout=30000)
    time.sleep(5)

    page.locator("[data-testid='stSelectbox']").first.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if "SEW100W" in opt.inner_text():
            opt.click()
            break
    time.sleep(2)

    for rl in page.locator("[data-testid='stRadio'] label").all():
        if "开环VVVF" in rl.inner_text():
            rl.click()
            break
    time.sleep(2)

    inp = page.locator("input[aria-label='Q轴电压指令 [V]']")
    inp.click()
    inp.fill("")
    inp.fill("5.0")
    inp.press("Enter")
    time.sleep(0.5)

    page.get_by_text("运行仿真").first.click()
    print("[WAIT] simulation running...")

    page.wait_for_function(
        "() => document.body.innerText.includes('仿真完成') || document.body.innerText.includes('仿真失败')",
        timeout=120000
    )
    time.sleep(3)
    print("[DONE] simulation finished")
    browser.close()

    data_file = os.path.join(SIM_DAT, "test_motor_velocity.dat")
    df = pd.read_csv(data_file)
    p = MOTOR_PARAMS["SEW100W"]
    omega_steady = df["varOmega"].iloc[-200:].mean()
    omega_theory = 5.0 / (p["npp"] * p["KE"])
    iq_steady = abs(df["iQ"].iloc[-200:].mean())
    err_pct = abs(omega_steady - omega_theory) / omega_theory * 100

    print(f"SEW100W VVVF uQ=5V:")
    print(f"  steady omega = {omega_steady:.2f} rad/s, theory = {omega_theory:.2f}, err = {err_pct:.1f}%")
    print(f"  steady iQ = {iq_steady:.6f} A (expect ~0)")
    print(f"  RESULT: {'PASS' if err_pct < 5 else 'FAIL'}")