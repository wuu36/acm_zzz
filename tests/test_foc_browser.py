# type: ignore
"""FOC电流模式浏览器验证测试

通过Playwright操作Streamlit UI，验证仿真结果
"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

import os, time, shutil
import pandas as pd
from playwright.sync_api import sync_playwright

OUT_PATH = os.path.join(os.path.dirname(__file__), "..", "dat", "foc_browser_test")
SDAT = os.path.join(os.path.dirname(__file__), "..", "frameworkCodes", "dat")
ST_URL = "http://localhost:8510"
os.makedirs(OUT_PATH, exist_ok=True)

tests = [
    # (tag, motor, mode_display, cmd_iq, tload, steps, expected_notes, data_file)
    ("FOC_no_load", "SEW100W", "FOC电流", 1.0, 0.0, 10000, "无负载: iQ→0, 速度饱和~560", "test_motor_velocity.dat"),
    ("FOC_tload005", "SEW100W", "FOC电流", 2.0, 0.05, 10000, "轻载: iQ≈0.779A, Tem≈0.05", "test_motor_foc_load.dat"),
    ("FOC_tload015", "SEW100W", "FOC电流", 2.0, 0.15, 10000, "重载: iQ≈2.33A, Tem≈0.15", "test_motor_foc_load.dat"),
]

print("="*70)
print("FOC电流模式 - 浏览器UI验证测试")
print("="*70)

with sync_playwright() as pw:
    for tag, motor, mode, cmd_iq, tload, steps, note, data_file in tests:
        print(f"\n--- {tag}: {note} ---")
        browser = pw.chromium.launch(headless=False)
        page = browser.new_page(viewport={"width": 1400, "height": 900})
        try:
            page.goto(ST_URL, timeout=30000)
        except Exception:
            print("  [WARN] Connection to Streamlit failed, trying in 3s...")
            time.sleep(3)
            page.goto(ST_URL, timeout=30000)
        time.sleep(5)

        # Select motor
        page.locator("[data-testid='stSelectbox']").first.click()
        time.sleep(1)
        for opt in page.locator("[role='option']").all():
            if motor in opt.inner_text():
                opt.click()
                break
        time.sleep(2)

        # Select FOC mode
        for rl in page.locator("[data-testid='stRadio'] label").all():
            if "FOC电流" in rl.inner_text():
                rl.click()
                break
        time.sleep(2)

        # Set steps, cmd_iq, tload
        for label, val in [("仿真步数", str(steps)),
                            ("Q轴电流指令 [A]", str(cmd_iq)),
                            ("负载转矩 [Nm]", str(tload))]:
            inp = page.locator(f"input[aria-label='{label}']")
            inp.click()
            inp.fill("")
            inp.fill(val)
            inp.press("Enter")
            time.sleep(0.3)

        # Run
        page.get_by_text("运行仿真").first.click()
        print(f"  [BROWSER] Running {tag}...")

        page.wait_for_function(
            "() => document.body.innerText.includes('仿真完成') || document.body.innerText.includes('仿真失败')",
            timeout=120000
        )
        time.sleep(2)
        browser.close()

        # Read data
        data_path = os.path.join(os.path.dirname(__file__), "..", "frameworkCodes", "dat")
        df_path = os.path.join(data_path, data_file)
        if not os.path.exists(df_path):
            print(f"  [FAIL] no data file: {df_path}")
            continue

        df = pd.read_csv(df_path)
        omega_col = "varOmega" if "varOmega" in df.columns else "omega"
        s = df.iloc[-30:].mean()
        omega_steady = s[omega_col]
        iq_steady = s["iQ"]
        tem_steady = s["Tem"]

        shutil.copy2(df_path, os.path.join(OUT_PATH, f"{tag}.dat"))

        # Compare
        expect_tem = tload
        expect_iq = tload / (1.5 * 4 * 0.0107) if tload > 0 else 0

        tem_err = abs(tem_steady - expect_tem) / max(abs(expect_tem), 0.001) * 100
        iq_err = abs(abs(iq_steady) - abs(expect_iq)) / max(abs(expect_iq), 0.001) * 100

        print(f"  Steady omega = {omega_steady:.2f} rad/s")
        print(f"  Steady iQ    = {iq_steady:.4f} A (expected ~{expect_iq:.4f}, err={iq_err:.1f}%)")
        print(f"  Steady Tem   = {tem_steady:.4f} Nm (expected {expect_tem:.2f}, err={tem_err:.1f}%)")

        if tload > 0:
            pass_check = iq_err < 10 and tem_err < 10
        else:
            pass_check = abs(tem_steady) < 0.001 and omega_steady > 500

        print(f"  RESULT: {'PASS' if pass_check else 'FAIL'}")

print("\n" + "="*70)
print("Browser UI test completed. Data saved to:")
print(OUT_PATH)
