import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
import time
from playwright.sync_api import sync_playwright

OUT = "C:/003_github/acm_zzz/dat/pw_test_results"

with sync_playwright() as pw:
    browser = pw.chromium.launch(headless=False)
    page = browser.new_page(viewport={"width": 1400, "height": 900})
    page.goto("http://localhost:8501", timeout=30000)
    time.sleep(5)

    # Select SEW100W
    page.locator("[data-testid='stSelectbox']").first.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if "SEW100W" in opt.inner_text():
            opt.click()
            break
    time.sleep(2)

    # Select VVVF
    for rl in page.locator("[data-testid='stRadio'] label").all():
        if "开环VVVF" in rl.inner_text():
            rl.click()
            break
    time.sleep(2)

    # uQ=5, TLoad=0, steps=50000
    for label, val in [("仿真步数", "50000"), ("Q轴电压指令 [V]", "5.0"), ("负载转矩 [Nm]", "0.0")]:
        inp = page.locator(f"input[aria-label='{label}']")
        inp.click()
        inp.fill("")
        inp.fill(val)
        inp.press("Enter")
        time.sleep(0.5)

    # Click run
    page.get_by_text("运行仿真").first.click()
    print("[1] Running SEW100W VVVF uQ=5 TLoad=0...")

    page.wait_for_function(
        "() => document.body.innerText.includes('仿真完成') || document.body.innerText.includes('仿真失败')",
        timeout=180000
    )
    time.sleep(3)
    page.screenshot(path=f"{OUT}/SEW100W_VVVF_uQ5_TL0_fullpage.png", full_page=True)

    # Select data file dropdown
    dat_select = page.locator("[data-testid='stSelectbox']").nth(1)
    dat_select.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if "test_motor_velocity" in opt.inner_text():
            opt.click()
            break
    time.sleep(3)

    # Screenshot the curve area
    plot_area = page.locator("[data-testid='stPyPlotCanvas']").first
    if plot_area.count() > 0:
        plot_area.screenshot(path=f"{OUT}/SEW100W_VVVF_uQ5_TL0_curves.png")
        print("[1] Curve screenshot saved")
    else:
        print("[1] No plot canvas found, full page screenshot instead")
        page.screenshot(path=f"{OUT}/SEW100W_VVVF_uQ5_TL0_curves_full.png", full_page=True)

    # Test 2: SEW200W
    page.locator("[data-testid='stSelectbox']").first.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if "SEW200W" in opt.inner_text():
            opt.click()
            break
    time.sleep(2)

    for label, val in [("仿真步数", "50000")]:
        inp = page.locator(f"input[aria-label='{label}']")
        inp.click()
        inp.fill("")
        inp.fill(val)
        inp.press("Enter")
        time.sleep(0.5)

    page.get_by_text("运行仿真").first.click()
    print("[2] Running SEW200W VVVF uQ=5 TLoad=0...")

    page.wait_for_function(
        "() => document.body.innerText.includes('仿真完成') || document.body.innerText.includes('仿真失败')",
        timeout=180000
    )
    time.sleep(3)

    dat_select = page.locator("[data-testid='stSelectbox']").nth(1)
    dat_select.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if "test_motor_velocity" in opt.inner_text():
            opt.click()
            break
    time.sleep(3)
    page.screenshot(path=f"{OUT}/SEW200W_VVVF_uQ5_TL0_fullpage.png", full_page=True)

    # Test 3: SEW100W with TLoad=0.05
    page.locator("[data-testid='stSelectbox']").first.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if "SEW100W" in opt.inner_text():
            opt.click()
            break
    time.sleep(2)

    inp = page.locator("input[aria-label='负载转矩 [Nm]']")
    inp.click()
    inp.fill("")
    inp.fill("0.05")
    inp.press("Enter")
    time.sleep(0.5)

    page.get_by_text("运行仿真").first.click()
    print("[3] Running SEW100W VVVF uQ=5 TLoad=0.05...")

    page.wait_for_function(
        "() => document.body.innerText.includes('仿真完成') || document.body.innerText.includes('仿真失败')",
        timeout=180000
    )
    time.sleep(3)

    dat_select = page.locator("[data-testid='stSelectbox']").nth(1)
    dat_select.click()
    time.sleep(1)
    for opt in page.locator("[role='option']").all():
        if "test_motor_velocity" in opt.inner_text():
            opt.click()
            break
    time.sleep(3)
    page.screenshot(path=f"{OUT}/SEW100W_VVVF_uQ5_TL0.05_fullpage.png", full_page=True)

    browser.close()
    print("All screenshots saved. Please review the curves in:")
    print(OUT)