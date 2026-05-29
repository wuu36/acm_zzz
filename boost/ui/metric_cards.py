import streamlit as st

def render_metric_cards(params):
    Vin = params["Vin"]
    D = params["D"]
    R = params["R"]
    L = params["L"]
    C = params["C"]
    f = params["f"]
    RL = params["RL"]
    Vd = params["Vd"]

    if D is None:
        Vout_target = params["Vout_target"]
        D = 1 - Vin / Vout_target

    D_safe = min(max(D, 0.01), 0.9)

    Lc = (1 - D_safe)**2 * D_safe * R / (2 * f)
    mode = "CCM" if L > Lc else "DCM"

    if mode == "CCM":
        gain = (1 / (1 - D_safe)) * (R / (R + RL / (1 - D_safe)**2))
        Vout = Vin * gain
    else:
        discriminant = 1 + 8 * L * f / (R * D_safe**2)
        gain = (1 + discriminant**0.5) / 2
        Vout = Vin * gain

    IL_avg = Vout / (R * (1 - D_safe))
    delta_IL = Vin * D_safe / (f * L)
    delta_Vout = Vout * D_safe / (f * R * C)
    IL_peak = IL_avg + delta_IL / 2
    Iout = Vout / R
    efficiency = Vout * Iout / (Vin * IL_avg) if IL_avg > 0 else 0

    col1, col2, col3, col4 = st.columns(4)
    col1.metric("工作模式", mode)
    col2.metric("实际 Vout", f"{Vout:.2f} V")
    col3.metric("电感纹波 ΔIL", f"{delta_IL*1e3:.2f} mA")
    col4.metric("输出纹波 ΔVout", f"{delta_Vout*1e3:.2f} mV")

    col5, col6, col7, col8 = st.columns(4)
    col5.metric("临界电感 Lc", f"{Lc*1e6:.2f} μH")
    col6.metric("峰值电流 IL_peak", f"{IL_peak:.2f} A")
    col7.metric("输出电流 Iout", f"{Iout:.2f} A")
    col8.metric("效率 η", f"{efficiency*100:.1f} %")

    if params["control_mode"] == "闭环(自动稳定)":
        st.info(f"闭环计算占空比 D = {D_safe:.3f}")

    return {
        "mode": mode,
        "Vout": Vout,
        "Lc": Lc,
        "IL_avg": IL_avg,
        "IL_peak": IL_peak,
        "delta_IL": delta_IL,
        "delta_Vout": delta_Vout,
        "Iout": Iout,
        "efficiency": efficiency,
        "D": D_safe,
        "gain": gain,
    }