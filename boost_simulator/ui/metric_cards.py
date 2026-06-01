import streamlit as st
from core.steady_state import solve_steady_state, solve_closed_loop_duty, D_MAX
from utils.validators import validate_params, validate_closed_loop

def render_metric_cards(params):
    Vin = params["Vin"]
    R = params["R"]
    L = params["L"]
    C = params["C"]
    f = params["f"]
    RL = params["RL"]
    Vd = params["Vd"]
    D = params["D"]
    control_mode = params["control_mode"]

    validation_errors = []
    validation_warnings = []

    if control_mode == "开环 (固定占空比)":
        v = validate_params(Vin, R, L, C, f, D, RL, Vd)
        validation_errors = v["errors"]
        validation_warnings = v["warnings"]
        if D is None:
            D = 0.5
    else:
        Vout_target = params["Vout_target"]
        v1 = validate_params(Vin, R, L, C, f, None, RL, Vd)
        v2 = validate_closed_loop(Vin, Vout_target)
        validation_errors = v1["errors"] + v2["errors"]
        validation_warnings = v1["warnings"] + v2["warnings"]
        D = solve_closed_loop_duty(Vin, Vout_target, R, RL, Vd)
        if D is None or D < 0.01 or D > D_MAX:
            st.error(f"无法计算满足目标 Vout={Vout_target}V 的占空比")
            return None

    for err in validation_errors:
        st.error(err)
    for warn in validation_warnings:
        st.warning(warn)

    if validation_errors:
        return None

    results = solve_steady_state(Vin, D, R, L, C, f, RL, Vd)

    col1, col2, col3, col4 = st.columns(4)
    col1.metric("工作模式", results["mode"])
    col2.metric("实际 Vout", f"{results['Vout']:.2f} V")
    col3.metric("电感纹波 ΔIL", f"{results['delta_IL']*1e3:.2f} mA")
    col4.metric("输出纹波 ΔVout", f"{results['delta_Vout']*1e3:.2f} mV")

    col5, col6, col7, col8 = st.columns(4)
    col5.metric("临界电感 Lc", f"{results['Lc']*1e6:.2f} μH")
    col6.metric("峰值电流 IL_peak", f"{results['IL_peak']:.2f} A")
    col7.metric("输出电流 Iout", f"{results['Iout']:.2f} A")
    col8.metric("效率 η", f"{results['efficiency']*100:.1f} %")

    col9, col10, col11, col12 = st.columns(4)
    col9.metric("理想 Vout", f"{results['Vout_ideal']:.2f} V")
    col10.metric("增益 M", f"{results['gain']:.3f}")
    col11.metric("占空比 D", f"{results['D']:.3f}")
    col12.metric("输入功率 Pin", f"{results['Pin']:.2f} W")

    if control_mode == "闭环 (自动稳压)":
        st.info(f"闭环计算占空比 D = {results['D']:.3f} (迭代求解含寄生损耗)")

    return results