import streamlit as st
from ui.mode_switch import render_mode_switch

def render_sidebar():
    with st.sidebar:
        st.header("参数设置")

        control_mode, D, Vout_target, Kp, Ki = render_mode_switch()

        st.markdown("---")
        st.subheader("电路参数")

        Vin = st.number_input("输入电压 Vin (V)", min_value=0.1, max_value=200.0, value=12.0, step=0.5)
        R = st.number_input("负载电阻 R (Ω)", min_value=0.1, max_value=10000.0, value=10.0, step=1.0)
        L_uH = st.number_input("电感 L (μH)", min_value=1.0, max_value=10000.0, value=100.0, step=10.0)
        C_uF = st.number_input("电容 C (μF)", min_value=1.0, max_value=100000.0, value=100.0, step=10.0)
        f_kHz = st.number_input("开关频率 f (kHz)", min_value=1.0, max_value=1000.0, value=100.0, step=10.0)

        st.markdown("---")
        st.subheader("寄生参数")

        RL = st.number_input("电感内阻 RL (Ω)", min_value=0.0, max_value=5.0, value=0.05, step=0.01)
        Vd = st.number_input("二极管压降 Vd (V)", min_value=0.0, max_value=1.0, value=0.5, step=0.05)

        L = L_uH * 1e-6
        C = C_uF * 1e-6
        f = f_kHz * 1e3

        return {
            "control_mode": control_mode,
            "Vin": Vin,
            "R": R,
            "L": L,
            "C": C,
            "f": f,
            "D": D,
            "Vout_target": Vout_target,
            "RL": RL,
            "Vd": Vd,
            "Kp": Kp,
            "Ki": Ki,
            "L_uH": L_uH,
            "C_uF": C_uF,
            "f_kHz": f_kHz,
        }