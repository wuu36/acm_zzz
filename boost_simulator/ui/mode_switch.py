import streamlit as st

def render_mode_switch():
    control_mode = st.radio(
        "控制模式",
        ["开环 (固定占空比)", "闭环 (自动稳压)"],
        index=0,
    )

    D = None
    Vout_target = None
    Kp = None
    Ki = None

    if control_mode == "开环 (固定占空比)":
        D = st.slider("占空比 D", min_value=0.01, max_value=0.90, value=0.50, step=0.01)
        st.caption("Vout 由 D 和电路参数决定，下方指标卡显示计算结果")
    else:
        Vout_target = st.number_input(
            "目标输出电压 Vout (V)",
            min_value=0.1, max_value=500.0, value=24.0, step=0.5,
        )
        st.caption("系统将自动计算所需占空比 D")
        Kp = st.number_input("PI 参数 Kp", min_value=0.001, max_value=10.0, value=0.1, step=0.01)
        Ki = st.number_input("PI 参数 Ki", min_value=0.001, max_value=1000.0, value=10.0, step=1.0)

    return control_mode, D, Vout_target, Kp, Ki