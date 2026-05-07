

import os
import streamlit as st


def motor_parameter_editor(motor_params):
    
    st.subheader("电机参数编辑")

    edited_params = {}

    # 基本参数
    col1, col2 = st.columns(2)

    with col1:
        edited_params['npp'] = st.number_input(
            "极对数",
            value=motor_params.get('npp', 4),
            min_value=1,
            max_value=20,
            step=1
        )
        edited_params['R'] = st.number_input(
            "定子电阻 [Ω]",
            value=float(motor_params.get('R', 0.5)),
            format="%.4f"
        )
        edited_params['Ld'] = st.number_input(
            "D轴电感 [mH]",
            value=float(motor_params.get('Ld', 2.0)*1000),
            format="%.2f"
        ) * 1e-3  # mH -> H

    with col2:
        edited_params['IN'] = st.number_input(
            "额定电流 [Arms]",
            value=float(motor_params.get('IN', 3.5)),
            format="%.2f"
        )
        edited_params['KE'] = st.number_input(
            "反电势系数 [Wb]",
            value=float(motor_params.get('KE', 0.01)),
            format="%.4f"
        )
        edited_params['Lq'] = st.number_input(
            "Q轴电感 [mH]",
            value=float(motor_params.get('Lq', 2.0)*1000),
            format="%.2f"
        ) * 1e-3
    
    edited_params['Js'] = st.number_input(
        "转动惯量 [kg·cm²]",
        value=float(motor_params.get('Js', 3.5)*1e6),
        format="%.2f"
    ) * 1e-6 # kg·cm² -> kg.m²

    edited_params['Vdc'] = st.number_input(
        "母线电压 [V]",
        value=int(motor_params.get('Vdc', 48)),
        min_value=12,
        max_value=400
    )

    return edited_params