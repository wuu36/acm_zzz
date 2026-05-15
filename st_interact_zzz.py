import streamlit as st
import json
import os

import importlib.resources
from typing import Dict, Any
import _plugins


def user_selected_mode():
    with st.sidebar:
        st.title('✨选择模式')
        mode = st.selectbox(
            'mode', 
            ['C']+_plugins.PLUGINS,
            key='user_selected_mode', 
            label_visibility='collapsed', 
            index=0
        ) # 0 对应c语言仿真
    return user_selected_mode

def get_user_history():
    history = {}
    fname_session_state = f'{os.path.dirname(__file__)}/streamlit_user_session_data.json'
    if not os.path.exists(fname_session_state):
        print("ok")
        with open(fname_session_state, 'w') as f:
            f.write('{\n}')
    with open(fname_session_state, 'r') as f:
        d_sim = json.load(f)
        for k, v in d_sim.items():
            history[k] = v
    return history

def clear_history_module(history):
    with st.sidebar:
        if st.button("清空历史数据", type='primary', use_container_width=True):
            with open(os.path.dirname(__file__)+'/streamlit_user_session_data.json', 'w') as f:
                f.write('{\n}')
            history = {}
    return history

def option_select_motor(history, d_user_config):
    print(f"user history : {history}")
    d_motorLib = init_motor_lib()
    motor_name_list = list(d_motorLib.keys()) + ['my-yaml-customer-motor']
    print(motor_name_list)

    with st.sidebar:
        st.header('🔌电机选择:')
        user_selected_mode = st.selectbox(
            '从下拉列表选择一台电机：',
            motor_name_list,
            index=st.session_state.DEFAULT_MOTOR_INDEX,
            key='user_selected_motor'
        )

def init_motor_lib() -> Dict[str, Any]:
    try:
        ref = importlib.resources.files(__name__).joinpath(
            "input_motorLibrary/motor_library.json"
        )
        with ref.open(encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        raise RuntimeError(
            "motor_library.json not found in input_motorLibrary/"
        )