import streamlit as st
import json
import os

import importlib.resources
from typing import Dict, Any
import _plugins
import numpy as np
import pandas as pd
import yaml
import time

import super_config as super_config

from streamlit.errors import StreamlitAPIException


def save_d_sim_2_dat_foler(d_sim):
    """将用户输入的参数保存到 dat 文件夹，方便使用cplot进行弹出框绘图"""
    dat_dir = os.path.join(os.path.dirname(__file__), 'frameworkCodes', 'dat')
    if not os.path.exists(dat_dir):
        os.makedirs(dat_dir)
    fname = os.path.join(dat_dir, f'{st.session_state.user_selected_motor}_d_sim.yaml')
    with open(fname, 'w') as f:
        yaml.dump(d_sim, f, default_flow_style=False, allow_unicode=True)
        

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
    return mode

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
        if st.button("清空历史数据", type='primary', width='stretch'):
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
        user_selected_motor = st.selectbox(
            '从下拉列表选择一台电机：',
            motor_name_list,
            index=st.session_state.DEFAULT_MOTOR_INDEX,
            key='user_selected_motor'
        )

        d_userMotor = dict()
        if user_selected_motor == 'my-yaml-customer-motor':
            print("in user_selected_motor")
            st.write(f"开发者模式：直接根据现有的 d_user_config.yaml 文件中的电机数据进行仿真，源自：{history['user_selected_motor']}")
            d_userMotor['motor'] = d_motorLib[history['user_selected_motor']]['基本参数']
        else:
            history['user_selected_motor'] = user_selected_motor
            d_userMotor['motor'] = d_motorLib[user_selected_motor]['基本参数']
            print(history)
        # 电机参数名字翻译
        d_userMotor['motor_simulated'] = {
            'init.npp':     d_userMotor['motor']['极对数 [1]'],
            'init.IN':      d_userMotor['motor']['额定电流 [Arms]'],
            'init.R':       d_userMotor['motor']['定子电阻 [Ohm]'],
            'init.Ld': 1e-3*d_userMotor['motor']['定子D轴电感 [mH]'],
            'init.Lq': 1e-3*d_userMotor['motor']['定子Q轴电感 [mH]'],
            'init.KE':      d_userMotor['motor']['额定反电势系数 [Wb]'],
            'init.Rreq': 0.0 if np.isnan(d_userMotor['motor']['反伽马转子电阻 [Ohm]']) else d_userMotor['motor']['反伽马转子电阻 [Ohm]'],
            'init.Js': 1e-4*d_userMotor['motor']['转动惯量 [kg.cm^2]'],
            'init.Vdc':     d_userMotor['motor']['母线电压 [Vdc]'],
        }

        # d_sim: 保存到 或者 读取自 streamlit session
        if st.session_state.user_selected_motor in history and 'd_sim' in history[st.session_state.user_selected_motor].keys():
            print('Loading history d_user...')
            d_sim = st.session_state.d_sim = history[st.session_state.user_selected_motor]['d_sim']
        else:
            st.session_state.d_sim = d_sim = dict()
            # d_sim['name'] = user_selected_motor
            d_sim.update(d_userMotor['motor_simulated'])            # 电机参数传递
            d_sim.update(d_user_config['simulation'])               # 仿真参数传递

        # d_sim = dict()
        # d_sim.update(d_userMotor['motor_simulated'])
        # d_sim.update(d_user_config['simulation'])
        # st.session_state.d_sim = d_sim

        df_basic_para_xlsx = pd.DataFrame.from_dict(d_userMotor['motor'], orient='index')
        # df_basic_para_python = pd.DataFrame.from_dict(d_userMotor['motor_simulated'], orient='index')
        
        # 更新侧边栏表格
        with st.expander(user_selected_motor + ' 的电机参数如下'):
            st.data_editor(df_basic_para_xlsx, disabled=True, width='stretch')

    return d_sim

def option_select_algorithm(d_sim, user_config):
    with st.sidebar:
        # options_alg = [key for key in d_sim.keys() if key.startswith('user.select_algorithm')
        # if len(options_alg) == 0:
        #     st.write('No algorithm to select')
        #     return d_sim
        
        d_who_is_usr = {
            'USER_CJH':     101976,
            'USER_XM':      102209,
            'USER_TEST':    101616,
        }
        selected_key = st.selectbox('who_is_user:', d_who_is_usr.keys(), index=st.session_state.DEFAULT_USER_INDEX)
        d_sim['user.who_is_user'] = d_who_is_usr[selected_key]

        d_mode_select = {
            'MODE_SELECT_PWM_DIRECT':         1,
            'MODE_SELECT_VOLTAGE_OPEN_LOOP':  11,
            'MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE': 2,
            'MODE_SELECT_FOC':                      3,
            'MODE_SELECT_FOC_SENSORLESS':           31,
            'MODE_SELECT_INDIRECT_FOC':             32,
            'MODE_SELECT_ID_SWEEPING_FREQ':         33,
            'MODE_SELECT_IQ_SWEEPING_FREQ':         34,
            'MODE_SELECT_FOC_AS_DC_GENERATOR':      35,
            'MODE_SELECT_FOC_HARNEFORS_1998':       36,
            'MODE_SELECT_VELOCITY_LOOP':            4,
            'MODE_SELECT_VELOCITY_LOOP_SENSORLESS': 41,
            'MODE_SELECT_TESTING_SENSORLESS':       42,
            'MODE_SELECT_VELOCITY_LOOP_WC_TUNER':   43,
            'MODE_SELECT_Marino2005':               44,
            'MODE_SELECT_VELOCITY_LOOP_HARNEFORS_1998':   45,
            'MODE_SELECT_SWEEPING_FREQ_FOR_VELOCITY_AND_CURRENT':   46,
            'MODE_SELECT_VELOCITY_LOOP_USING_ESO_FOR_SPEED' : 47,
            'MODE_SELECT_VARIABLE_PARAMETERS_VELOCITY_LOOP_SENSORLESS' : 48,
            'MODE_SELECT_INVERTER_NONLINEARITY_SENSORLESS': 49,
            'MODE_SELECT_POSITION_LOOP':            5,
            'MODE_SELECT_COMMISSIONING':            9,
            'MODE_SELECT_NYQUIST_PLOTTING':         91,
            'MODE_SELECT_UDQ_GIVEN_TEST':           98,
            'MODE_SELECT_GENERATOR':                8,
            'MODE_SELECT_NB_MODE':                  99,
        }
        d_sim['user.mode_select_synchronous_motor'] = 41 # MODE_SELECT_VELOCITY_LOOP_SENSORLESS
        d_sim['user.mode_select_induction_motor'] = 32   # MODE_SELECT_INDIRECT_FOC
        if(d_sim['init.Rreq'] > 0):
            selected_key = st.selectbox('mode_select_induction_motor:', d_mode_select.keys(), index=5)
            d_sim['user.mode_select_indunction_motor'] = d_mode_select[selected_key]
        else:
            selected_key = st.selectbox('mode_select_synchronous_motor:', d_mode_select.keys(), index=11)
            d_sim['user.mode_select_synchronous_motor'] = d_mode_select[selected_key]
    return d_sim


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

def save_para_config():
    # save user input filters as json file

    fname_session_state = f'{os.path.dirname(__file__)}/streamlit_user_session_data.json'
    with open(fname_session_state, 'r') as f:
        user_data = json.load(f)
        
    user_data['user_selected_motor'] = st.session_state.user_selected_motor
    user_data[st.session_state.user_selected_motor] = {}
    user_data[st.session_state.user_selected_motor]['d_sim'] = st.session_state.d_sim
    
    with open(fname_session_state, 'w') as f:
        json.dump(user_data, f, ensure_ascii=False, indent=4)
    return
    
def online_para_editor(default_var_list, d_sim):
    """使用st.data_editor对可变参数进行编辑"""
    # print(dict(d_sim))
    # print(default_var_list)
    df_user_input_motor_dict = pd.DataFrame.from_dict(d_sim, orient='index')
    with st.sidebar:
        with st.expander("可调参数: ", expanded=False):
            try:
                options = st.multiselect(
                    'Parameter table',
                    df_user_input_motor_dict.index.tolist(),
                    default_var_list)
            except StreamlitAPIException as e:
                print(e)
                print('清空你的streamllit_user_session_data.json文件再试一遍')
                raise e
            edited_df = st.data_editor(
                df_user_input_motor_dict.loc[options], width='stretch')
            for key in edited_df.index:
                st.session_state.d_sim[key] = edited_df.loc[key].iloc[0]
            if st.button(f"将上述设置保存为该电机默认设置", width='stretch'):
                save_para_config()
                

################################################
#                      C                       #
################################################

def c_save_run_module(d_sim, user_config):
    acm = super_config.SuperConfig(st.session_state.user_selected_motor)
    # acm.readDict(d_sim)
    with st.sidebar:
        # st.markdown('---')
        # st.text_input('TODO')
        if st.button('Save to C and compile', type="secondary", width='stretch'):
            st.write(f'update super_config.h at {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())}')
            path_to_dat = os.path.dirname(__file__) + f'/frameworkCodes/dat/{st.session_state.user_selected_motor}.dat'
            if os.path.exists(path_to_dat):
                os.remove(path_to_dat)
                
            print(f"d_sim: {d_sim}")
            print(f"user_config: {user_config}")
            # acm.update_super_config(d_sim, user_config)
            # acm.run_simulation()

        # if st.button('Evaluate design to get xf', type="secondary", use_container_width=True):
        #     st.write(f'update super_config.h at {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())}')
        #     path_to_dat = os.path.dirname(__file__) + f'/frameworkCodes/dat/{st.session_state.user_selected_motor}.dat'
        #     if os.path.exists(path_to_dat):
        #         os.remove(path_to_dat)
        #     acm.update_super_config(d_sim, user_config)
        #     # acm.run_simulation()
        #     # import optimize
        #     # optimize.optimize_main(params_list, self.d_sim, self.motor_name, max_p_value)