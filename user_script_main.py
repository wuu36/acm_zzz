
import streamlit as st
import os 

import yaml


def user_pre_process(d_sim, user_config):
    # TODO: 你可以任意更改下面内容
    """
    eg:
    func1()
    func2()
    ...
    """
    return d_sim

def user_bezier_super_config(motor_name, data, user_extend_settings, super_config_C_content, super_config_header_content):
    if user_extend_settings:
        if user_extend_settings.get("bezier_moo"):
            print("bezier")
        elif user_extend_settings.get("bezier_C_save"):
            print("bezier_C_save")


    else:
        print("beizer else")
        if data['user'].get('bezier_order'):
            print(111111)
    return super_config_C_content, super_config_header_content
    

def user_cplot_post_process(d_sim, user_plot_config, post_run):
    #TODO: 你可以任意更改下面的内容
    if post_run:
        if not os.path.exists(os.path.join(os.path.dirname(__file__), 'session_state.yaml')):
            raise FileNotFoundError(f"文件不存在: {os.path.join(os.path.dirname(__file__), 'session_state.yaml')}")
        with open(os.path.join(os.path.dirname(__file__), 'session_state.yaml'), 'r') as f:
            session_state = yaml.load(f, Loader=yaml.FullLoader)
            bool_OverwriteUserConfigYaml = session_state['bool_OverwriteUserConfigYaml']
    else:
        bool_OverwriteUserConfigYaml = st.session_state.bool_OverwriteUserConfigYaml
        
    def _update(user_no, yaml_frame, d_sim, user_plot_config):
        if d_sim['user.who_is_user'] == user_no:
            with open(os.path.dirname(__file__) + yaml_frame, encoding='utf-8') as f:
                user_plot_config_overwrite = yaml.load(f, Loader=yaml.FullLoader)
                user_plot_config['signal_library'].extend(user_plot_config_overwrite['signal_library'])
                for key, value in user_plot_config_overwrite['cplot'].items():
                    if key == 'subplot':
                        user_plot_config['cplot'][key].extend(value)
                    else:
                        user_plot_config['cplot'][key] = value
            if bool_OverwriteUserConfigYaml:
                user_plot_config['cplot']['subplot'] = user_plot_config_overwrite['cplot']['subplot']
    
    _update(101976, '/user_config_cjh.yaml', d_sim, user_plot_config)
    _update(101616, '/user_config_test.yaml', d_sim, user_plot_config)
    
        
