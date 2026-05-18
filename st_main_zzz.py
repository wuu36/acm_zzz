import os 
import sys
import datetime
import yaml
import streamlit as st
import st_interact_zzz as interact
import user_script_main as user

def streamlit_config():
    print("\n", '-='*15, datetime.datetime.now(), '=-'*15)
    st.set_page_config(layout="wide")
    # sider bar appearance settings
    with st.sidebar:
        st.markdown(
            """
        <style>
        [data-testid="stSidebar"][aria-expanded="true"]{
            min-width: 450px;
            max-width: 1450px;
        }
            """,
                unsafe_allow_html=True,
        )
    return

def main():
    st.session_state.DEFAULT_MOTOR_INDEX = 0
    st.session_state.DEFAULT_USER_INDEX = 2
    streamlit_config()

    user_selected_mode = interact.user_selected_mode()
    user_history = interact.get_user_history()
    user_history = interact.clear_history_module(user_history)
    
    with open(os.path.join(os.path.dirname(__file__), 'user_config.yaml'), encoding='utf-8') as f:
        user_config = yaml.load(f, Loader=yaml.FullLoader)

    with st.sidebar:
        with st.expander("仿真参数"):
            st.write(user_config['simulation'])

    d_sim = interact.option_select_motor(user_history, user_config)
    d_sim = interact.option_select_algorithm(d_sim, user_config)
    d_sim = user.user_pre_process(d_sim, user_config)
    interact.save_d_sim_2_dat_foler(d_sim)
    # print(f"\nd_sim : {d_sim}")


    # print("test")


if __name__ == '__main__':
    main()