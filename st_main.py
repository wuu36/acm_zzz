
import os

import streamlit as st
from super_config import SuperConfig
import cplot
import matplotlib.pyplot as plt
import st_interact


MODE_CONF = {
    "open_loop_vvvf": {"label": "Open-Loop VVVF", "mode_select": 11, "target": "vvvf_test",
                       "data_file": "test_motor_vvvf.dat"},
    "foc_current":    {"label": "FOC Current",    "mode_select": 3,  "target": "foc_test",
                       "data_file": "test_motor_foc.dat"},
    "foc_current_load": {"label": "FOC Load",     "mode_select": 3,  "target": "foc_load_test",
                         "data_file": "test_motor_foc_load.dat"},
    "speed_control":  {"label": "Speed Control",  "mode_select": 4,  "target": "velocity_test",
                       "data_file": "test_motor_velocity.dat"},
}


def streamlit_config():
    """streamlit页面配置"""
    st.set_page_config(
        page_title="emachinery_zzz",
        page_icon="⚡",
        layout="wide",
        initial_sidebar_state="expanded"
    )

def main():
    """streamlit主函数"""
    streamlit_config()

    st.title("⚡ Electric Machinery Simulation")
    st.markdown("---")
    
    config = SuperConfig()
    config.load_motor_library()
    config.load_user_config()

    with st.sidebar:
        st.header("🔌 电机选择")
        motor_list = list(config.motor_library.keys())
        selected_motor = st.selectbox("选择电机", motor_list, key="motor_select")

        st.header("⚙️ 仿真参数")
        cl_ts = st.number_input("控制周期 [s]", value=5e-5, format="%.5f", step=1e-5, key="cl_ts")
        num_steps = st.number_input("仿真步数", value=4000, step=1000, key="num_steps")

        st.header("🎯 测试指令")
        mode_key = st.radio(
            "控制模式",
            list(MODE_CONF.keys()),
            format_func=lambda k: f"{MODE_CONF[k]['label']}",
            key="mode_select"
        )

        if mode_key == "speed_control":
            cmd_speed = st.number_input("速度指令 [rad/s]", value=100.0, key="cmd_speed")
            cmd_iq = 0.0
            cmd_uD = 0.0
            cmd_uQ = 0.0
        elif "foc" in mode_key:
            cmd_iq = st.number_input("Q轴电流指令 [A]", value=1.0, key="cmd_iq")
            cmd_speed = 0.0
            cmd_uD = 0.0
            cmd_uQ = 0.0
        else:
            cmd_uD = st.number_input("D轴电压指令 [V]", value=0.0, key="cmd_uD")
            cmd_uQ = st.number_input("Q轴电压指令 [V]", value=5.0, key="cmd_uQ")
            cmd_speed = 0.0
            cmd_iq = 0.0

        tload = st.number_input("负载转矩 [Nm]", value=0.0, format="%.4f", key="tload")

        st.markdown("---")
        run_button = st.button("▶️ 运行仿真", type="primary")
    
    st.header("📊 电机参数")
    motor_params = config.get_motor_params(selected_motor)

    edit_mode = st.checkbox("启用参数编辑", value=False)
    
    if edit_mode and motor_params:
        edited_params = st_interact.motor_parameter_editor(motor_params)
        st.session_state['edited_motor_params'] = edited_params
        col1, col2, col3 = st.columns(3)
        with col1:
            st.metric("极对数", edited_params['npp'])
            st.metric("电阻", f"{edited_params['R']} Ω")
        with col2:
            st.metric("电感Ld", f"{edited_params['Ld']*1000:.2f} mH")
            st.metric("电感Lq", f"{edited_params['Lq']*1000:.2f} mH")
        with col3:
            st.metric("KE", f"{edited_params['KE']} Wb")
            st.metric("惯量", f"{edited_params['Js']*1e6:.2f} μkg·m²")
        st.info("参数已编辑, 点击运行仿真将使用修改后的参数")
    elif motor_params:
        col1, col2, col3 = st.columns(3)
        with col1:
            st.metric("极对数", motor_params['npp'])
            st.metric("电阻", f"{motor_params['R']} Ω")
        with col2:
            st.metric("电感Ld", f"{motor_params['Ld']*1000:.2f} mH")
            st.metric("电感Lq", f"{motor_params['Lq']*1000:.2f} mH")
        with col3:
            st.metric("KE", f"{motor_params['KE']} Wb")
            st.metric("惯量", f"{motor_params['Js']*1e6:.2f} μkg·m²")

    cfg = MODE_CONF[mode_key]

    if run_button:
        st.header("🔄 运行仿真")

        # Determine target: FOC with load → foc_load_test
        target = cfg["target"]
        if mode_key == "foc_current" and tload > 0:
            target = "foc_load_test"

        with st.spinner("生成配置..."):
            config.motor_name = selected_motor
            config.user_config['simulation']['sim.CL_TS'] = cl_ts
            config.user_config['simulation']['sim.NUMBER_OF_STEPS'] = num_steps
            config.user_config['test']['test.cmd_speed'] = cmd_speed
            config.user_config['test']['test.cmd_iQ'] = cmd_iq
            config.user_config['test']['test.TLoad'] = tload
            config.user_config["test"]["test.cmd_uD"] = cmd_uD
            config.user_config["test"]["test.cmd_uQ"] = cmd_uQ
            config.user_config['simulation']['sim.MODE_SELECT'] = cfg["mode_select"]
            config.update_super_config()
        
        with st.spinner("编译仿真..."):
            success = config.compile_simulation(target)
            if success:
                st.success("编译成功！")
            else:
                st.error("编译失败！")
                st.stop()
        
        with st.spinner("运行仿真..."):
            success = config.run_simulation(target)
            if success:
                st.success("仿真完成！")
            else:
                st.error("仿真失败！")
                st.stop()

    # 结果可视化
    st.header("📈 仿真结果")

    data_path = os.path.join(config.c_path, "..", "dat")
    data_files = []
    if os.path.exists(data_path):
        data_files = sorted(
            [f for f in os.listdir(data_path) if f.endswith('.dat')],
            key=lambda f: os.path.getmtime(os.path.join(data_path, f)),
            reverse=True
        )
        
    if data_files:
        default_idx = 0
        if cfg["data_file"] in data_files:
            default_idx = data_files.index(cfg["data_file"])
            
        selected_file = st.selectbox(
            "选择数据文件",
            data_files,
            key="data_file",
            index=default_idx
        )

        if selected_file:
            try:
                df = cplot.read_data(os.path.join(data_path, selected_file))
                if df is not None and len(df) > 0:
                    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
                    
                    axes[0].plot(df['time'], df.get('varOmega', df.get('omega', [])), 'b-', linewidth=1)
                    axes[0].set_ylabel('speed [rad/s]')
                    axes[0].set_title('motor speed')
                    axes[0].grid(True)

                    if 'iQ' in df.columns:
                        axes[1].plot(df['time'], df['iQ'], 'r-', linewidth=1)
                        axes[1].set_ylabel('iQ [A]')
                        axes[1].set_title('Q-axis current')
                        axes[1].grid(True)

                    if 'Tem' in df.columns:
                        axes[2].plot(df['time'], df['Tem'], 'g-', linewidth=1)
                        axes[2].set_ylabel('torque [Nm]')
                        axes[2].set_xlabel('time [s]')
                        axes[2].set_title('electromagnetic torque')
                        axes[2].grid(True)

                    plt.tight_layout()
                    st.pyplot(fig)

                    st.subheader("数据统计")
                    col1, col2 = st.columns(2)
                    with col1:
                        st.write(f"数据点数：{len(df)}")
                        st.write(f"仿真时长：{df['time'].iloc[-1]:.2f} s")
                    with col2:
                        if 'Tem' in df.columns:
                            st.write(f"稳态转矩: {df['Tem'].iloc[-100:].mean():.4f} Nm")

            except Exception as e:
                st.error(f"读取数据失败: {e}")
    else:
        st.info("暂无数据文件，请先运行仿真")


if __name__ == "__main__":
    main()
