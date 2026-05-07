

import streamlit as st

from super_config import SuperConfig


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

    # 标题
    st.title("⚡ Electric Machinery Simulation")
    st.markdown("---")
    
    # 初始化配置
    config = SuperConfig()
    config.load_motor_library()
    config.load_user_config()

    # 侧边栏： 电机选择
    with st.sidebar:
        st.header("🔌 电机选择")
        motor_list = list(config.motor_library.keys())
        selected_motor = st.selectbox(
            "选择电机",
            motor_list,
            key="motor_select"
        )
        st.header("⚙️ 仿真参数")
        # 时间步长
        cl_ts = st.number_input(
            "控制周期 [s]",
            value=1e-4,
            format="%.4f",
            key="cl_ts"
        )
        # 仿真步数
        num_steps = st.number_input(
            "仿真步数",
            value=100000,
            step=10000,
            key="num_steps"
        )

        st.header("🎯 测试指令")
        # 测试模式
        mode = st.radio(
            "控制模式",
            ["开环VVVF", "FOC电流", "速度控制"],
            key="mode_select"
        )

        # 根据模式显示不同参数
        if mode == "速度控制":
            cmd_speed = st.number_input(
                "速度指令 [rad/s]",
                value=100.0,
                key="cmd_speed"
            )
            cmd_iq = 0.0
        elif mode == "FOC电流":
            cmd_iq = st.number_input(
                "Q轴电流指令 [A]",
                value=1.0,
                key="cmd_iq"
            )
            cmd_speed = 0.0
        else:
            cmd_speed = 0.0
            cmd_iq = 0.0

        # 负载设置
        tload = st.number_input(
            "负载转矩 [Nm]",
            value=0.0,
            format="%.4f",
            key="tload"
        )

        # 运行按钮
        st.markdown("---")
        run_button = st.button("▶️ 运行仿真", type="primary")
    
    # 主区域：参数显示和结果可视化
    # 显示电机参数
    st.header("📊 电机参数")
    motor_params = config.get_motor_params(selected_motor)
    if motor_params:
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

    # 点击运行按钮
    if run_button:
        st.header("🔄 运行仿真")



if __name__ == "__main__":
    main()