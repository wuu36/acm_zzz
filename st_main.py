
import os

import streamlit as st

from super_config import SuperConfig
import cplot
import matplotlib.pyplot as plt


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

        with st.spinner("生成配置..."):
            # 更新配置
            config.motor_name = selected_motor
            config.user_config['simulation']['sim.CL_TS'] = cl_ts
            config.user_config['simulation']['sim.NUMBER_OF_STEPS'] = num_steps
            config.user_config['test']['test.cmd_speed'] = cmd_speed
            config.user_config['test']['test.cmd_iQ'] = cmd_iq
            config.user_config['test']['test.TLoad'] = tload

            # 模式映射
            mode_map = {"开环VVVF": 11, "FOC电流": 3, "速度控制": 4}
            config.user_config['simulation']['sim.MODE_SELECT'] = mode_map[mode]
            
            # 生成C配置
            config.update_super_config()
        
        with st.spinner("编译仿真..."):
            # 编译
            if mode == "速度控制":
                target = "velocity_test"
            elif mode == "FOC电流" and tload > 0:
                target = "foc_load_test"
            else:
                target = "velocity_test"
            
            success = config.compile_simulation(target)

            if success:
                st.success("编译成功！")
            else:
                st.error("编译失败！")
                st.stop()
        
        with st.spinner("运行仿真..."):
            # 运行
            success = config.run_simulation(target)
            
            if success:
                st.success("仿真完成！")
            else:
                st.error("仿真失败！")
                st.stop()

    # 结果可视化
    st.header("📈 仿真结果")

    # 检查数据文件
    data_path = os.path.join(config.c_path, "..", "dat")
    data_file = []
    if os.path.exists(data_path):
        data_files = [f for f in os.listdir(data_path) if f.endswith('.dat')]
        
    if data_files:
        selected_file = st.selectbox(
            "选择数据文件",
            data_files,
            key="data_file"
        )

        if selected_file:
            try:
                df = cplot.read_data(os.path.join(data_path, selected_file))
                
                if df is not None and len(df) > 0:
                    # 绘图
                    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
                    
                    # 转速
                    axes[0].plot(df['time'], df.get('varOmega', df.get('omega', [])), 'b-', linewidth=1)
                    axes[0].set_ylabel('speed [rad/s]')
                    axes[0].set_title('motor speed')
                    axes[0].grid(True)

                    # 电流
                    if 'iQ' in df.columns:
                        axes[1].plot(df['time'], df['iQ'], 'r-', linewidth=1)
                        axes[1].set_ylabel('iQ [A]')
                        axes[1].set_title('Q-axis current')
                        axes[1].grid(True)
                    # 转矩

                    plt.tight_layout()
                    st.pyplot(fig)


            except Exception as e:
                st.error(f"读取数据失败: {e}")
    else:
        st.info("暂无数据文件，请先运行仿真")


if __name__ == "__main__":
    main()