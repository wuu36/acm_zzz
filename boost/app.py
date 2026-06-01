import streamlit.components.v1 as components
import streamlit as st
from ui.sidebar import render_sidebar
from ui.metric_cards import render_metric_cards
# from viz.circuit_diagram import generate_boost_circuit_svg
from viz.circuit_diagram_v2 import generate_boost_circuit_svg
from core.transient import simulate_transient, get_steady_waveforms
from viz.waveforms import plot_il_waveform, plot_vout_waveform, plot_vsw_waveform

st.set_page_config(
    page_title="Boost 升压电路仿真器",
    page_icon="⚡",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.title("⚡ Boost 升压电路仿真器")

params = render_sidebar()
# print(params)

if params:
    results = render_metric_cards(params)

    if results is None:
        st.stop()

    st.markdown("---")
    st.subheader("电路拓扑图")
    svg_code = generate_boost_circuit_svg(params, results)
    components.html(svg_code, height=500)

    st.markdown("---")
    with st.spinner("运行时域仿真..."):
        D = results["D"]
        if params["control_mode"] == "闭环(自动稳定)":
            sim_mode = "closed_loop"
            Vout_target = params["Vout_target"]
            Kp = params["Kp"]
            Ki = params["Ki"]
        else:
            sim_mode = "open_loop"
            Vout_target = None
            Kp = 0
            Ki = 0

        sim_data = simulate_transient(
            Vin=params["Vin"], D=D, R=params["R"],
            L=params["L"], C=params["C"],
            RL=params["RL"], Vd=params["Vd"],
            f=params["f"], mode=sim_mode,
            Vout_target=Vout_target, Kp=Kp, Ki=Ki,
        )

    steady_wave = get_steady_waveforms(sim_data, params["f"], num_periods=10)
    
    st.subheader("仿真统计")
    col1, col2, col3, col4 = st.columns(4)
    col1.metric("仿真 Vout (稳态均值)", f"{sim_data['vC_steady_avg']:.2f} V")
    col2.metric("仿真 IL (稳态均值)", f"{sim_data['iL_steady_avg']:.2f} A")
    col3.metric("稳态 Vout 纹波", f"{(sim_data['vC_steady_peak'] - sim_data['vC_steady_min'])*1e3:.2f} mV")
    col4.metric("稳态 IL 峰值", f"{sim_data['iL_steady_peak']:.2f} A")

    st.markdown("---")
    st.subheader("电感电流波形 IL(t)")
    fig_il = plot_il_waveform(steady_wave, results, params)
    st.plotly_chart(fig_il, use_container_width=True)

    st.subheader("输出电压波形 Vout(t)")
    fig_vout = plot_vout_waveform(steady_wave, results, params)
    st.plotly_chart(fig_vout, use_container_width=True)

    st.subheader("开关节点电压 Vsw(t)")
    fig_vsw = plot_vsw_waveform(steady_wave, results, params)
    st.plotly_chart(fig_vsw, use_container_width=True)

    st.markdown("---")

    # st.subheader("稳态波形 (最后 3 个周期)")

    # col1, col2 = st.columns(2)
    # with col1:
    #     st.metric("仿真 Vout (稳态均值)", f"{sim_data['vC_steady_avg']:.2f} V")
    #     st.metric("仿真 IL (稳态均值)", f"{sim_data['iL_steady_avg']:.2f} A")
    # with col2:
    #     st.metric("稳态 Vout 纹波", f"{(sim_data['vC_steady_peak'] - sim_data['vC_steady_min'])*1e3:.2f} mV")
    #     st.metric("稳态 IL 峰值", f"{sim_data['iL_steady_peak']:.2f} A")

    # st.info("Phase 5 将使用 Plotly 绘制交互式波形图 IL(t), Vout(t), Vsw(t)")

    # st.markdown("---")
    # col3, col4 = st.columns(2)
    # with col3:
    #     st.subheader("增益曲线 (D vs Vout)")
    #     st.info("Phase 5 将在此处显示理想 vs 有损增益对比曲线")
    # with col4:
    #     st.subheader("参数扫描")
    #     st.info("Phase 5 将在此处显示 L/C/f sweep 分析")

    # col1, col2 = st.columns(2)
    # with col1:
    #     st.subheader("电感电流波形")
    #     st.info("Phase 5 将在此处显示 IL(t) 波形图")
    # with col2:
    #     st.subheader("输出电压波形")
    #     st.info("Phase 5 将在此处显示 Vout(t) 波形图")

    # col3, col4 = st.columns(2)
    # with col3:
    #     st.subheader("开关节点电压")
    #     st.info("Phase 5 将在此处显示 Vsw(t) 波形图")
    # with col4:
    #     st.subheader("增益曲线 (D vs Vout)")
    #     st.info("Phase 5 将在此处显示增益对比曲线")