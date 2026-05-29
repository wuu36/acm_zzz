import streamlit.components.v1 as components
import streamlit as st
from ui.sidebar import render_sidebar
from ui.metric_cards import render_metric_cards
# from viz.circuit_diagram import generate_boost_circuit_svg
from viz.circuit_diagram_v2 import generate_boost_circuit_svg

st.set_page_config(
    page_title="Boost 升压电路仿真器",
    page_icon="⚡",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.title("⚡ Boost 升压电路仿真器")

params = render_sidebar()

if params:
    results = render_metric_cards(params)

    st.markdown("---")
    st.subheader("电路拓扑图")
    svg_code = generate_boost_circuit_svg(params, results)
    components.html(svg_code, height=500)

    st.markdown("---")
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("电感电流波形")
        st.info("Phase 5 将在此处显示 IL(t) 波形图")
    with col2:
        st.subheader("输出电压波形")
        st.info("Phase 5 将在此处显示 Vout(t) 波形图")

    col3, col4 = st.columns(2)
    with col3:
        st.subheader("开关节点电压")
        st.info("Phase 5 将在此处显示 Vsw(t) 波形图")
    with col4:
        st.subheader("增益曲线 (D vs Vout)")
        st.info("Phase 5 将在此处显示增益对比曲线")