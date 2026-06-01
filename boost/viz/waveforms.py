import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np


def plot_il_waveform(steady_wave, results, params):
    t_us = steady_wave["t"] * 1e6
    iL = steady_wave["iL"]
    sw = steady_wave["switch_state"]
    IL_avg = results["IL_avg"]
    delta_IL = results["delta_IL"]
    IL_peak = IL_avg + delta_IL / 2
    IL_min = IL_avg - delta_IL / 2

    fig = make_subplots(
        rows=2, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        row_heights=[0.75, 0.25],
        subplot_titles=("电感电流 IL(t)", "开关状态"),
    )

    fig.add_trace(
        go.Scatter(
            x=t_us, y=iL,
            name="IL",
            line=dict(color="#2196F3", width=2),
        ),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=[t_us[0], t_us[-1]], y=[IL_avg, IL_avg],
            name=f"IL_avg={IL_avg:.2f}A", line=dict(color="#FF9800", width=1.5, dash="dash"),
            mode="lines", showlegend=True,
        ),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=[t_us[0], t_us[-1]], y=[IL_peak, IL_peak],
            name=f"IL_peak={IL_peak:.2f}A", line=dict(color="#F44336", width=1, dash="dot"),
            mode="lines", showlegend=True,
        ),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=[t_us[0], t_us[-1]], y=[IL_min, IL_min],
            name=f"IL_min={IL_min:.2f}A", line=dict(color="#4CAF50", width=1, dash="dot"),
            mode="lines", showlegend=True,
        ),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=t_us, y=sw, name="SW",
            line=dict(color="#9E9E9E", width=1, shape="hv"),
            fill="tozeroy", fillcolor="rgba(158,158,158,0.15)",
        ),
        row=2, col=1,
    )
    
    fig.update_yaxes(title_text="电流 (A)", row=1, col=1)
    fig.update_yaxes(title_text="ON/OFF", row=2, col=1, range=[-0.1, 1.3])
    fig.update_xaxes(title_text="时间 (μs)", row=2, col=1)

    fig.update_layout(
        height=450, showlegend=True,
        margin=dict(l=60, r=30, t=50, b=40),
        template="plotly_white",
        # legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    )

    return fig

def plot_vout_waveform(steady_wave, results, params):
    t_us = steady_wave["t"] * 1e6
    vC = steady_wave["vC"]
    Vout = results["Vout"]
    delta_Vout = results["delta_Vout"]

    fig = go.Figure()

    fig.add_trace(
        go.Scatter(x=t_us, y=vC, name="Vout", line=dict(color="#4CAF50", width=2)),
    )

    fig.add_trace(
        go.Scatter(
            x=[t_us[0], t_us[-1]], y=[Vout, Vout],
            name=f"Vout_avg={Vout:.2f}V", line=dict(color="#FF9800", width=1.5, dash="dash"),
            mode="lines", showlegend=True,
        )
    )

    fig.add_trace(
        go.Scatter(
            x=[t_us[0], t_us[-1]], y=[Vout - delta_Vout/2, Vout - delta_Vout/2],
            name=f"ΔVout={delta_Vout*1e3:.1f}mV (下界)",
            line=dict(color="#F44336", width=1, dash="dot"), mode="lines", showlegend=True,
        )
    )

    fig.add_trace(
        go.Scatter(
            x=[t_us[0], t_us[-1]], y=[Vout + delta_Vout/2, Vout + delta_Vout/2],
            name=f"ΔVout={delta_Vout*1e3:.1f}mV (上界)",
            line=dict(color="#F44336", width=1, dash="dot"), mode="lines", showlegend=True,
        )
    )

    fig.update_yaxes(title_text="电压 (V)")
    fig.update_xaxes(title_text="时间 (μs)")

    fig.update_layout(
        height=350, showlegend=True,
        margin=dict(l=60, r=30, t=30, b=40),
        template="plotly_white",
        # legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    )

    return fig

def plot_vsw_waveform(steady_wave, results, params):
    t_us = steady_wave["t"] * 1e6
    vC = steady_wave["vC"]
    sw = steady_wave["switch_state"]
    Vin = params["Vin"]

    vsw = np.where(sw > 0.5, 0.0, vC)

    fig = make_subplots(
        rows=2, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        row_heights=[0.75, 0.25],
        subplot_titles=("开关节点电压 Vsw(t)", "开关状态"),
    )

    fig.add_trace(
        go.Scatter(x=t_us, y=vsw, name="Vsw", line=dict(color="#9C27B0", width=2)),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=[t_us[0], t_us[-1]], y=[Vin, Vin],
            name=f"Vin={Vin:.1f}V", line=dict(color="#2196F3", width=1.5, dash="dash"),
            mode="lines", showlegend=True,
        ),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=t_us, y=sw, name="SW",
            line=dict(color="#9E9E9E", width=1, shape="hv"),
            fill="tozeroy", fillcolor="rgba(158,158,158,0.15)",
        ),
        row=2, col=1,
    )

    fig.update_yaxes(title_text="电压 (V)", row=1, col=1)
    fig.update_yaxes(title_text="ON/OFF", row=2, col=1, range=[-0.1, 1.3])
    fig.update_xaxes(title_text="时间 (μs)", row=2, col=1)

    fig.update_layout(
        height=450, showlegend=True,
        margin=dict(l=60, r=30, t=50, b=40),
        template="plotly_white",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    )

    return fig
