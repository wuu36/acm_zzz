import plotly.graph_objects as go
import numpy as np
from core.steady_state import solve_ccm_ideal, solve_ccm_with_loss, solve_dcm_gain, D_MAX

def plot_gain_curve(Vin, R, L, C, f, RL, Vd, current_D=None):
    D_arr = np.linspace(0.01, D_MAX, 200)
    Vout_ideal = np.array([solve_ccm_ideal(Vin, d) for d in D_arr])
    Vout_lossy = np.array([solve_ccm_with_loss(Vin, d, R, RL, Vd)[0] for d in D_arr])
    gain_ideal = Vout_ideal / Vin
    gain_lossy = Vout_lossy / Vin

    fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            x=D_arr, y=gain_ideal,
            name="理想增益 (无损耗)",
            line=dict(color="#2196F3", width=2),
        )
    )

    fig.add_trace(
        go.Scatter(
            x=D_arr, y=gain_lossy,
            name=f"实际增益 (RL={RL}Ω)",
            line=dict(color="#F44336", width=2),
        )
    )

    fig.add_vrect(
        x0=0.85, x1=D_MAX,
        fillcolor="rgba(255,0,0,0.08)", line_width=0,
        annotation_text="增益下降区",
    )

    if current_D is not None:
        current_gain_ideal = solve_ccm_ideal(Vin, current_D) / Vin
        current_gain_lossy = solve_ccm_with_loss(Vin, current_D, R, RL, Vd)[0] / Vin
        fig.add_trace(
            go.Scatter(
                x=[current_D], y=[current_gain_lossy],
                name=f"当前 D={current_D:.2f}",
                mode="markers",
                marker=dict(color="#FF9800", size=12, symbol="star"),
            )
        )

    fig.update_xaxes(title_text="占空比 D", range=[0, D_MAX + 0.02])
    fig.update_yaxes(title_text="增益 M = Vout/Vin")

    fig.update_layout(
        height=350,
        showlegend=True,
        margin=dict(l=60, r=30, t=30, b=40),
        template="plotly_white",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    )

    return fig

def plot_param_sweep_sensitivity(Vin, R, L, C, f, RL, Vd, D, sweep_param="L"):
    import numpy as np
    from core.steady_state import solve_steady_state

    if sweep_param == "L":
        L_range = np.logspace(np.log10(1e-7), np.log10(1e-2), 80)
        delta_IL_arr = []
        delta_Vout_arr = []
        mode_arr = []
        for l_val in L_range:
            r = solve_steady_state(Vin, D, R, l_val, C, f, RL, Vd)
            delta_IL_arr.append(r["delta_IL"])
            delta_Vout_arr.append(r["delta_Vout"])
            mode_arr.append(r["mode"])
        x = L_range * 1e6
        x_label = "电感 L (μH)"

    elif sweep_param == "C":
        C_range = np.logspace(np.log10(1e-7), np.log10(1e-1), 80)
        delta_IL_arr = []
        delta_Vout_arr = []
        mode_arr = []
        for c_val in C_range:
            r = solve_steady_state(Vin, D, R, L, c_val, f, RL, Vd)
            delta_IL_arr.append(r["delta_IL"])
            delta_Vout_arr.append(r["delta_Vout"])
            mode_arr.append(r["mode"])
        x = C_range * 1e6
        x_label = "电容 C (μF)"

    elif sweep_param == "f":
        f_range = np.logspace(np.log10(1e4), np.log10(1e6), 80)
        delta_IL_arr = []
        delta_Vout_arr = []
        mode_arr = []
        for f_val in f_range:
            r = solve_steady_state(Vin, D, R, L, C, f_val, RL, Vd)
            delta_IL_arr.append(r["delta_IL"])
            delta_Vout_arr.append(r["delta_Vout"])
            mode_arr.append(r["mode"])
        x = f_range * 1e-3
        x_label = "开关频率 f (kHz)"

    fig = make_subplots(
        rows=2, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        subplot_titles=(f"ΔIL vs {sweep_param}", f"ΔVout vs {sweep_param}"),
    )

    fig.add_trace(
        go.Scatter(
            x=x, y=np.array(delta_IL_arr) * 1e3,
            name="ΔIL (mA)",
            line=dict(color="#2196F3", width=2),
        ),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=x, y=np.array(delta_Vout_arr) * 1e3,
            name="ΔVout (mV)",
            line=dict(color="#F44336", width=2),
        ),
        row=2, col=1,
    )

    ccm_mask = [m == "CCM" for m in mode_arr]
    dcm_mask = [m == "DCM" for m in mode_arr]

    if any(dcm_mask):
        dcm_idx = [i for i, m in enumerate(mode_arr) if m == "DCM"]
        first_dcm = dcm_idx[0]
        fig.add_vline(
            x=x[first_dcm], line_dash="dash", line_color="#FF9800",
            annotation_text="CCM→DCM边界",
            row=1, col=1,
        )

    fig.update_xaxes(title_text=x_label, type="log", row=2, col=1)
    fig.update_yaxes(title_text="ΔIL (mA)", row=1, col=1)
    fig.update_yaxes(title_text="ΔVout (mV)", row=2, col=1)

    fig.update_layout(
        height=500,
        showlegend=True,
        margin=dict(l=60, r=30, t=50, b=40),
        template="plotly_white",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    )

    return fig