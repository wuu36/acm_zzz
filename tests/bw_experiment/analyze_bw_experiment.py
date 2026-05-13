import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
matplotlib.rcParams['axes.unicode_minus'] = False

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
RESULT_DIR = os.path.join(PROJECT_ROOT, "dat", "bw_experiment")

CASES = {
    "1a_bw5":    {"speed_bw": 5,   "current_bw": 100, "label": "BW_speed=5, BW_curr=100", "group": "speed_sweep"},
    "1b_bw10":   {"speed_bw": 10,  "current_bw": 100, "label": "BW_speed=10, BW_curr=100", "group": "speed_sweep"},
    "1c_bw20":   {"speed_bw": 20,  "current_bw": 100, "label": "BW_speed=20, BW_curr=100", "group": "speed_sweep"},
    "1d_bw50":   {"speed_bw": 50,  "current_bw": 100, "label": "BW_speed=50, BW_curr=100", "group": "speed_sweep"},
    "2a_bw100":  {"speed_bw": 100, "current_bw": 100, "label": "BW_speed=100, BW_curr=100 [violate]", "group": "violate"},
    "2b_bw150":  {"speed_bw": 150, "current_bw": 100, "label": "BW_speed=150, BW_curr=100 [violate]", "group": "violate"},
    "3a_curr50": {"speed_bw": 20,  "current_bw": 50,  "label": "BW_speed=20, BW_curr=50", "group": "current_sweep"},
}

COLORS = {
    "1a_bw5":    "#1f77b4",
    "1b_bw10":   "#ff7f0e",
    "1c_bw20":   "#2ca02c",
    "1d_bw50":   "#d62728",
    "2a_bw100":  "#9467bd",
    "2b_bw150":  "#8c564b",
    "3a_curr50": "#e377c2",
}

CMD_SPEED = 100.0

def load_data(case_name):
    dat_path = os.path.join(RESULT_DIR, f"bw_exp_{case_name}.dat")
    with open(dat_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    time = np.array([float(r['time']) for r in rows])
    varOmega = np.array([float(r['varOmega']) for r in rows])
    iQ = np.array([float(r['iQ']) for r in rows])
    cmd_iQ = np.array([float(r['cmd_iQ']) for r in rows])
    cmd_speed = np.array([float(r['cmd_speed']) for r in rows])
    return {'time': time, 'varOmega': varOmega, 'iQ': iQ, 'cmd_iQ': cmd_iQ, 'cmd_speed': cmd_speed}

def calc_metrics(data, case_params):
    t = data['time']
    omega = data['varOmega']
    iq = data['iQ']

    cmd = CMD_SPEED
    final_omega = omega[-1]
    final_iq = iq[-1]
    speed_error_pct = abs(final_omega - cmd) / cmd * 100 if abs(cmd) > 1e-6 else 0

    min_omega = np.min(omega[:200])
    min_omega_time = t[np.argmin(omega[:200])]

    rise_idx = np.where(omega >= cmd * 0.9)[0]
    rise_time = t[rise_idx[0]] if len(rise_idx) > 0 else float('inf')

    peak_iq = np.max(iq[:2000])
    min_iq = np.min(iq[:2000])

    settling_mask = np.abs(omega - cmd) <= cmd * 0.02
    settling_idx = np.where(settling_mask)[0]
    settling_time = t[settling_idx[0]] if len(settling_idx) > 0 else float('inf')

    overshoot = (np.max(omega) - cmd) / cmd * 100 if np.max(omega) > cmd else 0

    iq_steady = np.mean(iq[-500:]) if len(iq) >= 500 else np.mean(iq[-100:])

    return {
        'Case': case_params['label'],
        'BW_speed': case_params['speed_bw'],
        'BW_current': case_params['current_bw'],
        'Final_speed': f'{final_omega:.2f}',
        'Speed_error%': f'{speed_error_pct:.1f}',
        'Min_speed': f'{min_omega:.1f}',
        'Min_speed_time(ms)': f'{min_omega_time*1000:.1f}',
        'Peak_iQ': f'{peak_iq:.3f}',
        'Steady_iQ': f'{iq_steady:.3f}',
        'Rise_time(ms)': f'{rise_time*1000:.1f}' if rise_time != float('inf') else 'N/A',
        'Settling_time(ms)': f'{settling_time*1000:.1f}' if settling_time != float('inf') else 'N/A(>400)',
        'Overshoot%': f'{overshoot:.1f}',
    }

def plot_sub(ax, cases_to_plot, field, ylabel, ref_line=None, title=None):
    for case_name in cases_to_plot:
        data = load_data(case_name)
        ax.plot(data['time']*1000, data[field], color=COLORS[case_name],
                label=CASES[case_name]['label'], linewidth=1)
    if ref_line:
        for y, color, lbl in ref_line:
            ax.axhline(y=y, color=color, linestyle='--', linewidth=1, label=lbl)
    ax.axhline(y=0, color='gray', linestyle=':', linewidth=0.5)
    ax.set_xlabel('time [ms]')
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title, fontsize=10)
    ax.legend(loc='best', fontsize=6)
    ax.grid(True, alpha=0.3)

def plot_combined():
    speed_sweep = ["1a_bw5", "1b_bw10", "1c_bw20", "1d_bw50"]
    violate = ["2a_bw100", "2b_bw150"]
    current_sweep = ["3a_curr50", "1c_bw20"]
    all_cases = list(CASES.keys())

    fig, axes = plt.subplots(3, 3, figsize=(16, 12))

    speed_ref = [(CMD_SPEED, 'k', 'cmd=100 rad/s')]
    iq_ref = [(0.78, 'k', 'iq=TLoad/Kt=0.78A')]

    plot_sub(axes[0,0], speed_sweep, 'varOmega', 'Speed [rad/s]',
             ref_line=speed_ref, title='Exp1: BW_speed Sweep - Speed')
    plot_sub(axes[0,1], speed_sweep, 'iQ', 'iQ [A]',
             ref_line=iq_ref, title='Exp1: BW_speed Sweep - iQ')
    plot_sub(axes[0,2], speed_sweep, 'cmd_iQ', 'cmd_iQ [A]',
             ref_line=iq_ref, title='Exp1: BW_speed Sweep - cmd_iQ')

    plot_sub(axes[1,0], violate, 'varOmega', 'Speed [rad/s]',
             ref_line=speed_ref, title='Exp2: Cascade Violate - Speed')
    plot_sub(axes[1,1], violate, 'iQ', 'iQ [A]',
             ref_line=iq_ref, title='Exp2: Cascade Violate - iQ')
    plot_sub(axes[1,2], violate, 'cmd_iQ', 'cmd_iQ [A]',
             title='Exp2: Cascade Violate - cmd_iQ')

    plot_sub(axes[2,0], current_sweep, 'varOmega', 'Speed [rad/s]',
             ref_line=speed_ref, title='Exp3: BW_current - Speed')
    plot_sub(axes[2,1], current_sweep, 'iQ', 'iQ [A]',
             ref_line=iq_ref, title='Exp3: BW_current - iQ')
    plot_sub(axes[2,2], all_cases, 'varOmega', 'Speed [rad/s]',
             ref_line=speed_ref, title='All Cases Overview - Speed')

    plt.tight_layout()
    out_path = os.path.join(RESULT_DIR, "bw_experiment_all.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved: {out_path}")

def main():
    plot_combined()

    metrics = []
    for case_name, case_params in CASES.items():
        data = load_data(case_name)
        m = calc_metrics(data, case_params)
        metrics.append(m)

    print("\n" + "="*100)
    print("Performance Metrics Summary:")
    print("="*100)
    header = list(metrics[0].keys())
    fmt_row = " | ".join(f"{k:<18}" for k in header)
    print(fmt_row)
    print("-"*len(fmt_row))
    for m in metrics:
        print(" | ".join(f"{m[k]:<18}" for k in header))
    print("="*100)

    with open(os.path.join(RESULT_DIR, "metrics_summary.csv"), 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(metrics)
    print(f"Metrics saved to: {os.path.join(RESULT_DIR, 'metrics_summary.csv')}")

if __name__ == "__main__":
    main()