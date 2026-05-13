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

def plot_speed_comparison(cases_to_plot, title, filename):
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))
    for case_name in cases_to_plot:
        data = load_data(case_name)
        ax.plot(data['time']*1000, data['varOmega'], color=COLORS[case_name],
                label=CASES[case_name]['label'], linewidth=1.5)
    ax.axhline(y=CMD_SPEED, color='black', linestyle='--', linewidth=1, label='cmd=100 rad/s')
    ax.axhline(y=0, color='gray', linestyle=':', linewidth=0.5)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Speed (rad/s)')
    ax.set_title(title)
    ax.legend(loc='best', fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(RESULT_DIR, filename)
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved: {out_path}")

def plot_iq_comparison(cases_to_plot, title, filename):
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))
    for case_name in cases_to_plot:
        data = load_data(case_name)
        ax.plot(data['time']*1000, data['iQ'], color=COLORS[case_name],
                label=CASES[case_name]['label'], linewidth=1.5)
    ax.axhline(y=0.78, color='black', linestyle='--', linewidth=1, label='steady iq=TLoad/Kt=0.78A')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('iQ (A)')
    ax.set_title(title)
    ax.legend(loc='best', fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(RESULT_DIR, filename)
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved: {out_path}")

def main():
    speed_sweep = ["1a_bw5", "1b_bw10", "1c_bw20", "1d_bw50"]
    violate = ["2a_bw100", "2b_bw150"]
    current_sweep = ["3a_curr50", "1c_bw20"]

    plot_speed_comparison(speed_sweep, "BW_speed Sweep: Speed Response", "speed_bw_sweep.png")
    plot_iq_comparison(speed_sweep, "BW_speed Sweep: iQ Response", "iq_bw_sweep.png")

    plot_speed_comparison(violate, "Cascade Violation: Speed Response", "speed_violate.png")
    plot_iq_comparison(violate, "Cascade Violation: iQ Response", "iq_violate.png")

    plot_speed_comparison(current_sweep, "BW_current Comparison: Speed Response", "speed_current_bw.png")
    plot_iq_comparison(current_sweep, "BW_current Comparison: iQ Response", "iq_current_bw.png")

    all_cases = list(CASES.keys())
    plot_speed_comparison(all_cases, "All Cases: Speed Response Overview", "speed_all_cases.png")
    plot_iq_comparison(all_cases, "All Cases: iQ Response Overview", "iq_all_cases.png")

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