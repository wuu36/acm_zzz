import os
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from super_config import SuperConfig

BASE_PATH = os.path.dirname(os.path.abspath(__file__))
C_PATH = os.path.join(BASE_PATH, "frameworkCodes", "c")
SIM_DAT_PATH = os.path.join(BASE_PATH, "frameworkCodes", "dat")
OUT_DAT_PATH = os.path.join(BASE_PATH, "dat")

COLORS = {'SEW100W': '#1f77b4', 'SEW200W': '#ff7f0e', 'CustomPMSM': '#2ca02c'}

def run_vvvf_test(motor_name, num_steps=50000, cmd_uD=0.0, cmd_uQ=5.0, tload=0.0, cl_ts=1e-4, tag=""):
    config = SuperConfig(motor_name)
    config.load_motor_library()
    config.load_user_config()

    motor_params = config.get_motor_params(motor_name)

    config.user_config['simulation']['sim.CL_TS'] = cl_ts
    config.user_config['simulation']['sim.NUMBER_OF_STEPS'] = num_steps
    config.user_config['simulation']['sim.MODE_SELECT'] = 11

    config.user_config['test']['test.cmd_speed'] = 0.0
    config.user_config['test']['test.cmd_iQ'] = 0.0
    config.user_config['test']['test.cmd_uD'] = cmd_uD
    config.user_config['test']['test.cmd_uQ'] = cmd_uQ
    config.user_config['test']['test.TLoad'] = tload

    config.user_config['motor']['motor.npp'] = motor_params['npp']
    config.user_config['motor']['motor.IN'] = motor_params['IN']
    config.user_config['motor']['motor.R'] = motor_params['R']
    config.user_config['motor']['motor.Ld'] = motor_params['Ld']
    config.user_config['motor']['motor.Lq'] = motor_params['Lq']
    config.user_config['motor']['motor.KE'] = motor_params['KE']
    config.user_config['motor']['motor.Js'] = motor_params['Js']
    config.user_config['motor']['motor.Vdc'] = motor_params['Vdc']

    config.update_super_config()

    success = config.compile_simulation("velocity_test")
    if not success:
        print(f"[FAIL] compile failed for {motor_name} {tag}")
        return None

    success = config.run_simulation("velocity_test")
    if not success:
        print(f"[FAIL] simulation failed for {motor_name} {tag}")
        return None

    sim_data_file = os.path.join(SIM_DAT_PATH, "test_motor_velocity.dat")
    label = f"{motor_name}_{tag}" if tag else motor_name
    unique_name = f"vvvf_{label}.dat"
    unique_path = os.path.join(OUT_DAT_PATH, unique_name)

    if os.path.exists(sim_data_file):
        shutil.copy2(sim_data_file, unique_path)
        df = pd.read_csv(unique_path)
        print(f"[OK] {label}: final_omega={df['varOmega'].iloc[-1]:.2f} rad/s, "
              f"peak_iQ={df['iQ'].abs().max():.3f} A, "
              f"peak_Tem={df['Tem'].abs().max():.4f} Nm, "
              f"steady_omega={df['varOmega'].iloc[-100:].mean():.2f} rad/s")
        return df
    else:
        print(f"[FAIL] data file not found at {sim_data_file}")
        return None

def plot_comparison(results, title, filename, y_vars, y_labels, y_units):
    fig, axes = plt.subplots(len(y_vars), 1, figsize=(12, 4 * len(y_vars)))
    if len(y_vars) == 1:
        axes = [axes]

    for i, (var, label, unit) in enumerate(zip(y_vars, y_labels, y_units)):
        for name, df in results.items():
            color = COLORS.get(name.split('_')[0], '#999999')
            axes[i].plot(df['time'], df[var], color=color, linewidth=1, label=name)
        axes[i].set_ylabel(f'{label} [{unit}]')
        axes[i].set_title(f'{label} Comparison - {title}')
        axes[i].grid(True, alpha=0.3)
        axes[i].legend(fontsize=9)

    axes[-1].set_xlabel('time [s]')
    plt.tight_layout()
    out_path = os.path.join(OUT_DAT_PATH, filename)
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"[PLOT] saved {out_path}")

def plot_motor_params_comparison():
    fig, ax = plt.subplots(figsize=(10, 6))
    config = SuperConfig()
    config.load_motor_library()

    params_keys = ['R', 'Ld', 'Lq', 'KE', 'Js']
    display_names = ['R [Ohm]', 'Ld [mH]', 'Lq [mH]', 'KE [Wb]', 'Js [1e-6 kg*m2]']
    scale_factors = [1, 1000, 1000, 1, 1e6]

    x = np.arange(len(params_keys))
    width = 0.25
    for i, motor_name in enumerate(['SEW100W', 'SEW200W', 'CustomPMSM']):
        p = config.get_motor_params(motor_name)
        vals = [p[k] * s for k, s in zip(params_keys, scale_factors)]
        ax.bar(x + i * width, vals, width, label=motor_name, color=list(COLORS.values())[i])

    ax.set_xticks(x + width)
    ax.set_xticklabels(display_names, fontsize=9)
    ax.set_title('Motor Parameters Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    out_path = os.path.join(OUT_DAT_PATH, 'motor_params_comparison.png')
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"[PLOT] saved {out_path}")

def analyze_vvvf_physics(results, cmd_uQ):
    print("\n" + "=" * 70)
    print("VVVF Open-Loop Voltage Test - Physics Analysis")
    print("=" * 70)

    config = SuperConfig()
    config.load_motor_library()

    for name, df in results.items():
        motor_name = name.split('_')[0]
        p = config.get_motor_params(motor_name)

        steady_omega = df['varOmega'].iloc[-200:].mean()
        peak_iQ = df['iQ'].iloc[:2000].abs().max()
        peak_Tem = df['Tem'].iloc[:2000].abs().max()

        omega_elec = steady_omega * p['npp']
        back_emf = omega_elec * p['KE']
        theoretical_omega = cmd_uQ / (p['npp'] * p['KE'])

        steady_iQ = df['iQ'].iloc[-200:].mean()

        npp_ke = p['npp'] * p['KE']
        uq_r = cmd_uQ / p['R']
        r_ke = p['R'] / p['KE']
        lr_tau = p['Ld'] / p['R'] * 1000
        t90 = df[df['varOmega'] >= 0.9 * steady_omega]['time'].iloc[0]
        rpm = steady_omega * 60 / (2 * 3.14159)

        print(f"\n--- {motor_name} ---")
        print(f"  steady speed:      {steady_omega:.2f} rad/s ({rpm:.1f} rpm)")
        print(f"  theoretical speed: {theoretical_omega:.2f} rad/s (uQ/(npp*KE)={cmd_uQ}/{npp_ke:.4f})")
        print(f"  back-EMF:          {back_emf:.3f} V (omega_elec*KE)")
        print(f"  peak iQ:           {peak_iQ:.3f} A (uQ/R={uq_r:.3f} A)")
        print(f"  peak torque:       {peak_Tem:.4f} Nm")
        print(f"  steady iQ:         {steady_iQ:.6f} A")
        print(f"  R/KE ratio:        {r_ke:.2f}")
        print(f"  L/R time const:    {lr_tau:.2f} ms")
        print(f"  accel time (~90%): {t90:.3f} s")

def main():
    os.makedirs(OUT_DAT_PATH, exist_ok=True)
    os.makedirs(SIM_DAT_PATH, exist_ok=True)

    print("=" * 70)
    print("Phase 1: 3-motor VVVF baseline test (uD=0V, uQ=5V, 50000 steps)")
    print("=" * 70)

    motors = ['SEW100W', 'SEW200W', 'CustomPMSM']
    baseline_results = {}
    for motor in motors:
        df = run_vvvf_test(motor, num_steps=50000, cmd_uD=0.0, cmd_uQ=5.0, tag="baseline")
        if df is not None:
            baseline_results[motor] = df

    plot_comparison(baseline_results, "VVVF Baseline (uD=0V, uQ=5V)",
                    "vvvf_baseline_comparison.png",
                    ['varOmega', 'iQ', 'Tem'],
                    ['Speed', 'Q-axis Current', 'Torque'],
                    ['rad/s', 'A', 'Nm'])

    print("\n" + "=" * 70)
    print("Phase 2: Voltage amplitude test (SEW100W, uQ=2/5/10V)")
    print("=" * 70)

    voltage_results = {}
    for uQ in [2.0, 5.0, 10.0]:
        df = run_vvvf_test('SEW100W', num_steps=50000, cmd_uD=0.0, cmd_uQ=uQ, tag=f"uQ{uQ}")
        if df is not None:
            voltage_results[f'SEW100W_uQ={uQ}V'] = df

    plot_comparison(voltage_results, "VVVF Voltage Impact (SEW100W)",
                    "vvvf_voltage_comparison.png",
                    ['varOmega', 'iQ', 'Tem'],
                    ['Speed', 'Q-axis Current', 'Torque'],
                    ['rad/s', 'A', 'Nm'])

    print("\n" + "=" * 70)
    print("Phase 3: Load torque test (SEW100W, uQ=5V, TLoad=0/0.05/0.1)")
    print("=" * 70)

    load_results = {}
    for tload in [0.0, 0.05, 0.1]:
        df = run_vvvf_test('SEW100W', num_steps=50000, cmd_uD=0.0, cmd_uQ=5.0, tload=tload, tag=f"TLoad{tload}")
        if df is not None:
            load_results[f'SEW100W_TLoad={tload}Nm'] = df

    plot_comparison(load_results, "VVVF Load Impact (SEW100W, uQ=5V)",
                    "vvvf_load_comparison.png",
                    ['varOmega', 'iQ', 'Tem'],
                    ['Speed', 'Q-axis Current', 'Torque'],
                    ['rad/s', 'A', 'Nm'])

    print("\n" + "=" * 70)
    print("Phase 4: Simulation steps test (SEW100W, uQ=5V)")
    print("=" * 70)

    step_results = {}
    for steps in [20000, 50000, 100000]:
        df = run_vvvf_test('SEW100W', num_steps=steps, cmd_uD=0.0, cmd_uQ=5.0, tag=f"steps{steps}")
        if df is not None:
            step_results[f'SEW100W_{steps // 1000}k'] = df

    plot_comparison(step_results, "VVVF Steps Impact (SEW100W)",
                    "vvvf_steps_comparison.png",
                    ['varOmega', 'iQ'],
                    ['Speed', 'Q-axis Current'],
                    ['rad/s', 'A'])

    print("\n" + "=" * 70)
    print("Phase 5: Physics Analysis")
    print("=" * 70)

    analyze_vvvf_physics(baseline_results, 5.0)

    plot_motor_params_comparison()

    print("\n" + "=" * 70)
    print("All tests completed. Check dat/ folder for plots.")
    print("=" * 70)

if __name__ == "__main__":
    main()