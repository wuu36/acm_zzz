"""FOC closed-loop speed control simulation for automotive water pump.

Application: Automotive electronic water pump (12V system)
Power: 100-150W
Speed range: 3000-4000 rpm

Run:
    uv run python examples/foc_waterpump_simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt
from acmzzz.core import (
    MachineParams,
    MachineState,
    ACMachine,
    dynamics_machine,
    rk4_step,
)
from acmzzz.control import (
    ControllerParams,
    FOCConfig,
    MotorController,
)


# ============================================================
# Automotive Water Pump Motor Parameters
# ============================================================
# Typical 12V 100W PMSM for automotive water pump
#
# Key constraints:
# - DC bus voltage: 13.5V (12V automotive system)
# - Max phase voltage (linear modulation): Vdc/√3 ≈ 7.8V
# - Max phase voltage (over-modulation): Vdc/2 ≈ 6.75V
# - Power: 100W, efficiency ~85%
# - Current: 8-10A rated
# - Speed: 3000-4000 rpm

WATERPUMP_MOTOR_PARAMS = MachineParams(
    npp=4,              # Pole pairs (typical for small PMSM)
    R=0.08,             # Phase resistance [Ω] (low voltage motor)
    Ld=0.15e-3,         # d-axis inductance [H] = 0.15 mH
    Lq=0.18e-3,         # q-axis inductance [H] = 0.18 mH (slightly higher)
    KE=0.0045,          # PM flux linkage [Wb]
    Js=0.00008,         # Inertia [kg.m²] (small pump + impeller)
)

# DC bus voltage
VDC = 13.5             # Automotive 12V system nominal voltage

# Simulation settings
SPEED_COMMAND_RPM = 3500.0  # Target speed for tuning
SIMULATION_TIME = 1.0       # 1 second (pump has low inertia, fast response)


# ============================================================
# Parameter Tuning Experiments for Water Pump
# ============================================================

EXPERIMENTS = [
    {
        'name': 'wp_exp0_baseline',
        'desc': 'Baseline conservative params',
        'params': {
            'speed_Kp': 0.001,
            'speed_Ki': 0.5,
            'speed_limit': 5.0,      # Max iq command [A]
            'current_Kp': 0.5,       # Conservative for low inductance
            'current_Ki': 50.0,
            'current_limit': 7.5,    # Max voltage ≈ Vdc/√3
        }
    },
    {
        'name': 'wp_exp1_raise_voltage',
        'desc': 'Increase voltage limit',
        'params': {
            'speed_Kp': 0.001,
            'speed_Ki': 0.5,
            'speed_limit': 5.0,
            'current_Kp': 0.5,
            'current_Ki': 50.0,
            'current_limit': 13.0,   # Allow over-modulation
        }
    },
    {
        'name': 'wp_exp2_raise_current_limit',
        'desc': 'Increase current command limit',
        'params': {
            'speed_Kp': 0.001,
            'speed_Ki': 0.5,
            'speed_limit': 12.0,     # Allow higher current for faster acceleration
            'current_Kp': 0.5,
            'current_Ki': 50.0,
            'current_limit': 13.0,
        }
    },
    {
        'name': 'wp_exp3_increase_current_pi',
        'desc': 'Increase current loop PI (critical for low inductance)',
        'params': {
            'speed_Kp': 0.001,
            'speed_Ki': 0.5,
            'speed_limit': 12.0,
            'current_Kp': 2.0,       # Higher for fast current response
            'current_Ki': 200.0,     # Higher integral gain
            'current_limit': 13.0,
        }
    },
    {
        'name': 'wp_exp4_increase_speed_pi',
        'desc': 'Increase speed loop PI gains',
        'params': {
            'speed_Kp': 0.005,       # Higher proportional
            'speed_Ki': 5.0,         # Higher integral
            'speed_limit': 12.0,
            'current_Kp': 2.0,
            'current_Ki': 200.0,
            'current_limit': 13.0,
        }
    },
    {
        'name': 'wp_exp5_optimized',
        'desc': 'Optimized params for water pump',
        'params': {
            'speed_Kp': 0.01,
            'speed_Ki': 10.0,
            'speed_limit': 15.0,     # Allow short burst current
            'current_Kp': 3.0,       # High gain for low L motor
            'current_Ki': 300.0,
            'current_limit': 13.0,
        }
    },
    {
        'name': 'wp_exp6_auto_design',
        'desc': 'Auto-designed gains',
        'params': {
            'speed_Kp': 0.01,
            'speed_Ki': 10.0,
            'speed_limit': 15.0,
            'current_Kp': 3.0,
            'current_Ki': 300.0,
            'current_limit': 13.0,
        },
        'use_auto_design': True,
        'current_bw_hz': 2000.0,     # High bandwidth for low inductance
        'speed_bw_hz': 100.0,        # Speed bandwidth
    },
]


def run_single_experiment(exp_config: dict, motor_params: MachineParams) -> dict:
    """Run a single FOC simulation experiment."""
    exp_name = exp_config['name']
    exp_desc = exp_config['desc']
    params_dict = exp_config['params']

    print("\n" + "=" * 60)
    print(f"Experiment: {exp_name}")
    print(f"Description: {exp_desc}")
    print("=" * 60)

    # Motor Setup
    state = MachineState()
    machine = ACMachine(params=motor_params, state=state)

    print(f"\nMotor Parameters (Water Pump PMSM):")
    print(f"  Pole pairs: {motor_params.npp}")
    print(f"  Resistance: {motor_params.R:.3f} Ohm")
    print(f"  Ld: {motor_params.Ld*1000:.2f} mH, Lq: {motor_params.Lq*1000:.2f} mH")
    print(f"  PM flux (KE): {motor_params.KE:.4f} Wb")
    print(f"  Inertia: {motor_params.Js:.5f} kg.m^2")
    print(f"  DC Bus Voltage: {VDC:.1f} V")
    print(f"  Max phase voltage (linear): {VDC/np.sqrt(3):.2f} V")

    # Controller Setup
    ctrl_params = ControllerParams(
        CL_TS=1e-4,          # 0.1 ms current loop (10 kHz)
        VL_TS=1e-3,          # 1 ms speed loop (1 kHz)
        VL_ceiling=10,       # Speed loop every 10 current loops
        speed_Kp=params_dict['speed_Kp'],
        speed_Ki=params_dict['speed_Ki'],
        speed_limit=params_dict['speed_limit'],
        current_Kp=params_dict['current_Kp'],
        current_Ki=params_dict['current_Ki'],
        current_limit=params_dict['current_limit'],
    )

    foc_config = FOCConfig(
        control_mode='speed',
        id_command_strategy='mtpa',  # id=0 for surface PMSM
    )

    controller = MotorController(
        motor_params=motor_params,
        controller_params=ctrl_params,
        foc_config=foc_config,
    )

    # Apply auto_design if specified
    if exp_config.get('use_auto_design', False):
        controller.auto_design_gains(
            current_bw_hz=exp_config.get('current_bw_hz', 2000.0),
            speed_bw_hz=exp_config.get('speed_bw_hz', 100.0)
        )
        params_dict['speed_Kp'] = controller.speed_reg.Kp
        params_dict['speed_Ki'] = controller.speed_reg.Ki
        params_dict['current_Kp'] = controller.id_reg.Kp
        params_dict['current_Ki'] = controller.id_reg.Ki

    print(f"\nController Parameters:")
    print(f"  Current loop: {ctrl_params.get_current_loop_rate_hz()/1000:.1f} kHz")
    print(f"  Speed loop: {ctrl_params.get_speed_loop_rate_hz():.1f} Hz")
    print(f"  Speed PI: Kp={params_dict['speed_Kp']:.4f}, Ki={params_dict['speed_Ki']:.2f}")
    print(f"  Current PI: Kp={params_dict['current_Kp']:.2f}, Ki={params_dict['current_Ki']:.1f}")
    print(f"  Current limit (iq cmd): {params_dict['speed_limit']:.1f} A")
    print(f"  Voltage limit: {params_dict['current_limit']:.1f} V")

    # Simulation Setup
    x = np.zeros(5, dtype=np.float64)
    x[2] = motor_params.KE  # KA = KE for PMSM at id=0

    dt = ctrl_params.CL_TS
    n_steps = int(SIMULATION_TIME / dt)

    # Data logging
    times = np.zeros(n_steps)
    speed_rpm_log = np.zeros(n_steps)
    speed_cmd_log = np.zeros(n_steps)
    torque_log = np.zeros(n_steps)
    id_log = np.zeros(n_steps)
    iq_log = np.zeros(n_steps)
    ud_log = np.zeros(n_steps)
    uq_log = np.zeros(n_steps)

    # Load torque: simulate pump load (proportional to speed^2)
    # T_load = k * omega^2 (typical pump characteristic)
    # Assume 0.23 Nm at 3500 rpm → k = 0.23 / (367^2) = 1.7e-6
    pump_load_coeff = 1.7e-6

    print(f"\nSimulation Settings:")
    print(f"  Time step: {dt*1000:.2f} ms")
    print(f"  Total time: {SIMULATION_TIME} s")
    print(f"  Speed command: {SPEED_COMMAND_RPM} rpm")
    print(f"  Pump load coefficient: {pump_load_coeff:.2e} Nm/(rad/s)^2")

    # Run Simulation
    print(f"\nRunning simulation...")
    t = 0.0
    controller.set_speed_command(SPEED_COMMAND_RPM)

    for step in range(n_steps):
        times[step] = t

        theta_mech = x[0]
        omega_mech = x[1]
        KA = x[2]
        iD = x[3]
        iQ = x[4]

        theta_elec = theta_mech * motor_params.npp
        omega_elec = omega_mech * motor_params.npp

        controller.execute(
            omega_r_elec=omega_elec,
            id_=iD,
            iq=iQ,
            theta_d=theta_elec,
            Vdc=VDC
        )

        ud, uq = controller.get_voltage_commands_dq()

        machine.state.udq[0] = ud
        machine.state.udq[1] = uq

        # Pump load torque (quadratic characteristic)
        T_load = pump_load_coeff * omega_mech**2
        machine.state.Tload = T_load

        x = rk4_step(dynamics_machine, t, x, dt, machine)

        speed_rpm = omega_mech * 60.0 / (2.0 * np.pi)
        speed_rpm_log[step] = speed_rpm
        speed_cmd_log[step] = SPEED_COMMAND_RPM

        Tem = 1.5 * motor_params.npp * KA * iQ
        torque_log[step] = Tem

        id_log[step] = iD
        iq_log[step] = iQ
        ud_log[step] = ud
        uq_log[step] = uq

        t += dt

    # Calculate metrics
    final_speed = speed_rpm_log[-1]
    steady_state_error = abs(final_speed - SPEED_COMMAND_RPM)
    steady_state_error_pct = steady_state_error / SPEED_COMMAND_RPM * 100

    # Steady state analysis (last 20%)
    n_steady = int(n_steps * 0.2)
    steady_speed = speed_rpm_log[-n_steady:]
    steady_torque = torque_log[-n_steady:]
    steady_iq = iq_log[-n_steady:]

    speed_std = np.std(steady_speed)
    torque_mean = np.mean(steady_torque)

    max_iq = np.max(np.abs(iq_log))
    max_uq = np.max(np.abs(uq_log))

    # Rise time (10% to 90%)
    idx_10 = np.argmax(speed_rpm_log >= 0.1 * SPEED_COMMAND_RPM)
    idx_90 = np.argmax(speed_rpm_log >= 0.9 * SPEED_COMMAND_RPM)
    rise_time = times[idx_90] - times[idx_10] if idx_90 > idx_10 else 0.0

    # Overshoot
    max_speed = np.max(speed_rpm_log)
    overshoot_pct = (max_speed - SPEED_COMMAND_RPM) / SPEED_COMMAND_RPM * 100 if max_speed > SPEED_COMMAND_RPM else 0.0

    # Expected torque at final speed
    omega_final = final_speed * 2 * np.pi / 60
    expected_load = pump_load_coeff * omega_final**2

    print(f"\nResults:")
    print(f"  Final speed: {final_speed:.1f} rpm")
    print(f"  Steady state error: {steady_state_error:.1f} rpm ({steady_state_error_pct:.2f}%)")
    print(f"  Speed oscillation (std): {speed_std:.2f} rpm")
    print(f"  Steady torque: {torque_mean:.3f} Nm (expected load: {expected_load:.3f} Nm)")
    print(f"  Rise time (10-90%): {rise_time:.3f} s")
    print(f"  Overshoot: {overshoot_pct:.1f}%")
    print(f"  Max iq: {max_iq:.2f} A")
    print(f"  Max uq: {max_uq:.2f} V (limit: {VDC/np.sqrt(3):.2f} V linear)")

    return {
        'name': exp_name,
        'desc': exp_desc,
        'params': params_dict,
        'metrics': {
            'final_speed': final_speed,
            'steady_state_error': steady_state_error,
            'steady_state_error_pct': steady_state_error_pct,
            'speed_std': speed_std,
            'torque_mean': torque_mean,
            'rise_time': rise_time,
            'overshoot_pct': overshoot_pct,
            'max_iq': max_iq,
            'max_uq': max_uq,
            'expected_load': expected_load,
        },
        'data': {
            'times': times,
            'speed_rpm': speed_rpm_log,
            'speed_cmd': speed_cmd_log,
            'torque': torque_log,
            'id': id_log,
            'iq': iq_log,
            'ud': ud_log,
            'uq': uq_log,
        }
    }


def plot_experiment_result(result: dict, save_fig: bool = True):
    """Plot and save experiment results."""
    exp_name = result['name']
    exp_desc = result['desc']
    data = result['data']

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(data['times'], data['speed_cmd'], 'r--', linewidth=2, label='Command')
    axes[0].plot(data['times'], data['speed_rpm'], 'b-', linewidth=1.5, label='Actual')
    axes[0].set_ylabel('Speed [rpm]')
    axes[0].set_title(f'{exp_name}: {exp_desc}')
    axes[0].legend(loc='lower right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim([0, SIMULATION_TIME])

    axes[1].plot(data['times'], data['torque'], 'g-', linewidth=1.5, label='Electromagnetic')
    axes[1].axhline(y=result['metrics']['expected_load'], color='orange',
                    linestyle='--', linewidth=1.5, label='Expected Load')
    axes[1].set_ylabel('Torque [Nm]')
    axes[1].set_title('Electromagnetic Torque vs Pump Load')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(data['times'], data['id'], 'b-', linewidth=1.5, label='iD')
    axes[2].plot(data['times'], data['iq'], 'r-', linewidth=1.5, label='iQ')
    axes[2].set_ylabel('Current [A]')
    axes[2].set_title('DQ Currents')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(data['times'], data['ud'], 'b-', linewidth=1.5, label='uD')
    axes[3].plot(data['times'], data['uq'], 'r-', linewidth=1.5, label='uQ')
    axes[3].axhline(y=VDC/np.sqrt(3), color='k', linestyle='--',
                    linewidth=1, label=f'Vdc/√3={VDC/np.sqrt(3):.1f}V')
    axes[3].axhline(y=-VDC/np.sqrt(3), color='k', linestyle='--', linewidth=1)
    axes[3].set_ylabel('Voltage [V]')
    axes[3].set_xlabel('Time [s]')
    axes[3].set_title('DQ Voltages (vs Linear Modulation Limit)')
    axes[3].legend(loc='upper right')
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()

    if save_fig:
        output_file = f'{exp_name}_result.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"  Saved: {output_file}")

    plt.close()


def plot_comparison(results: list, save_fig: bool = True):
    """Plot comparison of all experiments."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'orange']

    for i, result in enumerate(results):
        color = colors[i % len(colors)]
        data = result['data']
        label = f"{result['name']}: err={result['metrics']['steady_state_error_pct']:.2f}%"

        axes[0].plot(data['times'], data['speed_rpm'], color=color,
                     linewidth=1.5, label=label, alpha=0.8)

        axes[1].plot(data['times'], data['iq'], color=color,
                     linewidth=1.5, label=result['name'], alpha=0.8)

        axes[2].plot(data['times'], data['uq'], color=color,
                     linewidth=1.5, label=result['name'], alpha=0.8)

    axes[0].plot([0, SIMULATION_TIME], [SPEED_COMMAND_RPM, SPEED_COMMAND_RPM],
                 'k--', linewidth=2, label=f'Target {SPEED_COMMAND_RPM} rpm')
    axes[0].set_ylabel('Speed [rpm]')
    axes[0].set_title('Speed Tracking Comparison')
    axes[0].legend(loc='lower right', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim([0, SIMULATION_TIME])

    axes[1].set_ylabel('iQ Current [A]')
    axes[1].set_title('Q-axis Current Comparison')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].axhline(y=VDC/np.sqrt(3), color='k', linestyle='--', linewidth=1)
    axes[2].axhline(y=-VDC/np.sqrt(3), color='k', linestyle='--', linewidth=1)
    axes[2].set_ylabel('uQ Voltage [V]')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_title(f'Q-axis Voltage (limit: Vdc/√3={VDC/np.sqrt(3):.1f}V)')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()

    if save_fig:
        plt.savefig('waterpump_tuning_comparison.png', dpi=150, bbox_inches='tight')
        print(f"  Saved: waterpump_tuning_comparison.png")

    plt.close()


def generate_tuning_report(results: list, motor_params: MachineParams) -> str:
    """Generate tuning report."""
    report = []
    report.append("# Automotive Water Pump FOC Parameter Tuning Report")
    report.append("")
    report.append("## Motor Specifications")
    report.append("")
    report.append(f"| Parameter | Value |")
    report.append(f"|-----------|-------|")
    report.append(f"| Application | Automotive electronic water pump |")
    report.append(f"| Power | 100-150W |")
    report.append(f"| DC Bus Voltage | {VDC} V (12V system) |")
    report.append(f"| Max Phase Voltage | {VDC/np.sqrt(3):.2f} V (linear modulation) |")
    report.append(f"| Pole pairs | {motor_params.npp} |")
    report.append(f"| Phase Resistance | {motor_params.R:.3f} Ohm |")
    report.append(f"| Ld | {motor_params.Ld*1000:.2f} mH |")
    report.append(f"| Lq | {motor_params.Lq*1000:.2f} mH |")
    report.append(f"| PM Flux (KE) | {motor_params.KE:.4f} Wb |")
    report.append(f"| Inertia | {motor_params.Js:.5f} kg.m^2 |")
    report.append(f"| Target Speed | {SPEED_COMMAND_RPM} rpm |")
    report.append("")

    report.append("## Experiment Results Summary")
    report.append("")
    report.append("| Exp | Description | Steady Error | Overshoot | Rise Time | Speed Std | max_iq | max_uq |")
    report.append("|-----|-------------|--------------|----------|-----------|-----------|--------|--------|")

    for r in results:
        m = r['metrics']
        report.append(f"| {r['name']} | {r['desc']} | {m['steady_state_error_pct']:.2f}% | "
                      f"{m['overshoot_pct']:.1f}% | {m['rise_time']:.3f}s | "
                      f"{m['speed_std']:.1f}rpm | {m['max_iq']:.1f}A | {m['max_uq']:.1f}V |")

    report.append("")

    # Analysis for each experiment
    report.append("## Detailed Analysis")
    report.append("")

    for r in results:
        report.append(f"### {r['name']}: {r['desc']}")
        report.append("")
        p = r['params']
        report.append("**Parameters:**")
        report.append(f"- speed_Kp: {p['speed_Kp']:.4f}, speed_Ki: {p['speed_Ki']:.2f}")
        report.append(f"- current_Kp: {p['current_Kp']:.2f}, current_Ki: {p['current_Ki']:.1f}")
        report.append(f"- speed_limit (iq cmd): {p['speed_limit']:.1f} A")
        report.append(f"- voltage_limit: {p['current_limit']:.1f} V")
        report.append("")

        m = r['metrics']
        report.append("**Results:**")
        report.append(f"- Final speed: {m['final_speed']:.1f} rpm")
        report.append(f"- Steady state error: {m['steady_state_error_pct']:.2f}%")
        report.append(f"- Speed oscillation: {m['speed_std']:.2f} rpm")
        report.append(f"- Rise time: {m['rise_time']:.3f} s")
        report.append(f"- Overshoot: {m['overshoot_pct']:.1f}%")
        report.append(f"- Max current: {m['max_iq']:.1f} A")
        report.append(f"- Max voltage: {m['max_uq']:.1f} V (linear limit: {VDC/np.sqrt(3):.1f} V)")
        report.append("")

        # Analysis
        report.append("**Analysis:**")

        # Voltage saturation check
        if m['max_uq'] > VDC/np.sqrt(3) * 0.95:
            report.append(f"- **Voltage saturated** ({m['max_uq']:.1f}V > {VDC/np.sqrt(3):.1f}V limit)")
        else:
            report.append(f"- Voltage within linear range ({m['max_uq']:.1f}V)")

        # Steady state error
        if m['steady_state_error_pct'] > 5:
            report.append(f"- Large steady error ({m['steady_state_error_pct']:.2f}%)")
        elif m['steady_state_error_pct'] > 1:
            report.append(f"- Moderate steady error ({m['steady_state_error_pct']:.2f}%)")
        else:
            report.append(f"- Good steady error ({m['steady_state_error_pct']:.2f}%)")

        # Oscillation
        if m['speed_std'] > 50:
            report.append(f"- **High oscillation** (std={m['speed_std']:.1f}rpm)")
        elif m['speed_std'] > 10:
            report.append(f"- Moderate oscillation (std={m['speed_std']:.1f}rpm)")
        else:
            report.append(f"- Low oscillation (std={m['speed_std']:.1f}rpm)")

        # Overshoot
        if m['overshoot_pct'] > 20:
            report.append(f"- High overshoot ({m['overshoot_pct']:.1f}%)")
        elif m['overshoot_pct'] > 5:
            report.append(f"- Moderate overshoot ({m['overshoot_pct']:.1f}%)")
        else:
            report.append(f"- Low overshoot ({m['overshoot_pct']:.1f}%)")

        report.append("")

    # Best result
    report.append("## Conclusion")
    report.append("")
    best = min(results, key=lambda x: x['metrics']['steady_state_error_pct'])
    report.append(f"**Best configuration: {best['name']}**")
    report.append("")
    report.append("```python")
    report.append(f"ctrl_params = ControllerParams(")
    report.append(f"    CL_TS=1e-4,")
    report.append(f"    VL_TS=1e-3,")
    report.append(f"    speed_Kp={best['params']['speed_Kp']:.4f},")
    report.append(f"    speed_Ki={best['params']['speed_Ki']:.2f},")
    report.append(f"    speed_limit={best['params']['speed_limit']:.1f},")
    report.append(f"    current_Kp={best['params']['current_Kp']:.2f},")
    report.append(f"    current_Ki={best['params']['current_Ki']:.1f},")
    report.append(f"    current_limit={best['params']['current_limit']:.1f},")
    report.append(f")")
    report.append("```")
    report.append("")
    report.append(f"Steady state error: **{best['metrics']['steady_state_error_pct']:.2f}%**")
    report.append("")

    return "\n".join(report)


def run_all_experiments():
    """Run all water pump tuning experiments."""
    print("\n" + "=" * 60)
    print("Automotive Water Pump FOC Parameter Tuning")
    print("=" * 60)
    print(f"\nMotor: 12V 100W PMSM")
    print(f"DC Bus: {VDC} V")
    print(f"Target Speed: {SPEED_COMMAND_RPM} rpm")

    results = []

    for exp in EXPERIMENTS:
        result = run_single_experiment(exp, WATERPUMP_MOTOR_PARAMS)
        plot_experiment_result(result)
        results.append(result)

    # Comparison plot
    plot_comparison(results)

    # Generate report
    report = generate_tuning_report(results, WATERPUMP_MOTOR_PARAMS)

    with open('waterpump_tuning_report.md', 'w', encoding='utf-8') as f:
        f.write(report)
    print(f"\nReport saved: waterpump_tuning_report.md")

    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(report)

    return results


if __name__ == '__main__':
    results = run_all_experiments()