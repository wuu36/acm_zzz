"""FOC closed-loop speed control simulation.

This example demonstrates a complete FOC (Field Oriented Control) speed
closed-loop simulation integrating Phase 4-15 components:
    - MachineParams, MachineState, ACMachine (Phase 4, 5, 7)
    - dynamics_machine (Phase 8)
    - rk4_integrate (Phase 9)
    - PIRegulator (Phase 11)
    - ControllerParams, FOCConfig (Phase 13)
    - MotorController (Phase 15)

The simulation shows a PMSM motor with FOC speed control following
a speed command from standstill.

Run:
    uv run python examples/foc_simulation.py
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
# Parameter Tuning Experiments Configuration
# ============================================================

# Target speed for all experiments
SPEED_COMMAND_RPM = 200.0
SIMULATION_TIME = 2.0

# Experiment configurations: each dict defines controller parameters
EXPERIMENTS = [
    {
        'name': 'exp0_baseline',
        'desc': '原始保守参数',
        'params': {
            'speed_Kp': 0.01,
            'speed_Ki': 2.0,
            'speed_limit': 3.0,
            'current_Kp': 2.0,
            'current_Ki': 200.0,
            'current_limit': 50.0,
        }
    },
    {
        'name': 'exp1_raise_voltage_limit',
        'desc': '提高电压限幅避免饱和',
        'params': {
            'speed_Kp': 0.01,
            'speed_Ki': 2.0,
            'speed_limit': 3.0,
            'current_Kp': 2.0,
            'current_Ki': 200.0,
            'current_limit': 150.0,  # 提高50->150V
        }
    },
    {
        'name': 'exp2_raise_current_limit',
        'desc': '提高电流命令限幅',
        'params': {
            'speed_Kp': 0.01,
            'speed_Ki': 2.0,
            'speed_limit': 10.0,     # 提高3->10A
            'current_Kp': 2.0,
            'current_Ki': 200.0,
            'current_limit': 150.0,
        }
    },
    {
        'name': 'exp3_increase_speed_pi',
        'desc': '提高速度环PI增益',
        'params': {
            'speed_Kp': 0.05,        # 提高0.01->0.05
            'speed_Ki': 20.0,        # 提高2->20
            'speed_limit': 10.0,
            'current_Kp': 2.0,
            'current_Ki': 200.0,
            'current_limit': 150.0,
        }
    },
    {
        'name': 'exp4_increase_current_pi',
        'desc': '同时提高电流环PI增益',
        'params': {
            'speed_Kp': 0.05,
            'speed_Ki': 20.0,
            'speed_limit': 10.0,
            'current_Kp': 10.0,      # 提高2->10
            'current_Ki': 1000.0,    # 提高200->1000
            'current_limit': 150.0,
        }
    },
    {
        'name': 'exp5_auto_design',
        'desc': '使用auto_design_gains自动设计',
        'params': {
            'speed_Kp': 0.038,       # 自动设计初始值
            'speed_Ki': 30.5,
            'speed_limit': 10.0,
            'current_Kp': 6.4,
            'current_Ki': 1523.2,
            'current_limit': 300.0,
        },
        'use_auto_design': True,
        'current_bw_hz': 1000.0,
        'speed_bw_hz': 50.0,
    },
]


def run_single_experiment(exp_config: dict) -> dict:
    """Run a single FOC simulation experiment with given parameters.

    Returns:
        dict: Experiment results including metrics and data logs
    """
    exp_name = exp_config['name']
    exp_desc = exp_config['desc']
    params_dict = exp_config['params']

    print("\n" + "=" * 60)
    print(f"实验: {exp_name} - {exp_desc}")
    print("=" * 60)

    # Motor Setup (same for all experiments)
    params = MachineParams(
        npp=4,
        R=1.1,
        Ld=5e-3,
        Lq=6e-3,
        KE=0.095,
        Js=0.0006168,
    )

    state = MachineState()
    machine = ACMachine(params=params, state=state)

    # Controller Setup with experiment parameters
    ctrl_params = ControllerParams(
        CL_TS=1e-4,
        VL_TS=5e-4,
        VL_ceiling=5,
        speed_Kp=params_dict['speed_Kp'],
        speed_Ki=params_dict['speed_Ki'],
        speed_limit=params_dict['speed_limit'],
        current_Kp=params_dict['current_Kp'],
        current_Ki=params_dict['current_Ki'],
        current_limit=params_dict['current_limit'],
    )

    foc_config = FOCConfig(
        control_mode='speed',
        id_command_strategy='mtpa',
    )

    controller = MotorController(
        motor_params=params,
        controller_params=ctrl_params,
        foc_config=foc_config,
    )

    # Apply auto_design if specified
    if exp_config.get('use_auto_design', False):
        controller.auto_design_gains(
            current_bw_hz=exp_config.get('current_bw_hz', 1000.0),
            speed_bw_hz=exp_config.get('speed_bw_hz', 50.0)
        )
        # Update params_dict with auto-designed values for logging
        params_dict['speed_Kp'] = controller.speed_reg.Kp
        params_dict['speed_Ki'] = controller.speed_reg.Ki
        params_dict['current_Kp'] = controller.id_reg.Kp
        params_dict['current_Ki'] = controller.id_reg.Ki

    print(f"\n控制器参数:")
    print(f"  速度环PI: Kp={params_dict['speed_Kp']:.4f}, Ki={params_dict['speed_Ki']:.2f}")
    print(f"  电流环PI: Kp={params_dict['current_Kp']:.2f}, Ki={params_dict['current_Ki']:.2f}")
    print(f"  电流限幅: {params_dict['speed_limit']:.1f} A")
    print(f"  电压限幅: {params_dict['current_limit']:.1f} V")

    # Simulation Setup
    x = np.zeros(5, dtype=np.float64)
    x[2] = params.KE

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

    T_load = 0.0

    # Run Closed-Loop Simulation
    print(f"\n运行仿真...")
    t = 0.0
    controller.set_speed_command(SPEED_COMMAND_RPM)

    for step in range(n_steps):
        times[step] = t

        theta_mech = x[0]
        omega_mech = x[1]
        KA = x[2]
        iD = x[3]
        iQ = x[4]

        theta_elec = theta_mech * params.npp
        omega_elec = omega_mech * params.npp

        controller.execute(
            omega_r_elec=omega_elec,
            id_=iD,
            iq=iQ,
            theta_d=theta_elec,
            Vdc=300.0
        )

        ud, uq = controller.get_voltage_commands_dq()

        machine.state.udq[0] = ud
        machine.state.udq[1] = uq
        machine.state.Tload = T_load

        x = rk4_step(dynamics_machine, t, x, dt, machine)

        speed_rpm = omega_mech * 60.0 / (2.0 * np.pi)
        speed_rpm_log[step] = speed_rpm
        speed_cmd_log[step] = SPEED_COMMAND_RPM

        Tem = 1.5 * params.npp * KA * iQ
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

    # Steady state analysis (last 20% of simulation)
    n_steady = int(n_steps * 0.2)
    steady_speed = speed_rpm_log[-n_steady:]
    steady_torque = torque_log[-n_steady:]
    steady_iq = iq_log[-n_steady:]

    speed_std = np.std(steady_speed)
    torque_mean = np.mean(steady_torque)

    max_iq = np.max(np.abs(iq_log))
    max_uq = np.max(np.abs(uq_log))

    # Rise time (10% to 90% of target)
    target_speed = SPEED_COMMAND_RPM
    idx_10 = np.argmax(speed_rpm_log >= 0.1 * target_speed)
    idx_90 = np.argmax(speed_rpm_log >= 0.9 * target_speed)
    rise_time = times[idx_90] - times[idx_10] if idx_90 > idx_10 else 0.0

    # Overshoot
    max_speed = np.max(speed_rpm_log)
    overshoot_pct = (max_speed - target_speed) / target_speed * 100 if max_speed > target_speed else 0.0

    print(f"\n结果:")
    print(f"  最终速度: {final_speed:.2f} rpm")
    print(f"  稳态误差: {steady_state_error:.2f} rpm ({steady_state_error_pct:.1f}%)")
    print(f"  速度波动(std): {speed_std:.2f} rpm")
    print(f"  稳态转矩: {torque_mean:.3f} Nm")
    print(f"  上升时间: {rise_time:.3f} s")
    print(f"  超调量: {overshoot_pct:.1f}%")
    print(f"  最大电流 iq: {max_iq:.2f} A")
    print(f"  最大电压 uq: {max_uq:.2f} V")

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

    axes[1].plot(data['times'], data['torque'], 'g-', linewidth=1.5)
    axes[1].set_ylabel('Torque [Nm]')
    axes[1].set_title('Electromagnetic Torque')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(data['times'], data['id'], 'b-', linewidth=1.5, label='iD')
    axes[2].plot(data['times'], data['iq'], 'r-', linewidth=1.5, label='iQ')
    axes[2].set_ylabel('Current [A]')
    axes[2].set_title('DQ Currents')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(data['times'], data['ud'], 'b-', linewidth=1.5, label='uD')
    axes[3].plot(data['times'], data['uq'], 'r-', linewidth=1.5, label='uQ')
    axes[3].set_ylabel('Voltage [V]')
    axes[3].set_xlabel('Time [s]')
    axes[3].set_title('DQ Voltages')
    axes[3].legend(loc='upper right')
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()

    if save_fig:
        output_file = f'{exp_name}_result.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"  保存图片: {output_file}")

    plt.close()


def plot_comparison(results: list, save_fig: bool = True):
    """Plot comparison of all experiments."""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

    for i, result in enumerate(results):
        color = colors[i % len(colors)]
        data = result['data']
        label = f"{result['name']}: err={result['metrics']['steady_state_error_pct']:.1f}%"

        axes[0].plot(data['times'], data['speed_rpm'], color=color,
                     linewidth=1.5, label=label, alpha=0.7)

        axes[1].plot(data['times'], data['iq'], color=color,
                     linewidth=1.5, label=result['name'], alpha=0.7)

    # Speed command reference
    axes[0].plot([0, SIMULATION_TIME], [SPEED_COMMAND_RPM, SPEED_COMMAND_RPM],
                 'k--', linewidth=2, label='Target 200 rpm')

    axes[0].set_ylabel('Speed [rpm]')
    axes[0].set_title('速度跟踪对比')
    axes[0].legend(loc='lower right', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim([0, SIMULATION_TIME])

    axes[1].set_ylabel('iQ Current [A]')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_title('q轴电流对比')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()

    if save_fig:
        plt.savefig('foc_tuning_comparison.png', dpi=150, bbox_inches='tight')
        print(f"  保存对比图: foc_tuning_comparison.png")

    plt.close()


def generate_tuning_report(results: list) -> str:
    """Generate tuning report markdown."""
    report = []
    report.append("# FOC仿真参数调优报告")
    report.append("")
    report.append(f"**目标速度**: {SPEED_COMMAND_RPM} rpm")
    report.append(f"**仿真时间**: {SIMULATION_TIME} s")
    report.append("")

    # Summary table
    report.append("## 实验结果汇总")
    report.append("")
    report.append("| 实验 | 描述 | 稳态误差 | 超调量 | 上升时间 | 速度波动 | max_iq | max_uq |")
    report.append("|------|------|----------|--------|----------|----------|--------|--------|")

    for r in results:
        m = r['metrics']
        report.append(f"| {r['name']} | {r['desc']} | {m['steady_state_error_pct']:.1f}% | "
                      f"{m['overshoot_pct']:.1f}% | {m['rise_time']:.3f}s | "
                      f"{m['speed_std']:.2f}rpm | {m['max_iq']:.2f}A | {m['max_uq']:.2f}V |")

    report.append("")

    # Detailed analysis for each experiment
    report.append("## 详细分析")
    report.append("")

    for r in results:
        report.append(f"### {r['name']}: {r['desc']}")
        report.append("")
        report.append("**参数设置:**")
        report.append("")
        p = r['params']
        report.append(f"- speed_Kp: {p['speed_Kp']}, speed_Ki: {p['speed_Ki']}")
        report.append(f"- current_Kp: {p['current_Kp']}, current_Ki: {p['current_Ki']}")
        report.append(f"- speed_limit: {p['speed_limit']} A, current_limit: {p['current_limit']} V")
        report.append("")
        m = r['metrics']
        report.append("**结果指标:**")
        report.append(f"- 最终速度: {m['final_speed']:.2f} rpm")
        report.append(f"- 稳态误差: {m['steady_state_error']:.2f} rpm ({m['steady_state_error_pct']:.1f}%)")
        report.append(f"- 速度波动(标准差): {m['speed_std']:.2f} rpm")
        report.append(f"- 稳态转矩: {m['torque_mean']:.3f} Nm")
        report.append(f"- 上升时间: {m['rise_time']:.3f} s")
        report.append(f"- 超调量: {m['overshoot_pct']:.1f}%")
        report.append("")
        report.append("**分析:**")
        report.append(_analyze_experiment(r))
        report.append("")

    # Conclusion
    report.append("## 结论")
    report.append("")
    best = min(results, key=lambda x: x['metrics']['steady_state_error_pct'])
    report.append(f"最佳参数配置: **{best['name']}** ({best['desc']})")
    report.append(f"- 稳态误差: {best['metrics']['steady_state_error_pct']:.1f}%")
    report.append("")

    return "\n".join(report)


def _analyze_experiment(r: dict) -> str:
    """Generate analysis text for an experiment."""
    m = r['metrics']
    p = r['params']

    analyses = []

    # Check voltage saturation
    if m['max_uq'] >= p['current_limit'] * 0.95:
        analyses.append(f"- 电压饱和(max_uq={m['max_uq']:.1f}V接近限幅{p['current_limit']:.1f}V)，影响电流响应")
    else:
        analyses.append(f"- 电压未饱和(max_uq={m['max_uq']:.1f}V < 限幅{p['current_limit']:.1f}V)")

    # Check steady state error
    if m['steady_state_error_pct'] > 5:
        analyses.append(f"- 稳态误差较大({m['steady_state_error_pct']:.1f}%)，需要提高积分增益")
    elif m['steady_state_error_pct'] > 1:
        analyses.append(f"- 稳态误差中等({m['steady_state_error_pct']:.1f}%)")
    else:
        analyses.append(f"- 稳态误差小({m['steady_state_error_pct']:.1f}%)，控制效果好")

    # Check speed oscillation
    if m['speed_std'] > 5:
        analyses.append(f"- 速度波动大(std={m['speed_std']:.2f}rpm)，存在振荡")
    elif m['speed_std'] > 1:
        analyses.append(f"- 速度波动中等(std={m['speed_std']:.2f}rpm)")
    else:
        analyses.append(f"- 速度波动小(std={m['speed_std']:.2f}rpm)，稳定性好")

    # Check overshoot
    if m['overshoot_pct'] > 10:
        analyses.append(f"- 超调量大({m['overshoot_pct']:.1f}%)，可降低Kp或增加Ki")
    elif m['overshoot_pct'] > 0:
        analyses.append(f"- 超调量适中({m['overshoot_pct']:.1f}%)")
    else:
        analyses.append(f"- 无超调")

    # Check steady torque (should be near zero with no load)
    if abs(m['torque_mean']) > 0.5:
        analyses.append(f"- 稳态转矩异常({m['torque_mean']:.3f}Nm)，控制器未正确收敛")
    else:
        analyses.append(f"- 稳态转矩正常({m['torque_mean']:.3f}Nm)")

    return "\n".join(analyses)


def run_all_experiments():
    """Run all experiments and generate report."""
    print("\n" + "=" * 60)
    print("FOC参数调优实验")
    print("=" * 60)

    results = []

    for exp in EXPERIMENTS:
        result = run_single_experiment(exp)
        plot_experiment_result(result)
        results.append(result)

    # Plot comparison
    plot_comparison(results)

    # Generate report
    report = generate_tuning_report(results)

    # Save report
    with open('foc_tuning_report.md', 'w', encoding='utf-8') as f:
        f.write(report)
    print(f"\n保存调优报告: foc_tuning_report.md")

    # Print summary
    print("\n" + "=" * 60)
    print("调优结果汇总")
    print("=" * 60)
    print(report)

    return results


if __name__ == '__main__':
    results = run_all_experiments()