

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from collections.abc import Sequence
from acmzzz.core import (
    MachineParams,
    MachineState,
    ACMachine,
    dynamics_machine_current_source,
    rk4_integrate,
)


def run_current_source_simulation():
    print("=" * 60)
    print("acmzzz - basic motor simulation (current source mode)")
    print("=" * 60)

    # ========================================
    # 1. create motor with parameters
    # ========================================
    # Default PMSM parameters (can be customized)
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
    
    print(f"\nMotor Parameters:")
    print(f"  Pole pairs (npp): {params.npp}")
    print(f"  Resistance (R): {params.R} Ohm")
    print(f"  Ld: {params.Ld*1000} mH")
    print(f"  Lq: {params.Lq*1000} mH")
    print(f"  PM flux (KE): {params.KE} Wb")
    print(f"  Inertia (js): {params.Js} kg.m^2")
    print(f"  Motor type: {'PMSM' if params.is_pmsm else 'Indction Motor'}")

    # ========================================
    # 2. simulation setup
    # ========================================
    # Initial state: motor at standstill
    x0 = np.zeros(5)
    x0[2] = params.KE   # KA = KE for PMSM

    # Current source command
    iD_cmd = 0.0
    iQ_cmd = 10.0

    # Time settings
    dt = 1e-4               # 0.1ms simulation step
    simulation_time = 1.0   # 1 second total
    n_steps = int(simulation_time / dt)
    
    print(f"\nSimulation Settings:")
    print(f"  Time step: {dt*1000} ms")
    print(f"  Total time: {simulation_time} s")
    print(f"  Number of steps: {n_steps}")
    print(f"  iQ command: {iQ_cmd} A")

    # ========================================
    # 3. calculate expected torque
    # ========================================
    # Tem = 1.5 * npp * KA * iQ
    excepted_torque = 1.5 * params.npp * params.KE * iQ_cmd
    print(f"\nExpected electromagnetic torque: {excepted_torque:.3f} Nm")

    # ========================================
    # 4. run RK4 integration
    # ========================================
    print(f"\nRunning RK4 integration...")
    
    # define dynamics function wrapper for current source mode
    def dynamics_wrapper(t, x, machine):
        return dynamics_machine_current_source(t, x, machine, iD_cmd, iQ_cmd)
    
    # run simulation
    times, states = rk4_integrate(
        dynamics_wrapper,
        t0=0.0,
        x0=x0,
        dt=dt,
        n_steps=n_steps,
        machine=machine
    )

    print(f"Simulation complete!")

    # ========================================
    # 5. analyze results
    # ========================================
    # extract state variables
    theta_mech = states[:, 0]   # mechanical angle [rad]
    omega_mech = states[:, 1]   # mechanical speed [rad/s]
    KA = states[:, 2]           # active flux [Wb]
    iD = states[:, 3]           # D-axis current [A]
    iQ = states[:, 4]           # Q-axis current [A]

    # convert to useful units
    speed_rpm = omega_mech * 60 / (2 * np.pi)   # [rpm]
    theta_deg = theta_mech * 180 / np.pi        # [deg]
    theta_deg_wrapped = theta_deg % 360

    # calculate actual torque over time
    # for current source mode, use commanded iQ (state iQ is not updated)
    # Tem = 1.5 * npp * KA * iQ
    # for PMSM: KA = KE (approximately, since iD=0)
    KA_actual = (params.Ld - params.Lq) * iD_cmd + params.KE # active flux
    torque = 1.5 * params.npp * KA_actual * iQ_cmd  # constant torque in current source mode
    
    print(f"\nResults Summary:")
    print(f"  Initial speed: {speed_rpm[0]:.2f} rpm")
    print(f"  Final speed: {speed_rpm[-1]:.2f} rpm")
    print(f"  Final position: {theta_deg[-1]:.2f} deg")
    print(f"  Torque (constant): {torque:.3f} Nm")

    # ========================================
    # 6. plot results
    # ========================================
    print(f"\nGenerating plots...")
    
    # create figure with subplots
    # fig,axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    # fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes: Sequence[Axes]    # 手动标注


    # plot 1: speed [rpm]
    axes[0].plot(times, speed_rpm, 'b-', linewidth=1.5)
    axes[0].set_ylabel('Speed [rpm]')
    axes[0].set_title('Motor Speed Response')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim(0, simulation_time)
    
    # plot 2: postition [deg]
    axes[1].plot(times, theta_deg_wrapped, 'g-', linewidth=1.5)
    axes[1].set_ylabel('Postition [deg]')
    axes[1].set_title('Rotor Position')
    axes[1].grid(True, alpha=0.3)

    # plot 3: torque [Nm] (constant in current source mode)
    torque_array = np.full_like(times, torque)
    axes[2].plot(times, torque_array, 'r-', linewidth=1.5)
    axes[2].set_ylabel('Torque [Nm]')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_title('Electromagnetic Torque')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()

    # save figure

    # show figure (if interactive)
    plt.show()

    return times, states

def run_volatage_source_simulation():
    print("\n" + "=" * 60)
    print("acmzzz - motor simulation (voltage source mode)")
    print("=" * 60)
    
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
    
    # initial state
    x0 = np.zeros(5)
    x0[2] = params.KE
    x0[4] = 0.0     # initial iQ
    
    # set voltage command (simplified - constant uQ)
    uD_cmd = 0.0
    uQ_cmd = 50.0

    machine.state.udq[0] = uD_cmd
    machine.state.udq[1] = uQ_cmd

    dt = 1e-4
    simulation_time = 0.5
    n_steps = int(simulation_time / dt)

    print(f"voltage commands: uD={uD_cmd}V, uQ={uQ_cmd}V")
    
    from acmzzz.core import dynamics_machine

    times, states = rk4_integrate(
        dynamics_machine,
        t0=0.0,
        x0=x0,
        dt=dt,
        n_steps=n_steps,
        machine=machine,
    )

    omega_mech = states[:, 1]
    speed_rpm = omega_mech * 60 / (2 * np.pi)
    
    print(f"final speed: {speed_rpm[-1]:.2f} rpm")
    
    return times, states

if __name__ == '__main__':
    # run basi current source simulation
    times, states = run_current_source_simulation()
    
    # optionally run voltage source simulation
    # times, states = run_volatage_source_simulation()