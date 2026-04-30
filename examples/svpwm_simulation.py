import numpy as np
import matplotlib
# matplotlib.use('Agg')   # non-interactive backend for saving figures
import matplotlib.pyplot as plt
from acmzzz.inverter import SVPWM, svpwm_duty_cycle, determine_sector
from acmzzz.core.transforms import inverse_park

def run_svpwm_rotation_simulation():
    print("=" * 60)
    print("acmzzz - svpwm waveform simulation")
    print("=" * 60)

    # ========================================
    # 1. setup
    # ========================================
    Vdc = 300.0         # DC bus voltate [V]
    magnitude = 100.0   # voltage magnitude [V]
    
    # svpwm generator
    svpwm = SVPWM(Vdc=Vdc, max_duty=0.95, min_duty=0.05)
    
    # angle range: 0 to 360 degrees
    n_points = 360
    angles_deg = np.linspace(0, 360, n_points)
    angles_rad = np.deg2rad(angles_deg)

    # data arrays
    Ualpha = magnitude * np.cos(angles_rad)
    Ubeta = magnitude * np.sin(angles_rad)
    
    sectors = np.zeros(n_points, dtype=int)
    Ta = np.zeros(n_points)
    Tb = np.zeros(n_points)
    Tc = np.zeros(n_points)

    print(f"\nSVPWM Parameters:")
    print(f"  DC bus voltage: {Vdc} V")
    print(f"  Voltage magnitude: {magnitude} V")
    print(f"  Max duty cycle: {svpwm.max_duty}")
    print(f"  Min duty cycle: {svpwm.min_duty}")

    # ========================================
    # 2. calculate SVPWM for each angle
    # ========================================
    print(f"\nCalculating SVPWM duty cycle...")

    for i, angle in enumerate(angles_rad):
        svpwm.set_voltage(Ualpha[i], Ubeta[i])
        svpwm.calculate()
        
        sectors[i] = svpwm.sector
        Ta[i] = svpwm.Ta
        Tb[i] = svpwm.Tb
        Tc[i] = svpwm.Tc
    
    print(f"Calculation complete!")

    # ========================================
    # 3. analyze sector distribution
    # ========================================
    unique_sectors = np.unique(sectors)
    sector_counts = {s: np.sum(sectors == s) for s in unique_sectors}

    print(f"\nSector Distribution:")
    for s, count in sorted(sector_counts.items()):
        percent = count / n_points * 100
        print(f"  Sector {s}: {count} points ({percent:.1f}%)")
    
    # ========================================
    # 4. calculate line-to-line voltages
    # ========================================
    # Va = Ta * Vdc, Vb = Tb * Vdc, Vc = Tc * Vdc
    Va = Ta * Vdc
    Vb = Tb * Vdc
    Vc = Tc * Vdc

    # line-to-line voltages
    Vab = Va - Vb
    Vbc = Vb - Vc
    Vca = Vc - Va

    # ========================================
    # 5. plot results
    # ========================================
    print(f"\nGenerating plots...")
    
    fig, axes = plt.subplots(4, 1, figsize=(12,10), sharex=True)

    # Plot 1: Duty cycles
    axes[0].plot(angles_deg, Ta, 'r-', linewidth=1.5, label='Ta')
    axes[0].plot(angles_deg, Tb, 'g-', linewidth=1.5, label='Tb')
    axes[0].plot(angles_deg, Tc, 'b-', linewidth=1.5, label='Tc')
    axes[0].set_ylabel('Duty Cycle')
    axes[0].set_title('SVPWM Duty Cycles (Ta, Tb, Tc)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_ylim([0, 1])

    # Plot 2: Sector indication
    axes[1].fill_between(angles_deg, sectors, alpha=0.5)
    axes[1].plot(angles_deg, sectors, 'k-', linewidth=1)
    axes[1].set_ylabel('Sector')
    axes[1].set_title('SVPWM Sector (0-6)')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_ylim([0, 6.5])

    # Plot 3: Alpha-beta voltages
    axes[2].plot(angles_deg, Ualpha, 'r-', linewidth=1.5, label='Uα')
    axes[2].plot(angles_deg, Ubeta, 'b-', linewidth=1.5, label='Uβ')
    axes[2].set_ylabel('Voltage [V]')
    axes[2].set_title('Input Voltages (Alpha-Beta)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    # Plot 4: Line-to-line voltages
    axes[3].plot(angles_deg, Vab, 'r-', linewidth=1.5, label='Vab')
    axes[3].plot(angles_deg, Vbc, 'g-', linewidth=1.5, label='Vbc')
    axes[3].plot(angles_deg, Vca, 'b-', linewidth=1.5, label='Vca')
    axes[3].set_ylabel('Voltage [V]')
    axes[3].set_xlabel('Angle [degrees]')
    axes[3].set_title('Line-to-Line Voltages')
    axes[3].legend(loc='upper right')
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()

    # save figure
    # output_file = 'svpwm_waveforms.png'
    # plt.savefig(output_file, dpi=150, bbox_inches='tight')
    # print(f"  Saved: {output_file}")

    plt.show()

    return angles_deg, sectors, Ta, Tb, Tc

def run_svpwm_with_dq_input():
    """Run SVPWM simulation with DQ voltage input.

    Demonstrates converting DQ voltages to alpha-beta and then
    generating SVPWM duty cycles.
    """
    print("\n" + "=" * 60)
    print("ACMSimPy - SVPWM with DQ Input Simulation")
    print("=" * 60)

    Vdc = 300.0
    Ud = 50.0  # D-axis voltage [V]
    Uq = 80.0  # Q-axis voltage [V]

    # Time range
    n_points = 500
    time = np.linspace(0, 0.1, n_points)  # 100ms

    # Rotating angle (electrical frequency)
    freq = 50.0  # Hz
    theta = 2.0 * np.pi * freq * time

    # Convert DQ to alpha-beta
    Ualpha = np.zeros(n_points)
    Ubeta = np.zeros(n_points)

    svpwm = SVPWM(Vdc=Vdc)

    Ta = np.zeros(n_points)
    Tb = np.zeros(n_points)
    Tc = np.zeros(n_points)
    sectors = np.zeros(n_points, dtype=int)

    print(f"\nInput Parameters:")
    print(f"  Ud = {Ud} V, Uq = {Uq} V")
    print(f"  Frequency: {freq} Hz")
    print(f"  DC bus: {Vdc} V")

    for i, th in enumerate(theta):
        # Inverse Park transform
        Ualpha[i], Ubeta[i] = inverse_park(Ud, Uq, th)

        # SVPWM calculation
        svpwm.set_voltage(Ualpha[i], Ubeta[i])
        svpwm.calculate()

        Ta[i] = svpwm.Ta
        Tb[i] = svpwm.Tb
        Tc[i] = svpwm.Tc
        sectors[i] = svpwm.sector

    print(f"\nGenerating plots...")

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    # Plot 1: Electrical angle
    axes[0].plot(time, theta, 'k-', linewidth=1.5)
    axes[0].set_ylabel('θ [rad]')
    axes[0].set_title('Electrical Angle')
    axes[0].grid(True, alpha=0.3)

    # Plot 2: Alpha-beta voltages
    axes[1].plot(time, Ualpha, 'r-', linewidth=1.5, label='Uα')
    axes[1].plot(time, Ubeta, 'b-', linewidth=1.5, label='Uβ')
    axes[1].set_ylabel('Voltage [V]')
    axes[1].set_title('Alpha-Beta Voltages (from DQ)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    # Plot 3: Duty cycles
    axes[2].plot(time, Ta, 'r-', linewidth=1.5, label='Ta')
    axes[2].plot(time, Tb, 'g-', linewidth=1.5, label='Tb')
    axes[2].plot(time, Tc, 'b-', linewidth=1.5, label='Tc')
    axes[2].set_ylabel('Duty Cycle')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_title('SVPWM Duty Cycles')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim([0, 1])

    plt.tight_layout()
    plt.savefig('svpwm_dq_input.png', dpi=150)
    print("  Saved: svpwm_dq_input.png")
    plt.show()

    return time, Ta, Tb, Tc

def run_svpwm_sector_boundary_analysis():
    """Analyze SVPWM sector boundaries.

    Shows exact angles where sector transitions occur.
    """
    print("\n" + "=" * 60)
    print("ACMSimPy - SVPWM Sector Boundary Analysis")
    print("=" * 60)

    Vdc = 300.0
    magnitude = 100.0

    # Fine angle resolution
    angles_deg = np.linspace(0, 360, 3600)
    angles_rad = np.deg2rad(angles_deg)

    Ualpha = magnitude * np.cos(angles_rad)
    Ubeta = magnitude * np.sin(angles_rad)

    sectors = np.zeros(len(angles_deg), dtype=int)

    for i in range(len(angles_deg)):
        sectors[i] = determine_sector(Ualpha[i], Ubeta[i], Vdc)

    # Find sector transitions
    transitions = []
    prev_sector = sectors[0]
    for i, s in enumerate(sectors):
        if s != prev_sector:
            transitions.append((angles_deg[i], prev_sector, s))
            prev_sector = s

    print(f"\nSector Transition Points:")
    for angle, from_s, to_s in transitions:
        print(f"  {angle:.2f}°: Sector {from_s} → {to_s}")

    # Plot sector vs angle
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(angles_deg, sectors, 'b-', linewidth=1)
    ax.set_xlabel('Angle [degrees]')
    ax.set_ylabel('Sector')
    ax.set_title('SVPWM Sector vs Voltage Angle')
    ax.grid(True, alpha=0.3)
    # ax.set_xlim([0, 360])
    ax.set_xlim(0, 360)
    # ax.set_ylim([0, 6.5])
    ax.set_ylim(0, 6.5)

    # Mark transitions
    for angle, from_s, to_s in transitions:
        ax.axvline(x=angle, color='r', linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.savefig('svpwm_sector_analysis.png', dpi=150)
    print("  Saved: svpwm_sector_analysis.png")
    plt.show()

    return transitions

def run_svpwm_voltage_limit_analysis():
    """Analyze SVPWM behavior at voltage limits.

    Shows how duty cycles saturate at high voltages.
    """
    print("\n" + "=" * 60)
    print("ACMSimPy - SVPWM Voltage Limit Analysis")
    print("=" * 60)

    Vdc = 300.0

    # Different voltage magnitudes
    magnitudes = [50, 100, 150, 200, 250, 300]

    # Single angle (30 degrees)
    angle = np.radians(30)

    results = []

    print(f"\nVoltage Limit Analysis at θ = 30°:")
    print(f"  Vdc = {Vdc} V")
    print(f"\n  Magnitude [V] | Ta     | Tb     | Tc     | Saturated?")
    print("-" * 55)

    for mag in magnitudes:
        Ualpha = mag * np.cos(angle)
        Ubeta = mag * np.sin(angle)

        sector, Ta, Tb, Tc = svpwm_duty_cycle(Ualpha, Ubeta, Vdc)

        # Check saturation
        saturated = any(d >= 0.95 or d <= 0.05 for d in [Ta, Tb, Tc])

        results.append({
            'magnitude': mag,
            'Ta': Ta,
            'Tb': Tb,
            'Tc': Tc,
            'saturated': saturated
        })

        sat_str = "Yes" if saturated else "No"
        print(f"  {mag:12} | {Ta:.3f} | {Tb:.3f} | {Tc:.3f} | {sat_str}")

    # Plot duty cycles vs magnitude
    fig, ax = plt.subplots(figsize=(10, 6))

    mags = [r['magnitude'] for r in results]
    Tas = [r['Ta'] for r in results]
    Tbs = [r['Tb'] for r in results]
    Tcs = [r['Tc'] for r in results]

    ax.plot(mags, Tas, 'r-o', linewidth=2, markersize=8, label='Ta')
    ax.plot(mags, Tbs, 'g-o', linewidth=2, markersize=8, label='Tb')
    ax.plot(mags, Tcs, 'b-o', linewidth=2, markersize=8, label='Tc')

    # Mark saturation limits
    ax.axhline(y=0.95, color='k', linestyle='--', alpha=0.5, label='Max limit')
    ax.axhline(y=0.05, color='k', linestyle=':', alpha=0.5, label='Min limit')

    ax.set_xlabel('Voltage Magnitude [V]')
    ax.set_ylabel('Duty Cycle')
    ax.set_title('SVPWM Duty Cycles vs Voltage Magnitude')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 1)

    plt.tight_layout()
    plt.savefig('svpwm_voltage_limit.png', dpi=150)
    print("  Saved: svpwm_voltage_limit.png")
    plt.show()

    return results    

if __name__ == '__main__':
    # run all svpwm simulations
    angles, sectors, Ta, Tb, Tc = run_svpwm_rotation_simulation()

    # run DQ input simulation
    time, Ta, Tb, Tc = run_svpwm_with_dq_input()

    # analyze sector boundaries
    transitions = run_svpwm_sector_boundary_analysis()

    # analyze voltage limits
    results = run_svpwm_voltage_limit_analysis()

