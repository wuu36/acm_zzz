import numpy as np

D_MAX = 0.90

def calculate_critical_inductance(D, R, f):
    D = np.clip(D, 0.01, D_MAX)
    return (1 - D)**2 * D * R / (2 * f)

def determine_mode(L, Lc):
    if L >= Lc:
        return "CCM"
    return "DCM"

def solve_ccm_ideal(Vin, D):
    D = np.clip(D, 0.01, D_MAX)
    return Vin / (1 - D)

def solve_dcm_gain(Vin, D, R, L, f):
    D = np.clip(D, 0.01, D_MAX)
    discriminant = 1 + 8 * L * f / (R * D**2)
    gain = (1 + np.sqrt(discriminant)) / 2
    return Vin * gain, gain

def solve_ccm_with_loss(Vin, D, R, RL, Vd=0.5):
    D = np.clip(D, 0.01, D_MAX)
    gain = (1 / (1 - D)) * (R / (R + RL / (1-D)**2))
    Vout = Vin * gain - Vd
    return Vout, gain

def solve_closed_loop_duty(Vin, Vout_target, R, RL, Vd=0.5):
    D_approx = 1 - Vin / Vout_target
    if D_approx < 0 or D_approx > D_MAX:
        return D_approx
    for _ in range(20):
        Vout_calc, _ = solve_ccm_with_loss(Vin, D_approx, R, RL, Vd)
        error = Vout_target - Vout_calc
        d_error = Vin * R * RL / ((R + RL / (1 - D_approx)**2)**2 * (1 - D_approx)**3) + Vin / (1 - D_approx)**2 * (1 - R * RL / ((R + RL / (1 - D_approx)**2)**2 * (1 - D_approx)**2))
        if abs(d_error) < 1e-10:
            break
        D_approx = D_approx + error / d_error
        D_approx = np.clip(D_approx, 0.01, D_MAX)
        if abs(error) < 0.001:
            break
    return float(D_approx)

def solve_steady_state(Vin, D, R, L, C, f, RL=0.05, Vd=0.5):
    D = float(np.clip(D, 0.01, D_MAX))

    Lc = calculate_critical_inductance(D, R, f)
    mode = determine_mode(L, Lc)
    
    if mode == "CCM":
        Vout, gain = solve_ccm_with_loss(Vin, D, R, RL, Vd)
    else:
        Vout, gain = solve_dcm_gain(Vin, D, R, L, f)
    
    if mode == "CCM":
        IL_avg = Vout / (R * (1 - D)) if (1 - D) > 0 else 0
        delta_IL = Vin * D / (f * L)
    else:
        delta_IL = Vin * D / (f * L)
        IL_avg = delta_IL / 2

    delta_Vout = Vout * D / (f * R * C)
    IL_peak = IL_avg + delta_IL / 2
    Iout = Vout / R
    Pin = Vin * IL_avg + Vin * delta_IL**2 / (12 * f * L) * RL if IL_avg > 0 else 0
    Pout = Vout * Iout
    efficiency = Pout / Pin if Pin > 0 else 0

    Vout_ideal = solve_ccm_ideal(Vin, D)

    return {
        "mode": mode,
        "D": D,
        "Vout": Vout,
        "Vout_ideal": Vout_ideal,
        "gain": gain,
        "Lc": Lc,
        "IL_avg": IL_avg,
        "IL_peak": IL_peak,
        "delta_IL": delta_IL,
        "delta_Vout": delta_Vout,
        "Iout": Iout,
        "Pin": Pin,
        "Pout": Pout,
        "efficiency": efficiency,
    }
