import numpy as np

D_MAX = 0.90

def simulate_transient(Vin, D, R, L, C, RL, Vd, f,
                       mode="open_loop",
                       Vout_target=None, Kp=0.1, Ki=10.0,
                       t_sim=10e-3, dt=1e-7):
    T = 1.0 / f
    N = int(t_sim / dt)
    steps_per_period = int(T / dt)
    n_on_base = int(D * steps_per_period)

    iL = np.zeros(N)
    vC = np.zeros(N)
    switch_state = np.zeros(N)
    t = np.linspace(0, t_sim, N)

    IL_init = Vin / (R * (1 - D)) * D * 0.5
    vC_init = Vin + (Vin / (1 - D) - Vin) * 0.6
    iL[0] = max(IL_init, 0.0)
    vC[0] = max(vC_init, Vin)

    pi_integral = 0.0
    current_D = D

    for n in range(N - 1):
        step_in_period = n % steps_per_period

        if mode == "closed_loop" and Vout_target is not None:
            error = Vout_target - vC[n]
            pi_integral += error * dt
            pi_integral = np.clip(pi_integral, -Vout_target * Ki * 1e-3, Vout_target * Ki * 1e-3)
            current_D = D + Kp * error + Ki * pi_integral
            current_D = np.clip(current_D, 0.01, D_MAX)
            n_on = int(current_D * steps_per_period)
        else:
            current_D = D
            n_on = n_on_base

        is_on = step_in_period < n_on
        switch_state[n] = 1 if is_on else 0

        if is_on:
            di_dt = (Vin - iL[n] * RL) / L
            dv_dt = -vC[n] / (R * C)
        else:
            if iL[n] > 1e-10:
                di_dt = (Vin - vC[n] - iL[n] * RL - Vd) / L
                dv_dt = (iL[n] - vC[n] / R) / C
            else:
                di_dt = 0.0
                dv_dt = -vC[n] / (R * C)

        iL[n+1] = iL[n] + di_dt * dt
        vC[n+1] = vC[n] + dv_dt * dt

        if iL[n+1] < 0:
            iL[n+1] = 0.0

        if vC[n+1] < 0:
            vC[n+1] = 0.0

    steady_start = int(N * 0.6)

    return {
        "t": t,
        "iL": iL,
        "vC": vC,
        "switch_state": switch_state,
        "D_actual": current_D,
        "iL_steady_avg": np.mean(iL[steady_start:]),
        "vC_steady_avg": np.mean(vC[steady_start:]),
        "iL_steady_peak": np.max(iL[steady_start:]),
        "vC_steady_peak": np.max(vC[steady_start:]),
        "vC_steady_min": np.min(vC[steady_start:]),
    }

def get_steady_waveforms(sim_data, f, num_periods=10):
    T = 1.0 / f
    t = sim_data["t"]
    N = len(t)
    steady_start_idx = int(N * 0.8)
    t_steady = t[steady_start_idx:]
    period_start_idx = int((t_steady[-1] - num_periods * T) / (t[1] - t[0]))
    period_start_idx = max(steady_start_idx, N - int(num_periods * T / (t[1] - t[0])))

    mask = np.arange(period_start_idx, N)
    return {
        "t": t[mask],
        "iL": sim_data["iL"][mask],
        "vC": sim_data["vC"][mask],
        "switch_state": sim_data["switch_state"][mask],
    }