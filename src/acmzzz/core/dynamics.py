
import numpy as np
from .machine import ACMachine, CLARKE_TRANS_TORQUE_GAIN

def dynamics_machine(t: float, x: np.ndarray, machine: ACMachine) -> np.ndarray:
    fx = np.zeros(5, dtype=np.float64)

    # unpack state vector
    theta_d_mech = x[0]
    omega_r_mech = x[1]
    KA = x[2]
    iD = x[3]
    iQ = x[4]

    # motor parameters
    params = machine.params

    # input voltages
    uD = machine.state.udq[0]
    uQ = machine.state.udq[1]

    # load torque
    TLoad = machine.state.Tload

    # calculate slip speed and synchronous speed
    # for IM: omega_slip = Rreq * iQ / KA
    # for PMSM: omega_slip = 0
    if params.Rreq > 0 and KA > 0:
        omega_slip = params.Rreq * iQ / KA
    else:
        omega_slip = 0.0
    
    omega_r_elec = omega_r_mech * params.npp
    omega_syn = omega_r_elec + omega_slip

    # ======================================
    # electromagnetic dynamics
    # ======================================
    if params.Rreq > 0:
        # Induction Motor (IM) - KA has dynamics
        # dKA/dt = Rreq * iD - Rreq * KA / (Ld - Lq)
        # This is from Park transform of rotor flux dynamics
        if params.Ld != params.Lq:
            fx[2] = params.Rreq * iD - params.Rreq * KA / (params.Ld - params.Lq)
        else:
            fx[2] = params.Rreq * iD

        # diD/dt for IM:
        # Note: the voltage equation includes the KA dynamics term
        fx[3] = (uD - params.R * iD + omega_syn * params.Lq * iQ - fx[2]) / params.Lq
    else:
        # PMSM - KA is quasi-static (depends on iD)
        # KA = (Ld - Lq) * iD + KE
        # dKA/dt = (Ld - Lq) * diD/dt

        # diD/dt for PMSM:
        fx[3] = (uD - params.R * iD + omega_syn * params.Lq * iQ) / params.Ld

        # dKA/dt for PMSM (quasi-static):
        fx[2] = (params.Ld - params.Lq) * fx[3] 

    # diQ/dt (same for both PMSM and IM):
    fx[4] = (uQ - params.R * iQ - omega_syn * params.Ld * iD - omega_syn * KA) / params.Lq

    # ======================================
    # mechanical dynamics
    # ======================================
    # Electromagnetic torque: Tem = 1.5 * npp * KA * iQ
    Tem = CLARKE_TRANS_TORQUE_GAIN * params.npp * KA * iQ

    # Position derivative: dtheta/dt = omega + slip/npp
    fx[0] = omega_r_mech + omega_slip / params.npp

    # Speed derivative: domega/dt = (Tem - TLoad) / Js
    fx[1] = (Tem - TLoad) / params.Js

    return fx

def dynamics_machine_current_source(t: float, x: np.ndarray, machine: ACMachine,
                                    iD_source: float, iQ_source: float) -> np.ndarray:
    fx = np.zeros(5, dtype=np.float64)

    theta_d_mech = x[0]
    omega_r_mech = x[1]

    params = machine.params
    TLoad = machine.state.Tload

    # currents are directly set (no dynamics)
    iD = iD_source
    iQ = iQ_source

    # active flux for PMSM (static)
    if params.Rreq > 0:
        # IM: use current KA from state
        KA = x[2]
        omege_slip = params.Rreq * iQ / KA if KA > 0 else 0.0
    else:
        # PMSM: KA = (Ld-Lq)*iD + KE
        KA = (params.Ld - params.Lq) * iD + params.KE
        omege_slip = 0.0
    
    # electromagnetic torque
    Tem = CLARKE_TRANS_TORQUE_GAIN * params.npp * KA * iQ

    # position and speed dynamics
    fx[0] = omega_r_mech + omege_slip / params.npp
    fx[1] = (Tem - TLoad) / params.Js

    # current states are static (derivative = 0 for current source mode)
    fx[3] = 0.0
    fx[4] = 0.0

    # KA dynamics
    if params.Rreq > 0:
        fx[2] = params.Rreq * iD - params.Rreq * KA / (params.Ld - params.Lq)
    else:
        fx[2] = 0.0
    
    return fx

def dynamics_only_mechanical(t: float, x: np.ndarray, machine: ACMachine,
                             Tem_source: float) -> np.ndarray:
    fx = np.zeros(5, dtype=np.float64)
    
    omega_r_mech = x[1]
    params = machine.params
    TLoad = machine.state.Tload

    # only mechanical dynamics
    fx[0] = omega_r_mech    # position
    fx[1] = (Tem_source - TLoad) / params.Js    # speed

    # electromagnetic states unchanged
    fx[2] = 0.0
    fx[3] = 0.0
    fx[4] = 0.0

    return fx