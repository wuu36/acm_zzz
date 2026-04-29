
from dataclasses import dataclass
import numpy as np
from .machine_params import MachineParams
from .machine_state import MachineState
from .transforms import inverse_park


# clarke transformation gain for torque calculation
CLARKE_TRANS_TORQUE_GAIN = 1.5

@dataclass
class ACMachine:
    params: MachineParams
    state: MachineState

    def __post_init__(self):
        """initialize machine after creation."""
        # set initial flux linkage
        self.state.set_initial_flux(self.params.KE)

    def calculate_torque(self) -> float:
        return CLARKE_TRANS_TORQUE_GAIN * self.params.npp * self.state.KA * self.state.iQ
    
    def update_outputs(self):
        self.state.sync_from_state_vector(self.params)
        self.state.Tem = self.calculate_torque()
    
    def set_voltage_dq(self, uD: float, uQ: float):
        self.state.udq[0] = uD
        self.state.udq[1] = uQ

    def set_voltage_ab(self, ualpha: float, ubeta: float):
        self.state.uab[0] = ualpha
        self.state.uab[1] = ubeta

    def set_load_torque(self, TLoad: float):
        self.state.Tload = TLoad

    def set_slip_speed(self, omega_slip: float):
        self.state.omega_slip = omega_slip

    def get_electrical_angle(self) -> float:
        return self.state.theta_d_mech * self.params.npp
    
    def get_electrical_speed(self) -> float:
        return self.state.omega_r_mech * self.params.npp
    
    def get_speed_rpm(self) -> float:
        return self.state.omega_r_mech * 60.0 / (2.0 * np.pi)
    
    def set_speed_rpm(self, rmp: float):
        self.state.omega_r_mech = rmp * 2.0 * np.pi / 60.0
        self.state.x[1] = self.state.omega_r_mech

    def reset(self):
        self.state.reset()
        self.state.set_initial_flux(self.params.KE)
        
    def copy_state(self) -> np.ndarray:
        return self.state.x.copy()
    
    def set_state(self, x: np.ndarray):
        self.state.x = x.copy()
        self.update_outputs()
        
    def get_info(self) -> dict:
        return {
            'motor_type': 'PMSM' if self.params.is_pmsm else 'Induction Motor',
            'pole_pairs': self.params.npp,
            'resistance': self.params.R,
            'Ld': self.params.Ld,
            'Lq': self.params.Lq,
            'KE': self.params.KE,
            'inertia': self.params.Js,
            'speed_rpm': self.get_speed_rpm(),
            'torque': self.state.Tem,
            'iD': self.state.iD,
            'iQ': self.state.iQ
        }
        
    
        