from dataclasses import dataclass, field
import numpy as np

from .regulator import PIRegulator, incremental_pi
from .controller_params import ControllerParams, FOCConfig

@dataclass
class FOCController:
    """FOC controller combining speed and current loops.

    Implements cascade control:
        Speed loop (outer) → iq current command
        Current loop (inner) → ud, uq voltage commands

    Attributes:
        params: ControllerParams instance
        config: FOCConfig instance
        speed_reg: PI regulator for speed loop
        id_reg: PI regulator for d-axis current loop
        iq_reg: PI regulator for q-axis current loop

    Example:
        >>> params = ControllerParams(speed_Kp=0.038, speed_Ki=30.5)
        >>> config = FOCConfig(control_mode='speed')
        >>> foc = FOCController(params, config)
        >>> foc.set_speed_command(1000.0)  # 1000 rpm
        >>> foc.execute(omega_elec, id, iq)
        >>> ud, uq = foc.get_voltage_commands()
    """

    params: ControllerParams
    config: FOCConfig

    # PI regulators
    # speed_reg: PIRegulator | None = None
    # id_reg: PIRegulator | None = None
    # iq_reg: PIRegulator | None = None
    speed_reg: PIRegulator = field(init=False)
    id_reg: PIRegulator = field(init=False)
    iq_reg: PIRegulator = field(init=False)

    # commands
    cmd_rpm: float = 0.0
    # cmd_idq: np.ndarray | None = None
    # cmd_udq: np.ndarray | None = None
    cmd_idq: np.ndarray = field(default_factory=lambda: np.zeros(2))
    cmd_udq: np.ndarray = field(default_factory=lambda: np.zeros(2))

    # feedback
    omega_r_elec: float = 0.0
    # idq: np.ndarray | None = None
    idq: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def __post_init__(self):
        """initialize PI regulators and arrays."""
        self.speed_reg = PIRegulator(
            Kp=self.params.speed_Kp,
            Ki=self.params.speed_Ki,
            OutLimit=self.params.speed_limit
        )
        
        self.id_reg = PIRegulator(
            Kp=self.params.current_Kp,
            Ki=self.params.current_Ki,
            OutLimit=self.params.current_limit
        )
        
        self.iq_reg = PIRegulator(
            Kp=self.params.current_Kp,
            Ki=self.params.current_Ki,
            OutLimit=self.params.current_limit
        )
        
        if self.cmd_idq is None:
            self.cmd_idq = np.zeros(2, dtype=np.float64)

        if self.cmd_udq is None:
            self.cmd_udq = np.zeros(2, dtype=np.float64)
        
        if self.idq is None:
            self.idq = np.zeros(2, dtype=np.float64)
        
    def set_speed_command(self, rpm: float):
        self.cmd_rpm = rpm
        
    def set_torque_command(self, torque: float):
        pass

    def get_speed_command_elec(self, npp: int) -> float:
        return self.cmd_rpm / 60.0 * 2.0 * np.pi * npp
    
    def execute(self, omega_r_elec: float, id_: float, iq: float,
                motor_params=None, Vdc: float = 300.0):
        assert self.speed_reg is not None
        assert self.id_reg is not None
        assert self.iq_reg is not None
        assert self.idq is not None
        assert self.cmd_idq is not None
        assert self.cmd_udq is not None

        self.omega_r_elec = omega_r_elec
        self.idq[0] = id_
        self.idq[1] = iq

        # speed loop (executes at VL_TS rate)
        speed_ref = self.get_speed_command_elec(
            motor_params.npp if motor_params else 4
        )
        
        self.speed_reg.Ref = speed_ref
        self.speed_reg.Fbk = omega_r_elec

        # check is speed loop should execute this cycle
        execute_speed = self.params.increment_velocity_counter()

        if execute_speed:
            incremental_pi(self.speed_reg)
        
        # iq command from speed regulator output
        self.cmd_idq[1] = self.speed_reg.Out

        # id command based on strategy
        if motor_params is not None:
            omega_elec = omega_r_elec
            self.cmd_idq[0] = self.config.get_id_command(
                motor_params, omega_elec, Vdc
            )
        else:
            self.cmd_idq[0] = 0.0

        # d-axis current loop
        self.id_reg.Ref = self.cmd_idq[0]
        self.id_reg.Fbk = self.idq[0]
        incremental_pi(self.id_reg)
        self.cmd_udq[0] = self.id_reg.Out

        # q-axis current loop
        self.iq_reg.Ref = self.cmd_idq[1]
        self.iq_reg.Fbk = self.idq[1]
        incremental_pi(self.iq_reg)
        self.cmd_udq[1] = self.iq_reg.Out

    def get_voltage_commands(self) -> tuple[float, float]:
        assert self.cmd_udq is not None

        return self.cmd_udq[0], self.cmd_udq[1]
    
    def get_current_commands(self) -> tuple[float, float]:
        assert self.cmd_idq is not None

        return self.cmd_idq[0], self.cmd_idq[1]
    
    def reset(self):
        """reset all regulators to zero state."""
        assert self.speed_reg is not None
        assert self.id_reg is not None
        assert self.iq_reg is not None
        assert self.cmd_idq is not None
        assert self.cmd_udq is not None
        
        self.speed_reg.reset()
        self.id_reg.reset()
        self.iq_reg.reset()
        self.params.reset_velocity_counter()
        self.cmd_rpm = 0.0
        self.cmd_idq.fill(0.0)
        self.cmd_udq.fill(0.0)

def foc_control(
        params: ControllerParams,
        config: FOCConfig,
        speed_reg: PIRegulator,
        id_reg: PIRegulator,
        iq_reg: PIRegulator,
        cmd_rpm: float,
        omega_r_elec: float,
        id_: float,
        iq: float,
        motor_params=None,
        Vdc: float = 300.0
) -> tuple[float, float]:
    npp = motor_params.npp if motor_params else 4

    # speed loop
    speed_ref = cmd_rpm / 60.0 * 2.0 * np.pi * npp
    speed_reg.Ref = speed_ref
    speed_reg.Fbk = omega_r_elec
    
    # velocity loop counter
    execute_speed = params.increment_velocity_counter()

    if execute_speed:
        incremental_pi(speed_reg)

    # current commands
    cmd_idq = np.zeros(2, dtype=np.float64)
    cmd_idq[1] = speed_reg.Out

    # id command
    if motor_params is not None:
        cmd_idq[0] = config.get_id_command(motor_params, omega_r_elec, Vdc)
    else:
        cmd_idq[0] = 0.0

    # d-axis current loop
    id_reg.Ref = cmd_idq[0]
    id_reg.Fbk = id_
    incremental_pi(id_reg)
    ud = id_reg.Out
    
    # q-axis current loop
    iq_reg.Ref = cmd_idq[1]
    iq_reg.Fbk = iq
    incremental_pi(iq_reg)
    uq = iq_reg.Out

    return ud, uq
