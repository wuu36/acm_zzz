from dataclasses import dataclass, field
import numpy as np

from .regulator import PIRegulator
from .controller_params import ControllerParams, FOCConfig
from .foc import foc_control
from ..core.machine_params import MachineParams

@dataclass
class MotorController:
    # parameters
    # motor_params: MachineParams = field(init=False)
    # controller_params: ControllerParams = field(init=False)
    # foc_config: FOCConfig = field(init=False)
    motor_params: MachineParams = field(default_factory=MachineParams)
    controller_params: ControllerParams = field(default_factory=ControllerParams)
    foc_config: FOCConfig = field(default_factory=FOCConfig)
    
    # pi regulators (can be passed in or auto-created)
    speed_reg: PIRegulator | None = None
    id_reg: PIRegulator | None = None
    iq_reg: PIRegulator | None = None
    
    # time and state
    timebase: float = 0.0
    
    # feedback signals (input from motor)
    theta_d: float = 0.0
    omega_r_elec: float = 0.0
    omega_syn: float = 0.0
    idq: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    
    # motor state estimates
    KA: float = 0.0
    Tem: float = 0.0

    # commands (output to motor)
    cmd_rpm: float = 0.0
    cmd_idq: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    cmd_udq: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    cmd_uab: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    
    # observer states
    observer_order: int = 3     # observer order (2, 3, or 4)
    observer_bw: float = 100.0  # observer bandwidth [rad/s]
    use_observer: bool = False  # use speed observer instead of encoder
    use_diturbance_ff: bool = False # use disturbance feedforward
    
    # observer internal states
    xS: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))
    observer_output_error: float = 0.0
    total_disturbance_ff: float = 0.0

    # observer gains (computed based on order)
    ell1: float = 0.0
    ell2: float = 0.0
    ell3: float = 0.0
    ell4: float = 0.0
    
    # signal sweep parameters
    speed_sine_amp_rpm: float = 100.0       # speed sine amplitude [rpm]
    speed_sine_hz: float = 0.0              # speed sine frequency [Hz]
    speed_sine_hz_ceiling: float = 100.0    # max sweep frequency [Hz] 
    speed_sine_step_size: float = 1.0       # frequency step [Hz]
    current_sine_amp: float = 1.0           # current sine amplitude [A]

    def __post_init__(self):
        """Initialize controller with default parameters if not provided."""
        # Initialize KA from motor params
        self.KA = self.motor_params.KE

        # Initialize PI regulators if not provided
        if self.speed_reg is None:
            self.speed_reg = PIRegulator(
                Kp=self.controller_params.speed_Kp,
                Ki=self.controller_params.speed_Ki,
                OutLimit=self.controller_params.speed_limit
            )

        if self.id_reg is None:
            self.id_reg = PIRegulator(
                Kp=self.controller_params.current_Kp,
                Ki=self.controller_params.current_Ki,
                OutLimit=self.controller_params.current_limit
            )

        if self.iq_reg is None:
            self.iq_reg = PIRegulator(
                Kp=self.controller_params.current_Kp,
                Ki=self.controller_params.current_Ki,
                OutLimit=self.controller_params.current_limit
            )

        # Compute observer gains
        self._compute_observer_gains()
    
    def _compute_observer_gains(self):
        """Compute observer gains based on order and bandwidth."""
        omega_ob = self.observer_bw
        Js = self.motor_params.Js
        npp = self.motor_params.npp

        if self.observer_order == 2:
            # 2nd-order position observer
            self.ell1 = 2 * omega_ob
            self.ell2 = omega_ob**2 * Js / npp
        elif self.observer_order == 3:
            # 3rd-order position observer (default)
            self.ell1 = 3 * omega_ob
            self.ell2 = 3 * omega_ob**2
            self.ell3 = omega_ob**3 * Js / npp
        elif self.observer_order == 4:
            # 4th-order position observer
            self.ell1 = 4 * omega_ob
            self.ell2 = 6 * omega_ob**2
            self.ell3 = 4 * omega_ob**3 * Js / npp
            self.ell4 = omega_ob**4

    def set_speed_command(self, rpm: float):
        """Set speed command in RPM.

        Args:
            rpm: Speed command [rpm]
        """
        self.cmd_rpm = rpm

    def set_torque_command(self, torque: float):
        """Set torque command (direct torque control).

        Args:
            torque: Torque command [N·m]
        """
        # Convert torque to iq command
        # iq = torque / (1.5 * npp * KA)
        if self.KA > 0:
            iq_cmd = torque / (1.5 * self.motor_params.npp * self.KA)
            self.cmd_idq[1] = iq_cmd
        else:
            self.cmd_idq[1] = 0.0

    def get_speed_command_elec(self) -> float:
        """Convert RPM command to electrical angular speed.

        Returns:
            Electrical angular speed [rad/s]
        """
        return self.cmd_rpm / 60.0 * 2.0 * np.pi * self.motor_params.npp

    def update_time(self):
        """Update timebase by one current loop period."""
        self.timebase += self.controller_params.CL_TS

    def update_feedback(self, theta_d: float, omega_r_elec: float,
                        id_: float, iq: float):
        """Update feedback signals from motor measurements.

        Args:
            theta_d: Rotor position in d-frame [rad]
            omega_r_elec: Electrical rotor speed [rad/s]
            id_: d-axis current [A]
            iq: q-axis current [A]
        """
        self.theta_d = theta_d
        self.omega_r_elec = omega_r_elec
        self.idq[0] = id_
        self.idq[1] = iq

        # Compute active flux and torque
        self.KA = (self.motor_params.Ld - self.motor_params.Lq) * self.idq[0] + self.motor_params.KE
        self.Tem = 1.5 * self.motor_params.npp * self.idq[1] * self.KA

    def execute_foc(self, Vdc: float = 300.0):
        """Execute FOC control step.

        Args:
            Vdc: DC bus voltage [V] (for flux weakening)
        """
        assert self.speed_reg is not None
        assert self.id_reg is not None
        assert self.iq_reg is not None

        ud, uq = foc_control(
            self.controller_params,
            self.foc_config,
            self.speed_reg,
            self.id_reg,
            self.iq_reg,
            self.cmd_rpm,
            self.omega_r_elec,
            self.idq[0],
            self.idq[1],
            self.motor_params,
            Vdc
        )

        self.cmd_udq[0] = ud
        self.cmd_udq[1] = uq

        # Copy iq command from speed regulator
        self.cmd_idq[1] = self.speed_reg.Out
        self.cmd_idq[0] = self.id_reg.Ref

    def execute(self, omega_r_elec: float, id_: float, iq: float,
                theta_d: float = 0.0, Vdc: float = 300.0):
        """Execute one complete control step.

        This combines time update, feedback update, and FOC execution.

        Args:
            omega_r_elec: Electrical rotor speed [rad/s]
            id_: d-axis current [A]
            iq: q-axis current [A]
            theta_d: Rotor position [rad] (optional)
            Vdc: DC bus voltage [V]
        """
        self.update_time()
        self.update_feedback(theta_d, omega_r_elec, id_, iq)
        self.execute_foc(Vdc)

    def get_voltage_commands_dq(self) -> tuple[float, float]:
        """Get voltage commands in dq frame.

        Returns:
            Tuple of (ud, uq) voltage commands [V]
        """
        return self.cmd_udq[0], self.cmd_udq[1]

    def get_voltage_commands_ab(self) -> tuple[float, float]:
        """Get voltage commands in alpha-beta frame.

        Requires theta_d to be set for inverse Park transform.

        Returns:
            Tuple of (u_alpha, u_beta) voltage commands [V]
        """
        cosT = np.cos(self.theta_d)
        sinT = np.sin(self.theta_d)

        ud, uq = self.cmd_udq[0], self.cmd_udq[1]

        # Inverse Park transform
        u_alpha = ud * cosT - uq * sinT
        u_beta = ud * sinT + uq * cosT

        self.cmd_uab[0] = u_alpha
        self.cmd_uab[1] = u_beta

        return u_alpha, u_beta

    def reset(self):
        """Reset controller to initial state."""
        assert self.speed_reg is not None
        assert self.id_reg is not None
        assert self.iq_reg is not None

        self.timebase = 0.0
        self.theta_d = 0.0
        self.omega_r_elec = 0.0
        self.KA = self.motor_params.KE
        self.Tem = 0.0

        self.speed_reg.reset()
        self.id_reg.reset()
        self.iq_reg.reset()

        self.controller_params.reset_velocity_counter()

        self.cmd_rpm = 0.0
        self.cmd_idq.fill(0.0)
        self.cmd_udq.fill(0.0)
        self.cmd_uab.fill(0.0)
        self.idq.fill(0.0)
        self.xS.fill(0.0)

    def set_pi_gains(self, speed_Kp: float | None = None, speed_Ki: float | None = None,
                     current_Kp: float | None = None, current_Ki: float | None = None):
        """Update PI regulator gains.

        Args:
            speed_Kp: Speed loop proportional gain (optional)
            speed_Ki: Speed loop integral gain (optional)
            current_Kp: Current loop proportional gain (optional)
            current_Ki: Current loop integral gain (optional)
        """
        assert self.speed_reg is not None
        assert self.id_reg is not None
        assert self.iq_reg is not None

        if speed_Kp is not None:
            self.speed_reg.Kp = speed_Kp
            self.controller_params.speed_Kp = speed_Kp

        if speed_Ki is not None:
            self.speed_reg.Ki = speed_Ki
            self.controller_params.speed_Ki = speed_Ki

        if current_Kp is not None:
            self.id_reg.Kp = current_Kp
            self.iq_reg.Kp = current_Kp
            self.controller_params.current_Kp = current_Kp

        if current_Ki is not None:
            self.id_reg.Ki = current_Ki
            self.iq_reg.Ki = current_Ki
            self.controller_params.current_Ki = current_Ki

    def auto_design_gains(self, current_bw_hz: float = 1000.0,
                          speed_bw_hz: float = 50.0):
        """Auto-design PI gains based on motor parameters.

        Args:
            current_bw_hz: Desired current loop bandwidth [Hz]
            speed_bw_hz: Desired speed loop bandwidth [Hz]
        """
        assert self.speed_reg is not None
        assert self.id_reg is not None
        assert self.iq_reg is not None

        from .controller_params import auto_design_pi_gains

        current_Kp, current_Ki, speed_Kp, speed_Ki = auto_design_pi_gains(
            self.motor_params,
            self.controller_params,
            current_bw_hz,
            speed_bw_hz
        )

        # Update regulators
        self.speed_reg.Kp = speed_Kp
        self.speed_reg.Ki = speed_Ki
        self.id_reg.Kp = current_Kp
        self.id_reg.Ki = current_Ki
        self.iq_reg.Kp = current_Kp
        self.iq_reg.Ki = current_Ki

    def get_info(self) -> dict:
        """Get controller information summary.

        Returns:
            Dictionary with controller parameters and state
        """
        assert self.speed_reg is not None
        assert self.id_reg is not None

        return {
            'time': self.timebase,
            'speed_cmd_rpm': self.cmd_rpm,
            'omega_elec': self.omega_r_elec,
            'KA': self.KA,
            'Tem': self.Tem,
            'id': self.idq[0],
            'iq': self.idq[1],
            'ud': self.cmd_udq[0],
            'uq': self.cmd_udq[1],
            'speed_Kp': self.speed_reg.Kp,
            'speed_Ki': self.speed_reg.Ki,
            'current_Kp': self.id_reg.Kp,
            'current_Ki': self.id_reg.Ki,
        }

    
