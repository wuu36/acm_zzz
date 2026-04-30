from dataclasses import dataclass
import numpy as np

@dataclass
class ControllerParams:
    """FOC controller parameters.

    Stores timing and gain parameters for current and speed loops
    in a FOC motor control system.

    Timing attributes:
        CL_TS: Current loop sampling period [s]
        VL_TS: Speed loop sampling period [s]
        VL_ceiling: Speed loop execution ceiling (VL_TS = CL_TS * VL_ceiling)

    PI gain attributes:
        speed_Kp: Speed loop proportional gain
        speed_Ki: Speed loop integral gain
        speed_limit: Speed loop output limit (iq current limit) [A]

        current_Kp: Current loop proportional gain
        current_Ki: Current loop integral gain
        current_limit: Current loop output limit (voltage limit) [V]

    Example:
        >>> params = ControllerParams(
        >>>     CL_TS=1e-4,
        >>>     VL_TS=5e-4,
        >>>     speed_Kp=0.038,
        >>>     speed_Ki=30.5,
        >>>     current_Kp=6.4,
        >>>     current_Ki=1523.2,
        >>> )
    """

    # timing parameters
    CL_TS: float = 1e-4     # current loop sampling period [s]
    VL_TS: float = 5e-4     # speed loop sampling period [s]
    VL_ceiling: int = 5     # speed loop runs every N current loops
    
    # speed loop PI gains
    speed_Kp: float = 0.038
    speed_Ki: float = 30.5
    speed_limit: float = 10.0   # iq current limit [A]
    
    # current loop PI gains
    current_Kp: float = 6.4
    current_Ki: float = 1523.2  # Ki * Kp * CL_TS form
    current_limit: float = 600.0    # voltage limit [V]
    
    # controller state
    velocity_loop_counter: int = 0

    def __post_init__(self):
        """validate and compute derived parameters."""
        # verify timing relationship
        expected_VL_TS = self.CL_TS * self.VL_ceiling
        if abs(self.VL_TS - expected_VL_TS) > 1e-6:
            # auto-correct VL_TS based on CL_TS and ceiling
            self.VL_TS = self.CL_TS * self.VL_ceiling

    def get_current_loop_rate_hz(self) -> float:
        """get current loop execution rate in Hz."""
        return 1.0 / self.CL_TS
    
    def get_speed_loop_rate_hz(self) -> float:
        """get speed loop execution rate in Hz."""
        return 1.0 / self.VL_TS
    
    def increment_velocity_counter(self) -> bool:
        """Increment velocity loop counter and return True if speed loop should execute.

        Returns:
            True if speed loop should execute this cycle
        """
        self.velocity_loop_counter += 1
        if self.velocity_loop_counter >= self.VL_ceiling:
            self.velocity_loop_counter = 0
            return True
        return False
    
    def reset_velocity_counter(self):
        """reset velocity loop counter."""
        self.velocity_loop_counter = 0

@dataclass
class FOCConfig:
    """FOC (Field Oriented Control) configuration.

    Higher-level configuration for FOC control strategy,
    including control mode and options.

    Attributes:
        control_mode: Control mode ('speed', 'torque', 'position')
        id_command_strategy: D-axis current command strategy
            - 'mtpa': Maximum Torque Per Ampere (id=0 for surface PMSM)
            - 'mtpv': Maximum Torque Per Volt (flux weakening)
            - 'fixed': Fixed id command
        id_command_value: Fixed id command value (if strategy='fixed') [A]
        use_observer: Whether to use speed observer instead of encoder
        use_disturbance_feedforward: Whether to use disturbance feedforward
    """

    control_mode: str = 'speed'         # 'speed', 'torque', 'position'
    id_command_strategy: str = 'mtpa'   # 'mtpa', 'mtpv', 'fixed'
    id_command_value: float = 0.0       # fixed id command [A]
    use_observer: bool = False          # use speed observer
    use_disturbance_feedforward: bool = False   # use disturbance feedforward
    
    def get_id_command(self, motor_params, omega_elec: float, Vdc: float) -> float:
        """Calculate d-axis current command based on strategy.

        Args:
            motor_params: MachineParams instance
            omega_elec: Electrical angular speed [rad/s]
            Vdc: DC bus voltage [V]

        Returns:
            id command [A]
        """
        if self.id_command_strategy == 'mtpa':
            # for surface PMSM (Ld ≈ Lq), MTPA means id = 0
            return 0.0
        elif self.id_command_strategy == 'mtpv':
            # flux weakening: negative id for high speed
            # simplified: id = -KE * omega / (Ld * omega) for demo
            # real implementation needs voltage limit calculation
            if omega_elec > 0:
                return -motor_params.KE * 0.1 / motor_params.Ld
            return 0.0
        elif self.id_command_strategy == 'fixed':
            return self.id_command_value
        else:
            return 0.0
        
def design_current_pi_gains(R: float, L: float, bandwidth_hz: float) -> tuple[float, float]:
    """Design current loop PI gains based on desired bandwidth.

    Uses zero-pole cancellation method for current loop design:
    - Kp = bandwidth * 2*pi * L
    - Ki = R / L

    Args:
        R: Stator resistance [Ohm]
        L: Inductance [H]
        bandwidth_hz: Desired current loop bandwidth [Hz]

    Returns:
        Tuple of (Kp, Ki)
    """
    Kp = bandwidth_hz * 2.0 * np.pi * L
    Ki = R / L
    return Kp, Ki


def design_speed_pi_gains(Js: float, npp: int, KA: float,
                          delta: float, current_bw_rad: float) -> tuple[float, float]:
    """Design speed loop PI gains using symmetric optimal method.

    Args:
        Js: Total inertia [kg*m^2]
        npp: Pole pairs
        KA: Active flux [Wb]
        delta: Symmetric optimal parameter (typically 3-10)
        current_bw_rad: Current loop bandwidth [rad/s]

    Returns:
        Tuple of (Kp, Ki)
    """
    # Symmetric optimal: speed loop bandwidth = current_bw / delta^2
    Ki = current_bw_rad / delta**2

    # Kp based on inertia and torque constant
    KT = 1.5 * npp * KA  # Torque constant
    Kp = Js / npp / KT * delta * Ki

    return Kp, Ki


def auto_design_pi_gains(motor_params, controller_params,
                         current_bw_hz: float = 1000.0,
                         speed_bw_hz: float = 50.0) -> tuple[float, float, float, float]:
    """Auto-design PI gains based on motor parameters.

    Args:
        motor_params: MachineParams instance
        controller_params: ControllerParams instance (will be updated)
        current_bw_hz: Desired current loop bandwidth [Hz]
        speed_bw_hz: Desired speed loop bandwidth [Hz]

    Returns:
        Tuple of (current_Kp, current_Ki, speed_Kp, speed_Ki)
    """
    # Current loop design
    current_Kp, current_Ki = design_current_pi_gains(
        motor_params.R, motor_params.Lq, current_bw_hz
    )

    # Speed loop design
    current_bw_rad = current_bw_hz * 2.0 * np.pi
    delta = (current_bw_rad / (speed_bw_hz * 2.0 * np.pi))**0.5

    speed_Kp, speed_Ki = design_speed_pi_gains(
        motor_params.Js, motor_params.npp, motor_params.KE,
        delta, current_bw_rad
    )

    # Update controller params
    controller_params.current_Kp = current_Kp
    controller_params.current_Ki = current_Ki
    controller_params.speed_Kp = speed_Kp
    controller_params.speed_Ki = speed_Ki

    return current_Kp, current_Ki, speed_Kp, speed_Ki
