from dataclasses import dataclass, field
import numpy as np

from ..utils.angle import angle_diff

@dataclass
class SpeedObserver:
    """Speed and position observer for motor control.

    Implements a 3rd or 4th-order position observer that estimates:
        - Position [rad]
        - Speed [rad/s]
        - Disturbance torque [Nm]
        - Torque derivative [Nm/s] (for 4th-order)

    The observer uses angle difference error between measured position
    and estimated position to drive the correction.

    Attributes:
        order: Observer order (2, 3, or 4)
        bandwidth: Observer bandwidth [rad/s]
        npp: Pole pairs
        Js: Inertia [kg.m^2]
        ell1-ell4: Observer gains
        x: Observer state vector (position, speed, disturbance, ...)
        output_error: Angle error for debugging

    Example:
        >>> observer = SpeedObserver(npp=4, Js=0.0006168, bandwidth=100.0)
        >>> observer.update(theta_measured, Tem, dt)
        >>> speed_est = observer.get_speed_estimate()
    """
    
    # motor parameters
    npp: int = 4
    Js: float = 0.0006168

    # observer parameters
    order: int = 3  # 2, 3, or 4
    bandwidth: float = 100.0    # observer bandwidth [rad/s]
    
    # observer gains (computed from bandwidth)
    ell1: float = 0.0
    ell2: float = 0.0
    ell3: float = 0.0
    ell4: float = 0.0
    
    # state vector (size depends on order)
    x: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float64))
    
    # output error (for debugging/analysis)
    output_error: float = 0.0

    # electromagnetic torque input
    Tem: float = 0.0

    def __post_init__(self):
        """Compute observer gains based on bandwidth and order."""
        self._compute_gains()
        self._init_state()

    def _compute_gains(self):
        """Compute observer gains using pole placement."""
        omega_ob = self.bandwidth

        if self.order == 2:
            # 2nd-order position observer
            self.ell1 = 2.0 * omega_ob
            self.ell2 = omega_ob**2 * self.Js / self.npp
            self.ell3 = 0.0
            self.ell4 = 0.0
        elif self.order == 3:
            # 3rd-order position observer (default)
            self.ell1 = 3.0 * omega_ob
            self.ell2 = 3.0 * omega_ob**2
            self.ell3 = omega_ob**3 * self.Js / self.npp
            self.ell4 = 0.0
        elif self.order == 4:
            # 4th-order position observer
            self.ell1 = 4.0 * omega_ob
            self.ell2 = 6.0 * omega_ob**2
            self.ell3 = 4.0 * omega_ob**3 * self.Js / self.npp
            self.ell4 = omega_ob**4

    def _init_state(self):
        """Initialize state vector with correct size."""
        if self.order == 2:
            self.x = np.zeros(3, dtype=np.float64)
        elif self.order == 3:
            self.x = np.zeros(4, dtype=np.float64)
        elif self.order == 4:
            self.x = np.zeros(4, dtype=np.float64)

    def set_position_estimate(self, theta: float):
        self.x[0] = theta

    def set_speed_estimate(self, omega: float):
        self.x[1] = omega

    def get_position_estimate(self) -> float:
        return self.x[0]
    
    def get_speed_estimate(self) -> float:
        return self.x[1]
    
    def get_speed_estimate_mech(self) -> float:
        return self.x[1] / self.npp
    
    def get_speed_estimate_rpm(self) -> float:
        omega_mech = self.x[1] / self.npp
        return omega_mech * 60.0 / (2.0 * np.pi)
    
    def get_disturbance_estimate(self) -> float:
        if self.order >= 3:
            return self.x[2]
        return 0.0
    
    def get_total_torque_estimate(self) -> float:
        return self.Tem + self.get_disturbance_estimate()
    
    def update(self, theta_measured: float, Tem: float, dt: float):
        self.Tem = Tem
    
        # RK4 integration
        k1 = self._dynamics(theta_measured) * dt
        k2 = self._dynamics(theta_measured, self.x + k1 * 0.5) * dt
        k3 = self._dynamics(theta_measured, self.x + k2 * 0.5) * dt
        k4 = self._dynamics(theta_measured, self.x + k3) * dt

        self.x = self.x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    
    def _dynamics(self, theta_measured: float, x: np.ndarray | None = None) -> np.ndarray:
        """Calculate observer dynamics.

        Args:
            theta_measured: Measured position [rad]
            x: State vector (optional, uses self.x if None)

        Returns:
            State derivative dx/dt
        """
        if x is None:
            x = self.x

        # Use provided state for RK4 intermediate steps
        fx = np.zeros_like(x)

        # Output error: difference between measured and estimated position
        self.output_error = angle_diff(theta_measured, x[0])

        # State equations (3rd-order observer)
        # x[0]: position estimate
        # x[1]: speed estimate
        # x[2]: disturbance torque estimate
        # x[3]: disturbance torque derivative (for 4th-order)

        if self.order == 2:
            # 2nd-order: only position and speed
            fx[0] = self.ell1 * self.output_error + x[1]
            fx[1] = self.ell2 * self.output_error + self.Tem * self.npp / self.Js

        elif self.order == 3:
            # 3rd-order: position, speed, disturbance
            fx[0] = self.ell1 * self.output_error + x[1]
            fx[1] = self.ell2 * self.output_error + (self.Tem + x[2]) * self.npp / self.Js
            fx[2] = self.ell3 * self.output_error + x[3]
            fx[3] = 0.0  # x[3] unused for 3rd-order

        elif self.order == 4:
            # 4th-order: position, speed, disturbance, disturbance derivative
            fx[0] = self.ell1 * self.output_error + x[1]
            fx[1] = self.ell2 * self.output_error + (self.Tem + x[2]) * self.npp / self.Js
            fx[2] = self.ell3 * self.output_error + x[3]
            fx[3] = self.ell4 * self.output_error

        return fx
    
    def reset(self):
        """reset observer state to zero."""
        self.x.fill(0.0)
        self.output_error = 0.0
        self.Tem = 0.0

    def set_bandwidth(self, bandwidth: float):
        self.bandwidth = bandwidth
        self._compute_gains()
        
def dynamics_speed_observer(x: np.ndarray, theta_measured: float,
                            Tem: float, ell: tuple, npp: int, Js: float) -> np.ndarray:
    """Standalone speed observer dynamics function.

    This is a functional version matching the original DYNAMICS_SpeedObserver.

    Args:
        x: Observer state vector
        theta_measured: Measured position [rad]
        Tem: Electromagnetic torque [Nm]
        ell: Tuple of (ell1, ell2, ell3, ell4) observer gains
        npp: Pole pairs
        Js: Inertia [kg.m^2]

    Returns:
        State derivative dx/dt
    """
    ell1, ell2, ell3, ell4 = ell

    fx = np.zeros_like(x)
    
    # output error
    output_error = angle_diff(theta_measured, x[0])

    # Output error
    output_error = angle_diff(theta_measured, x[0])

    # Observer dynamics
    fx[0] = ell1 * output_error + x[1]
    fx[1] = ell2 * output_error + (Tem + x[2]) * npp / Js
    fx[2] = ell3 * output_error + x[3]
    fx[3] = ell4 * output_error

    return fx

def compute_observer_gains(order: int, bandwidth: float,
                           npp: int, Js: float) -> tuple[float, float, float, float]:
    """Compute observer gains for given bandwidth.

    Args:
        order: Observer order (2, 3, or 4)
        bandwidth: Observer bandwidth [rad/s]
        npp: Pole pairs
        Js: Inertia [kg.m^2]

    Returns:
        Tuple of (ell1, ell2, ell3, ell4) observer gains
    """
    omega_ob = bandwidth

    if order == 2:
        ell1 = 2.0 * omega_ob
        ell2 = omega_ob**2 * Js / npp
        ell3 = 0.0
        ell4 = 0.0
    elif order == 3:
        ell1 = 3.0 * omega_ob
        ell2 = 3.0 * omega_ob**2
        ell3 = omega_ob**3 * Js / npp
        ell4 = 0.0
    elif order == 4:
        ell1 = 4.0 * omega_ob
        ell2 = 6.0 * omega_ob**2
        ell3 = 4.0 * omega_ob**3 * Js / npp
        ell4 = omega_ob**4
    else:
        raise ValueError(f"invalid observer order: {order}")
    
    return ell1, ell2, ell3, ell4

    
