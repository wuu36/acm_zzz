from typing import TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    import control

try:
    import control
    HAS_CONTROL_LIB = True
except ImportError:
    HAS_CONTROL_LIB = False
    control = None  # 显式设置为None，满足类型检查器

def get_current_pi(R: float, L: float, bandwidth_hz: float) -> tuple[float, float]:
    """Design current loop PI gains using zero-pole cancellation.

    The current loop is designed using the zero-pole cancellation method:
        Kp = bandwidth * 2π * L
        Ki = R / L

    This places the PI zero at the motor pole location, resulting in
    a first-order closed-loop response with bandwidth = Kp/L.

    Args:
        R: Stator resistance [Ohm]
        L: Inductance [H]
        bandwidth_hz: Desired current loop bandwidth [Hz]

    Returns:
        Tuple of (Kp, Ki)

    Example:
        >>> Kp, Ki = get_current_pi(1.1, 6e-3, 1000.0)
        >>> # Kp ≈ 37.7, Ki ≈ 183.3
    """
    Kp = bandwidth_hz * 2.0 * np.pi * L
    Ki = R / L
    return Kp, Ki

def get_speed_pi(Js: float, npp: int, KA: float, delta: float,
                 current_bw_rad: float) -> tuple[float, float]:
    """Design speed loop PI gains using symmetric optimal method.

    The speed loop uses the symmetric optimal (SO) method which ensures
    good damping and robustness:
        Ki = current_bw / delta^2
        Kp = Js/npp / (1.5*npp*KA) * delta * Ki

    The parameter delta (typically 3-10) determines the ratio between
    current loop and speed loop bandwidth.

    Args:
        Js: Total inertia [kg.m^2]
        npp: Pole pairs
        KA: Active flux [Wb]
        delta: Symmetric optimal parameter (typically 3-10)
        current_bw_rad: Current loop bandwidth [rad/s]

    Returns:
        Tuple of (Kp, Ki)

    Example:
        >>> Kp, Ki = get_speed_pi(0.0006168, 4, 0.095, 6.5, 1000*2*np.pi)
    """
    # Symmetric optimal: speed Ki
    Ki = current_bw_rad / delta**2

    # Torque constant
    KT = 1.5 * npp * KA

    # Speed Kp based on inertia
    Kp = Js / npp / KT * delta * Ki

    return Kp, Ki

def tune_for_desired_speed_bandwidth(
        R: float,
        L: float,
        Js: float,
        npp: int,
        KA: float,
        desired_vl_bw_hz: float,
        delta: float = 6.5,
        initial_cl_bw_hz: float = 1000.0,
        cl_bw_step: float = 100.0,
        max_iterations: int = 20
) -> tuple[float, float, float, float, float, float]:
    """Iteratively tune PI gains for desired speed loop bandwidth.

    Adjusts current loop bandwidth to achieve the desired speed loop
    bandwidth through iteration.

    Args:
        R: Stator resistance [Ohm]
        L: Inductance [H]
        Js: Inertia [kg.m^2]
        npp: Pole pairs
        KA: Active flux [Wb]
        desired_vl_bw_hz: Desired speed loop bandwidth [Hz]
        delta: Symmetric optimal parameter
        initial_cl_bw_hz: Initial current loop bandwidth [Hz]
        cl_bw_step: Step size for bandwidth adjustment [Hz]
        max_iterations: Maximum iteration count

    Returns:
        Tuple of (current_Kp, current_Ki, speed_Kp, speed_Ki,
                  achieved_cl_bw_hz, achieved_vl_bw_hz)
    """
    cl_bw_hz = initial_cl_bw_hz
    vl_bw_hz = 0.0

    for iteration in range(max_iterations):
        # Design current loop PI
        current_Kp, current_Ki = get_current_pi(R, L, cl_bw_hz)
        current_bw_rad = cl_bw_hz * 2.0 * np.pi

        # Design speed loop PI
        speed_Kp, speed_Ki = get_speed_pi(Js, npp, KA, delta, current_bw_rad)

        # Estimate achieved speed bandwidth
        # For SO method: vl_bw ≈ current_bw / delta^2
        vl_bw_hz = cl_bw_hz / delta**2

        # Check convergence
        if abs(vl_bw_hz - desired_vl_bw_hz) <= 10.0:  # Within 10 Hz tolerance
            break

        # Adjust current bandwidth
        if vl_bw_hz > desired_vl_bw_hz:
            cl_bw_hz -= cl_bw_step
            if cl_bw_hz <= 0:
                raise ValueError("Current bandwidth became negative. Reduce step size.")
        else:
            cl_bw_hz += cl_bw_step

    return current_Kp, current_Ki, speed_Kp, speed_Ki, cl_bw_hz, vl_bw_hz


def analyze_current_loop_tf(R: float, L: float, Kp: float, Ki: float):
    """Analyze current loop transfer function (requires control library).

    Args:
        R: Stator resistance [Ohm]
        L: Inductance [H]
        Kp: Current PI proportional gain
        Ki: Current PI integral gain

    Returns:
        Dictionary with transfer function analysis (if control lib available)
    """
    if not HAS_CONTROL_LIB:
        return {"error": "python-control library not installed"}

    assert control is not None  # Type guard for static analyzer

    # Motor plant: 1/(R + sL)
    # PI controller: Kp + Ki/s

    # Closed-loop (with zero-pole cancellation design)
    # After cancellation: G_closed = 1 / (s*L/Kp + 1)
    G_closed = control.tf([1], [L/Kp, 1])

    # Calculate bandwidth using frequency_response (new API)
    response = control.frequency_response(G_closed)
    mag = response.magnitude[0]
    phase = response.phase[0]
    omega = response.frequency
    # Bandwidth at |G| = 0.707
    bw_idx = np.argmin(np.abs(mag - 0.707))
    bw_hz = omega[bw_idx] / (2.0 * np.pi)

    return {
        "closed_loop_tf": G_closed,
        "bandwidth_hz": bw_hz,
        "magnitude": mag,
        "phase": phase,
        "omega": omega
    }

def analyze_speed_loop_tf(Js: float, npp: int, KA: float,
                          L: float, # 添加电感参数
                          current_Kp: float, current_bw_rad: float,
                          speed_Kp: float, speed_Ki: float):
    """Analyze speed loop transfer function (requires control library).

    Args:
        Js: Inertia [kg.m^2]
        npp: Pole pairs
        KA: Active flux [Wb]
        current_Kp: Current PI Kp
        current_bw_rad: Current loop bandwidth [rad/s]
        speed_Kp: Speed PI Kp
        speed_Ki: Speed PI Ki

    Returns:
        Dictionary with transfer function analysis (if control lib available)
    """
    if not HAS_CONTROL_LIB:
        return {"error": "python-control library not installed"}

    assert control is not None  # Type guard for static analyzer

    # Torque constant
    KT = 1.5 * npp * KA

    # Current loop (closed): G_i = 1/(s*L/Kp + 1)
    G_current = control.tf([1], [L/current_Kp, 1])

    # Mechanical system: omega = KT * i / Js
    G_mech = control.tf([KT / Js], [1, 0])

    # Speed PI controller: Kp + Ki/s = (Kp*s + Ki)/s
    G_speed_pi = control.tf([speed_Kp, speed_Ki], [1, 0])

    # Open-loop transfer function
    G_open = G_mech * G_current * G_speed_pi

    # Closed-loop transfer function
    G_closed = G_open / (1 + G_open)

    # Calculate bandwidth using frequency_response (new API)
    response = control.frequency_response(G_closed)
    mag = response.magnitude[0]
    phase = response.phase[0]
    omega = response.frequency
    bw_idx = np.argmin(np.abs(mag - 0.707))
    bw_hz = omega[bw_idx] / (2.0 * np.pi)

    return {
        "open_loop_tf": G_open,
        "closed_loop_tf": G_closed,
        "bandwidth_hz": bw_hz,
        "magnitude": mag,
        "phase": phase,
        "omega": omega
    }

def print_pi_parameters(current_Kp: float, current_Ki: float,
                         speed_Kp: float, speed_Ki: float,
                         cl_bw_hz: float | None = None, vl_bw_hz: float | None = None):
    """Print formatted PI parameter summary.

    Args:
        current_Kp: Current loop Kp
        current_Ki: Current loop Ki
        speed_Kp: Speed loop Kp
        speed_Ki: Speed loop Ki
        cl_bw_hz: Current loop bandwidth [Hz] (optional)
        vl_bw_hz: Speed loop bandwidth [Hz] (optional)
    """
    print("=" * 50)
    print("PI Controller Parameters")
    print("=" * 50)

    print("\nCurrent Loop PI:")
    print(f"  Kp = {current_Kp:.4f}")
    print(f"  Ki = {current_Ki:.4f}")
    if cl_bw_hz:
        print(f"  Bandwidth = {cl_bw_hz:.1f} Hz")

    print("\nSpeed Loop PI:")
    print(f"  Kp = {speed_Kp:.6f}")
    print(f"  Ki = {speed_Ki:.4f}")
    if vl_bw_hz:
        print(f"  Bandwidth = {vl_bw_hz:.1f} Hz")

    print("=" * 50)


class PITuner:
    """PI parameter tuner for motor control.

    Provides a convenient interface for designing PI controller parameters
    based on motor parameters and desired bandwidth.

    Attributes:
        R: Stator resistance [Ohm]
        L: Inductance [H]
        Js: Inertia [kg.m^2]
        npp: Pole pairs
        KA: Active flux [Wb]
        delta: Symmetric optimal parameter

    Example:
        >>> tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)
        >>> tuner.tune_for_speed_bandwidth(50.0)  # 50 Hz speed bandwidth
        >>> tuner.print_parameters()
    """

    def __init__(self, R: float, L: float, Js: float, npp: int, KA: float,
                 delta: float = 6.5):
        """Initialize PI tuner with motor parameters.

        Args:
            R: Stator resistance [Ohm]
            L: Inductance [H]
            Js: Inertia [kg.m^2]
            npp: Pole pairs
            KA: Active flux [Wb]
            delta: Symmetric optimal parameter (default 6.5)
        """
        self.R = R
        self.L = L
        self.Js = Js
        self.npp = npp
        self.KA = KA
        self.delta = delta

        # Designed parameters
        self.current_Kp = 0.0
        self.current_Ki = 0.0
        self.speed_Kp = 0.0
        self.speed_Ki = 0.0
        self.cl_bw_hz = 0.0
        self.vl_bw_hz = 0.0

    def tune_current_loop(self, bandwidth_hz: float):
        """Design current loop PI for specified bandwidth.

        Args:
            bandwidth_hz: Desired current loop bandwidth [Hz]
        """
        self.current_Kp, self.current_Ki = get_current_pi(
            self.R, self.L, bandwidth_hz
        )
        self.cl_bw_hz = bandwidth_hz

    def tune_speed_loop(self, current_bw_hz: float | None = None):
        """Design speed loop PI based on current loop bandwidth.

        Args:
            current_bw_hz: Current loop bandwidth [Hz] (uses self.cl_bw_hz if None)
        """
        if current_bw_hz is None:
            current_bw_hz = self.cl_bw_hz

        current_bw_rad = current_bw_hz * 2.0 * np.pi

        self.speed_Kp, self.speed_Ki = get_speed_pi(
            self.Js, self.npp, self.KA, self.delta, current_bw_rad
        )

        # Estimated speed bandwidth
        self.vl_bw_hz = current_bw_hz / self.delta**2

    def tune_for_speed_bandwidth(self, desired_vl_bw_hz: float):
        """Iteratively tune for desired speed loop bandwidth.

        Args:
            desired_vl_bw_hz: Desired speed loop bandwidth [Hz]
        """
        (current_Kp, current_Ki, speed_Kp, speed_Ki,
         cl_bw_hz, vl_bw_hz) = tune_for_desired_speed_bandwidth(
            self.R, self.L, self.Js, self.npp, self.KA,
            desired_vl_bw_hz, self.delta
        )

        self.current_Kp = current_Kp
        self.current_Ki = current_Ki
        self.speed_Kp = speed_Kp
        self.speed_Ki = speed_Ki
        self.cl_bw_hz = cl_bw_hz
        self.vl_bw_hz = vl_bw_hz

    def get_parameters(self) -> dict:
        """Get designed PI parameters.

        Returns:
            Dictionary with PI parameters and bandwidths
        """
        return {
            "current_Kp": self.current_Kp,
            "current_Ki": self.current_Ki,
            "speed_Kp": self.speed_Kp,
            "speed_Ki": self.speed_Ki,
            "current_bw_hz": self.cl_bw_hz,
            "speed_bw_hz": self.vl_bw_hz
        }

    def print_parameters(self):
        """Print formatted parameter summary."""
        print_pi_parameters(
            self.current_Kp, self.current_Ki,
            self.speed_Kp, self.speed_Ki,
            self.cl_bw_hz, self.vl_bw_hz
        )

    def analyze(self) -> dict:
        """Perform transfer function analysis (if control lib available).

        Returns:
            Dictionary with analysis results
        """
        if not HAS_CONTROL_LIB:
            return {"error": "python-control library not installed"}

        current_analysis = analyze_current_loop_tf(
            self.R, self.L, self.current_Kp, self.current_Ki
        )

        current_bw_rad = self.cl_bw_hz * 2.0 * np.pi
        speed_analysis = analyze_speed_loop_tf(
            self.Js, self.npp, self.KA,
            self.L,
            self.current_Kp, current_bw_rad,
            self.speed_Kp, self.speed_Ki
        )

        return {
            "current_loop": current_analysis,
            "speed_loop": speed_analysis
        }