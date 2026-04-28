from dataclasses import dataclass, field
import numpy as np

@dataclass
class MachineState:
    """AC motor state variables.

    The motor state is represented by a 5-element state vector for RK4 integration:
    - x[0]: theta_d_mech (mechanical rotor angle) [rad]
    - x[1]: omega_r_mech (mechanical rotor speed) [rad/s]
    - x[2]: KA (active flux linkage) [Wb]
    - x[3]: iD (d-axis current) [A]
    - x[4]: iQ (q-axis current) [A]

    Additional derived outputs are computed from the state vector.

    Attributes:
        x: State vector for RK4 integration (5 elements)
        uab: Two-phase voltage input (alpha, beta) [V]
        udq: dq-frame voltage input [V]
        TLoad: Load torque [Nm]

    Example:
        >>> state = MachineState()
        >>> state.x[2] = 0.095  # Set initial flux
        >>> state.iQ = 1.0      # Set q-axis current
    """

    # state vector for RK4 integration
    NS: int = 5     # number of states (constant)
    x: np.ndarray = field(default_factory=lambda: np.zeros(5, dtype=np.float64))
    
    # input variables
    uab: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    udq: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    Tload: float = 0.0 # load torque [Nm]
    
    # output variables (derived form state)
    omega_slip: float = 0.0     # slip angular speed [rad/s]
    omega_r_elec: float = 0.0   # electrical rotor speed [rad/s]
    omega_r_mech: float = 0.0   # mechanical rotor speed [rad/s]
    omega_syn: float = 0.0      # synchronous speed [rad/s]
    theta_d: float = 0.0        # electrical rotor angle [rad]
    theta_d_mech: float = 0.0   # mechanical rotor angle [rad]
    KA: float = 0.0             # active flux linkage [Wb]
    iD: float = 0.0             # d-axis current [A]
    iQ: float = 0.0             # q-axis current [A]
    Tem: float = 0.0            # electromagnetic torque [Nm]
    
    # trigonometric values for park transform
    cosT: float = 1.0           # cos(theta_d)
    sinT: float = 0.0           # sin(theta_d)

    def __post_init__(self):
        """initialize state array if needed."""
        # ensure x is a numpy array with correct dtype
        if not isinstance(self.x, np.ndarray):
            self.x = np.zeros(self.NS, dtype=np.float64)
        elif self.x.dtype != np.float64:
            self.x = self.x.astype(np.float64)
            
        # ensure input arrays are correct type
        if not isinstance(self.uab, np.ndarray):
            self.uab = np.zeros(2, dtype=np.float64)
        if not isinstance(self.udq, np.ndarray):
            self.udq = np.zeros(2, dtype=np.float64)
    
    def sync_from_state_vector(self, params):
        """sync derived outputs from state vector x.
        called after RK4 integration to update all derived variables
        args:
            params: MachineParams instance with motor parameters
        """
        # state vector unpacking
        self.theta_d_mech = self.x[0]
        self.omega_r_mech = self.x[1]
        self.KA = self.x[2]
        self.iD = self.x[3]
        self.iQ = self.x[4]
        
        # derived electrical quantities
        self.theta_d = self.theta_d_mech * params.npp
        self.omega_r_elec = self.omega_r_mech * params.npp
        self.omega_syn = self.omega_r_elec + self.omega_slip

        # trigonometric values
        self.cosT = np.cos(self.theta_d)
        self.sinT = np.sin(self.theta_d)

        # electromagnetic torque
        # Tem = 1.5 * npp * KA * iQ (clarke transformation gain = 1.5)
        CLARKE_TRANS_TORQUE_GAIN = 1.5
        self.Tem = CLARKE_TRANS_TORQUE_GAIN * params.npp * self.KA * self.iQ
        
    def sync_to_state_vector(self):
        """sync state vector x from derived outputs.
        called before RK4 integration if state was modified directly.
        """
        self.x[0] = self.theta_d_mech
        self.x[1] = self.omega_r_mech
        self.x[2] = self.KA
        self.x[3] = self.iD
        self.x[4] = self.iQ

    def set_initial_flux(self, KE: float):
        """set initial active flux linkage
        Args:
            KE: initial flux linkage value [Wb]
        """
        self.KA = KE
        self.x[2] = KE

    def reset(self):
        """reset all state variables to zero."""
        self.x = np.zeros(self.NS, dtype=np.float64)
        self.uab = np.zeros(2, dtype=np.float64)
        self.udq = np.zeros(2, dtype=np.float64)
        self.Tload = 0.0
        self.omega_slip = 0.0
        self.omega_r_elec = 0.0
        self.omega_r_mech = 0.0
        self.omega_syn = 0.0
        self.theta_d = 0.0
        self.theta_d_mech = 0.0
        self.KA = 0.0
        self.iD = 0.0
        self.iQ = 0.0
        self.Tem = 0.0
        self.cosT = 1.0
        self.sinT = 0.0

        

