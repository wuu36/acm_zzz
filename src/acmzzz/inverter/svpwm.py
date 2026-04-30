

from dataclasses import dataclass
import numpy as np

# constants
SQRT3 = 1.7320508075688772  # sqrt(3)
SQRT3_OVER_2 = 0.8660254037844386  # sqrt(3)/2

@dataclass
class SVPWM:
    # DC bus voltage
    Vdc: float = 300.0

    # duty cycle limits
    max_duty: float = 0.95
    min_duty: float = 0.05

    # input voltages (alpha-beta frame)
    Ualpha: float = 0.0
    Ubeta: float = 0.0
    Unot: float = 0.0   # zero sequence voltage (optional)

    # outputs
    sector: int = 0
    Ta: float = 0.5
    Tb: float = 0.5
    Tc: float = 0.5

    def set_voltage(self, Ualpha: float, Ubeta: float, Unot: float = 0.0):
        """Set voltage commands in alpha-beta frame.

        Args:
            Ualpha: Alpha-axis voltage [V]
            Ubeta: Beta-axis voltage [V]
            Unot: Zero sequence voltage [V] (optional)
        """
        self.Ualpha = Ualpha
        self.Ubeta = Ubeta
        self.Unot = Unot

    def calculate(self):
        """Calculate SVPWM duty cycles.

        Executes the complete SVPWM algorithm:
            1. Normalize voltages by Vdc
            2. Determine sector
            3. Calculate duty cycles
            4. Apply duty cycle limits
        """
        # Normalization (converts [V] to [s], duty cycle proportion)
        one_over_Vdc = 1.0 / self.Vdc
        Talpha = self.Ualpha * one_over_Vdc
        Tbeta = self.Ubeta * one_over_Vdc
        Tz = self.Unot * one_over_Vdc

        # Sector determination variables (60-degree sectors)
        A = Tbeta  # 0-degree reference
        C = SQRT3 * Talpha - Tbeta
        B = -SQRT3 * Talpha - Tbeta

        # Determine sector (0-6)
        self.sector = 0
        if A > 0:
            self.sector = 1
        if C > 0:
            self.sector += 2
        if B > 0:
            self.sector += 4

        # X, Y, Z calculations
        X = SQRT3 * Tbeta
        Y = 1.5 * Talpha + SQRT3_OVER_2 * Tbeta
        Z = -1.5 * Talpha + SQRT3_OVER_2 * Tbeta

        # Calculate duty cycles based on sector
        if self.sector == 0:
            # Zero voltage vector
            self.Ta = 0.5
            self.Tb = 0.5
            self.Tc = 0.5
        elif self.sector == 1:
            # Sector 1: t1=Z, t2=Y
            t1 = Z
            t2 = Y
            self.Tb = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
            self.Ta = self.Tb + t1
            self.Tc = self.Ta + t2
        elif self.sector == 2:
            # Sector 2: t1=Y, t2=-X
            t1 = Y
            t2 = -X
            self.Ta = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
            self.Tc = self.Ta + t1
            self.Tb = self.Tc + t2
        elif self.sector == 3:
            # Sector 3: t1=-Z, t2=X
            t1 = -Z
            t2 = X
            self.Ta = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
            self.Tb = self.Ta + t1
            self.Tc = self.Tb + t2
        elif self.sector == 4:
            # Sector 4: t1=-X, t2=Z
            t1 = -X
            t2 = Z
            self.Tc = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
            self.Tb = self.Tc + t1
            self.Ta = self.Tb + t2
        elif self.sector == 5:
            # Sector 5: t1=X, t2=-Y
            t1 = X
            t2 = -Y
            self.Tb = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
            self.Tc = self.Tb + t1
            self.Ta = self.Tc + t2
        elif self.sector == 6:
            # Sector 6: t1=-Y, t2=-Z
            t1 = -Y
            t2 = -Z
            self.Tc = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
            self.Ta = self.Tc + t1
            self.Tb = self.Ta + t2

        # Invert for active-high logic
        self.Ta = 1.0 - self.Ta
        self.Tb = 1.0 - self.Tb
        self.Tc = 1.0 - self.Tc

        # Apply duty cycle limits
        self.Ta = np.clip(self.Ta, self.min_duty, self.max_duty)
        self.Tb = np.clip(self.Tb, self.min_duty, self.max_duty)
        self.Tc = np.clip(self.Tc, self.min_duty, self.max_duty)

    def get_duty_cycles(self) -> tuple[float, float, float]:
        """Get calculated duty cycles.

        Returns:
            Tuple of (Ta, Tb, Tc) duty cycles
        """
        return self.Ta, self.Tb, self.Tc

    def get_sector(self) -> int:
        """Get current sector.

        Returns:
            Sector number (0-6)
        """
        return self.sector

    def reset(self):
        """Reset to zero voltage state."""
        self.Ualpha = 0.0
        self.Ubeta = 0.0
        self.Unot = 0.0
        self.sector = 0
        self.Ta = 0.5
        self.Tb = 0.5
        self.Tc = 0.5

def determine_sector(Ualpha: float, Ubeta: float, Vdc: float) -> int:
    """Determine SVPWM sector from voltage commands.

    This is a standalone function for sector determination only.

    Args:
        Ualpha: Alpha-axis voltage [V]
        Ubeta: Beta-axis voltage [V]
        Vdc: DC bus voltage [V]

    Returns:
        Sector number (0-6)

    Example:
        >>> sector = determine_sector(100.0, 50.0, 300.0)
        >>> print(f"Sector: {sector}")
    """
    one_over_Vdc = 1.0 / Vdc
    Talpha = Ualpha * one_over_Vdc
    Tbeta = Ubeta * one_over_Vdc

    # Sector determination variables
    A = Tbeta
    C = SQRT3 * Talpha - Tbeta
    B = -SQRT3 * Talpha - Tbeta

    # Determine sector
    sector = 0
    if A > 0:
        sector = 1
    if C > 0:
        sector += 2
    if B > 0:
        sector += 4

    return sector


def svpwm_duty_cycle(Ualpha: float, Ubeta: float, Vdc: float,
                     max_duty: float = 0.95, min_duty: float = 0.05,
                     Unot: float = 0.0) -> tuple[int, float, float, float]:
    """Calculate SVPWM duty cycles (standalone function).

    This is a functional version matching the original SVGEN_DQ pattern.

    Args:
        Ualpha: Alpha-axis voltage [V]
        Ubeta: Beta-axis voltage [V]
        Vdc: DC bus voltage [V]
        max_duty: Maximum duty cycle limit
        min_duty: Minimum duty cycle limit
        Unot: Zero sequence voltage [V]

    Returns:
        Tuple of (sector, Ta, Tb, Tc)

    Example:
        >>> sector, Ta, Tb, Tc = svpwm_duty_cycle(100.0, 50.0, 300.0)
    """
    # Normalization
    one_over_Vdc = 1.0 / Vdc
    Talpha = Ualpha * one_over_Vdc
    Tbeta = Ubeta * one_over_Vdc
    Tz = Unot * one_over_Vdc

    # Sector determination
    A = Tbeta
    C = SQRT3 * Talpha - Tbeta
    B = -SQRT3 * Talpha - Tbeta

    sector = 0
    if A > 0:
        sector = 1
    if C > 0:
        sector += 2
    if B > 0:
        sector += 4

    # X, Y, Z calculations
    X = SQRT3 * Tbeta
    Y = 1.5 * Talpha + SQRT3_OVER_2 * Tbeta
    Z = -1.5 * Talpha + SQRT3_OVER_2 * Tbeta

    # Calculate duty cycles
    if sector == 0:
        Ta, Tb, Tc = 0.5, 0.5, 0.5
    elif sector == 1:
        t1, t2 = Z, Y
        Tb = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
        Ta = Tb + t1
        Tc = Ta + t2
    elif sector == 2:
        t1, t2 = Y, -X
        Ta = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
        Tc = Ta + t1
        Tb = Tc + t2
    elif sector == 3:
        t1, t2 = -Z, X
        Ta = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
        Tb = Ta + t1
        Tc = Tb + t2
    elif sector == 4:
        t1, t2 = -X, Z
        Tc = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
        Tb = Tc + t1
        Ta = Tb + t2
    elif sector == 5:
        t1, t2 = X, -Y
        Tb = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
        Tc = Tb + t1
        Ta = Tc + t2
    elif sector == 6:
        t1, t2 = -Y, -Z
        Tc = (1.0 - t1 - t2) * 0.5 + Tz * 0.5
        Ta = Tc + t1
        Tb = Ta + t2

    # Invert for active-high logic
    Ta = 1.0 - Ta
    Tb = 1.0 - Tb
    Tc = 1.0 - Tc

    # Apply limits
    Ta = np.clip(Ta, min_duty, max_duty)
    Tb = np.clip(Tb, min_duty, max_duty)
    Tc = np.clip(Tc, min_duty, max_duty)

    return sector, Ta, Tb, Tc


def inverse_clarke_svpwm(Ualpha: float, Ubeta: float) -> tuple[float, float, float]:
    """Inverse Clarke transform for SVPWM reference.

    Calculates equivalent phase voltages from alpha-beta voltages.
    This is for reference/verification only - actual SVPWM uses
    different duty cycle calculation.

    Args:
        Ualpha: Alpha-axis voltage [V]
        Ubeta: Beta-axis voltage [V]

    Returns:
        Tuple of (Ua, Ub, Uc) phase voltages [V]
    """
    # Inverse Clarke (amplitude invariant)
    Ua = Ualpha
    Ub = -0.5 * Ualpha + SQRT3_OVER_2 * Ubeta
    Uc = -0.5 * Ualpha - SQRT3_OVER_2 * Ubeta

    return Ua, Ub, Uc