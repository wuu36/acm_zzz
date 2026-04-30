import numpy as np

def angle_normalize(angle: float, range_type: str= '0_to_2pi') -> float:
    """Normalize angle to specified range.

    Args:
        angle: Input angle [rad]
        range_type: Range type, either '0_to_2pi' or '-pi_to_pi'

    Returns:
        Normalized angle [rad]

    Example:
        >>> angle_normalize(7.0)
        0.7168...  # 7 - 2π
        >>> angle_normalize(-0.5, range_type='-pi_to_pi')
        -0.5
        >>> angle_normalize(4.0, range_type='-pi_to_pi')
        -2.2831...  # 4 - 2π
    """
    if range_type == '0_to_2pi':
        # normalize to [0, 2pi)
        _, normalized = divmod(angle, 2.0 * np.pi)
        return normalized
    elif range_type == '-pi_to_pi':
        # normalize to [-pi, pi)
        normalized = angle % (2.0 * np.pi)
        if normalized > np.pi:
            normalized -= 2.0 * np.pi
        return normalized
    else:
        raise ValueError(f"unknown range_type: {range_type}")
    
def angle_diff(a: float, b: float) -> float:
    """Calculate minimum angular difference between two angles.

    Returns the signed difference a - b wrapped to [-π, π].
    This is useful for position error calculation in motor control,
    where the error should be the minimum angular distance regardless
    of which direction the motor needs to rotate.

    Args:
        a: First angle [rad]
        b: Second angle [rad]

    Returns:
        Angular difference a - b [rad], in range [-π, π]

    Example:
        >>> angle_diff(0.1, 6.2)  # Near 2π wraparound
        ≈ 0.18  # Wrapped to small positive
        >>> angle_diff(0, 3*np.pi/4)  # 135° apart
        -3*np.pi/4 ≈ -2.36  # Direct distance (no shorter path)
    """
    # normalize both angles to [0, 2pi)
    _, a_norm = divmod(a, 2.0 * np.pi)
    _, b_norm = divmod(b, 2.0 * np.pi)
    
    # calculate direct difference
    d1 = a_norm - b_norm

    # calculate wrapped difference (opposite direction)
    if d1 > 0:
        d2 = a_norm - (b_norm + 2.0 * np.pi)
    else:
        d2 = (2.0 * np.pi + a_norm) - b_norm
    
    # return the smaller magnitude difference
    if np.abs(d1) < np.abs(d2):
        return d1
    else:
        return d2
    
def angle_wrap(angle: float, center: float = 0.0) -> float:
    """Wrap angle around a center point.

    Useful for wrapping position estimates around a reference,
    such as wrapping the observer position around the measured position.

    Args:
        angle: Input angle [rad]
        center: Center point for wrapping [rad]

    Returns:
        Wrapped angle [rad]

    Example:
        >>> angle_wrap(7.0, 0.0)  # Wrap around 0
        0.7168...  # 7 - 2π
    """
    # normalize to [0, 2pi) relative to center
    shifted = angle - center
    _, wrapped = divmod(shifted, 2.0 * np.pi)
    return wrapped + center

def angle_increment(prev_angle: float, delta: float) -> float:
    """Increment angle by delta, maintaining continuity.

    Useful for tracking continuous rotation without sudden jumps.

    Args:
        prev_angle: Previous angle [rad]
        delta: Angular increment [rad]

    Returns:
        New angle [rad]

    Example:
        >>> angle_increment(6.0, 0.5)  # Cross 2π boundary
        6.5  # No wrap, continuous
    """
    return prev_angle + delta

def angle_to_degrees(angle: float) -> float:
    """Convert angle from radians to degrees.

    Args:
        angle: Angle [rad]

    Returns:
        Angle [deg]
    """
    return angle * 180.0 / np.pi

def angle_to_radians(angle: float) -> float:
    """Convert angle from degrees to radians.

    Args:
        angle: Angle [deg]

    Returns:
        Angle [rad]
    """
    return angle * np.pi / 180.0


def speed_to_rpm(omega: float, npp: int = 1) -> float:
    """Convert mechanical angular speed to RPM.

    Args:
        omega: Mechanical angular speed [rad/s]
        npp: Pole pairs (optional, for electrical speed conversion)

    Returns:
        Speed [rpm]
    """
    return omega * 60.0 / (2.0 * np.pi)


def rpm_to_speed(rpm: float, npp: int = 1) -> float:
    """Convert RPM to mechanical angular speed.

    Args:
        rpm: Speed [rpm]
        npp: Pole pairs (optional, for electrical speed conversion)

    Returns:
        Mechanical angular speed [rad/s]
    """
    return rpm * 2.0 * np.pi / 60.0


def electrical_angle(theta_mech: float, npp: int) -> float:
    """Convert mechanical angle to electrical angle.

    Args:
        theta_mech: Mechanical angle [rad]
        npp: Pole pairs

    Returns:
        Electrical angle [rad]
    """
    return theta_mech * npp


def mechanical_angle(theta_elec: float, npp: int) -> float:
    """Convert electrical angle to mechanical angle.

    Args:
        theta_elec: Electrical angle [rad]
        npp: Pole pairs

    Returns:
        Mechanical angle [rad]
    """
    return theta_elec / npp
