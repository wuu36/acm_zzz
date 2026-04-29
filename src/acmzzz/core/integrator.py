"""RK4 (4th-order Runge-Kutta) numerical integrator.

This module implements the classic RK4 method for numerical integration
of ordinary differential equations (ODEs). RK4 is widely used in motor
simulation due to its balance of accuracy and computational efficiency.

The RK4 formula:
    x(t+h) = x(t) + (k1 + 2*k2 + 2*k3 + k4) / 6

where:
    k1 = h * f(t, x)
    k2 = h * f(t + h/2, x + k1/2)
    k3 = h * f(t + h/2, x + k2/2)
    k4 = h * f(t + h, x + k3)
"""

import numpy as np
from typing import Callable, Union, Optional
from .machine import ACMachine


def rk4_step(
    dynamics_func: Callable[[float, np.ndarray, ACMachine], np.ndarray],
    t: float,
    x: np.ndarray,
    dt: float,
    machine: ACMachine
) -> np.ndarray:
    """Perform one RK4 integration step.

    Computes the next state vector using the 4th-order Runge-Kutta method.

    Args:
        dynamics_func: Function that computes dx/dt, signature f(t, x, machine) -> dx
        t: Current time [s]
        x: Current state vector
        dt: Time step (integration interval) [s]
        machine: ACMachine instance (passed to dynamics function)

    Returns:
        New state vector x(t + dt)

    Example:
        >>> from acmsimpy.core.dynamics import dynamics_machine
        >>> x_new = rk4_step(dynamics_machine, 0.0, x, 1e-4, machine)
    """
    # Stage 1: k1 = h * f(t, x)
    k1 = dynamics_func(t, x, machine) * dt
    xk = x + k1 * 0.5

    # Stage 2: k2 = h * f(t + h/2, x + k1/2)
    k2 = dynamics_func(t + dt * 0.5, xk, machine) * dt
    xk = x + k2 * 0.5

    # Stage 3: k3 = h * f(t + h/2, x + k2/2)
    k3 = dynamics_func(t + dt * 0.5, xk, machine) * dt
    xk = x + k3

    # Stage 4: k4 = h * f(t + h, x + k3)
    k4 = dynamics_func(t + dt, xk, machine) * dt

    # Combine: x_new = x + (k1 + 2*k2 + 2*k3 + k4) / 6
    x_new = x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0

    return x_new


def rk4_integrate(
    dynamics_func: Callable[[float, np.ndarray, ACMachine], np.ndarray],
    t0: float,
    x0: np.ndarray,
    dt: float,
    n_steps: int,
    machine: ACMachine,
    callback: Optional[Callable[[float, np.ndarray], None]] = None
) -> tuple[np.ndarray, np.ndarray]:
    """Perform multiple RK4 integration steps.

    Integrates the system from t0 to t0 + n_steps * dt, recording all
    intermediate states.

    Args:
        dynamics_func: Function that computes dx/dt
        t0: Initial time [s]
        x0: Initial state vector
        dt: Time step [s]
        n_steps: Number of integration steps
        machine: ACMachine instance
        callback: Optional callback function called at each step f(t, x)

    Returns:
        Tuple of (times, states):
            - times: Array of time values [t0, t0+dt, ..., t0+n_steps*dt]
            - states: Array of state vectors at each time, shape (n_steps+1, n_states)

    Example:
        >>> times, states = rk4_integrate(dynamics_machine, 0.0, x0, 1e-4, 1000, machine)
        >>> # states[0] = initial state, states[1000] = final state
    """
    n_states = len(x0)

    # Preallocate arrays
    times = np.zeros(n_steps + 1)
    states = np.zeros((n_steps + 1, n_states))

    # Initialize
    t = t0
    x = x0.copy()
    times[0] = t
    states[0] = x.copy()

    # Integration loop
    for i in range(n_steps):
        # RK4 step
        x = rk4_step(dynamics_func, t, x, dt, machine)

        # Update time
        t = t0 + (i + 1) * dt

        # Record state
        times[i + 1] = t
        states[i + 1] = x.copy()

        # Callback if provided
        if callback is not None:
            callback(t, x)

    return times, states


def rk4_integrate_with_controller(
    dynamics_func: Callable[[float, np.ndarray, ACMachine], np.ndarray],
    controller_func: Callable[[float, np.ndarray, ACMachine], None],
    t0: float,
    x0: np.ndarray,
    dt: float,
    n_steps: int,
    machine: ACMachine,
    controller_downsample: int = 1
) -> tuple[np.ndarray, np.ndarray]:
    """RK4 integration with periodic controller execution.

    Simulates the machine with a controller that runs at a lower rate
    than the simulation. This matches typical DSP implementation where
    the current loop runs faster than the speed loop.

    Args:
        dynamics_func: Machine dynamics function
        controller_func: Controller function that sets machine.udq
        t0: Initial time [s]
        x0: Initial state vector
        dt: Simulation time step [s]
        n_steps: Number of simulation steps
        machine: ACMachine instance
        controller_downsample: Controller runs every N simulation steps

    Returns:
        Tuple of (times, states)

    Example:
        >>> # Controller runs every 5 simulation steps
        >>> times, states = rk4_integrate_with_controller(
        >>>     dynamics_machine, foc_controller, 0.0, x0, 1e-4, 1000, machine, 5
        >>> )
    """
    n_states = len(x0)

    times = np.zeros(n_steps + 1)
    states = np.zeros((n_steps + 1, n_states))

    t = t0
    x = x0.copy()
    times[0] = t
    states[0] = x.copy()

    # Update machine state from initial x
    machine.state.x = x.copy()
    machine.update_outputs()

    for i in range(n_steps):
        # Run controller at downsampled rate
        if i % controller_downsample == 0:
            controller_func(t, x, machine)

        # RK4 integration step
        x = rk4_step(dynamics_func, t, x, dt, machine)

        # Update time
        t = t0 + (i + 1) * dt

        # Update machine state for next controller call
        machine.state.x = x.copy()
        machine.update_outputs()

        # Record
        times[i + 1] = t
        states[i + 1] = x.copy()

    return times, states


def rk4_step_simple(
    dynamics_func: Callable[[float, np.ndarray], np.ndarray],
    t: float,
    x: np.ndarray,
    dt: float
) -> np.ndarray:
    """Simplified RK4 step without machine parameter.

    For use with simple dynamics functions that don't need machine context.

    Args:
        dynamics_func: Function f(t, x) -> dx/dt
        t: Current time
        x: Current state
        dt: Time step

    Returns:
        New state vector
    """
    k1 = dynamics_func(t, x) * dt
    k2 = dynamics_func(t + dt * 0.5, x + k1 * 0.5) * dt
    k3 = dynamics_func(t + dt * 0.5, x + k2 * 0.5) * dt
    k4 = dynamics_func(t + dt, x + k3) * dt

    return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


# Euler method (for comparison/testing)
def euler_step(
    dynamics_func: Callable[[float, np.ndarray, ACMachine], np.ndarray],
    t: float,
    x: np.ndarray,
    dt: float,
    machine: ACMachine
) -> np.ndarray:
    """Simple Euler integration step (1st order).

    Less accurate than RK4 but faster. Useful for testing or
    when accuracy is not critical.

    Args:
        dynamics_func: Dynamics function f(t, x, machine) -> dx/dt
        t: Current time
        x: Current state
        dt: Time step
        machine: ACMachine instance

    Returns:
        New state vector: x + dt * f(t, x)
    """
    dx = dynamics_func(t, x, machine)
    return x + dt * dx