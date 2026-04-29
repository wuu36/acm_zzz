"""Tests for RK4 integrator."""

import pytest
import numpy as np
from acmzzz.core.machine_params import MachineParams
from acmzzz.core.machine_state import MachineState
from acmzzz.core.machine import ACMachine
from acmzzz.core.dynamics import dynamics_machine
from acmzzz.core.integrator import (
    rk4_step,
    rk4_integrate,
    rk4_integrate_with_controller,
    rk4_step_simple,
    euler_step,
)


def test_rk4_step_constant_derivative():
    """Test RK4 with constant derivative (linear growth)."""
    def const_deriv(t, x, m=None):
        return np.array([1.0, 0.0])

    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x0 = np.array([0.0, 0.0])
    x1 = rk4_step(const_deriv, 0.0, x0, 1.0, machine)

    # With constant derivative 1, x[0] should increase by exactly dt=1.0
    assert pytest.approx(x1[0], rel=1e-10) == 1.0
    assert pytest.approx(x1[1], rel=1e-10) == 0.0


def test_rk4_step_exponential_decay():
    """Test RK4 with exponential decay."""
    def exp_decay(t, x, m=None):
        return np.array([-x[0]])

    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x0 = np.array([1.0])
    dt = 0.01

    # Decay for many steps
    x = x0.copy()
    for _ in range(1000):
        x = rk4_step(exp_decay, 0.0, x, dt, machine)

    # Should decay to nearly zero
    assert abs(x[0]) < 0.001


def test_rk4_step_simple():
    """Test simplified RK4 without machine."""
    def linear(t, x):
        return np.array([2.0, 3.0])

    x0 = np.array([0.0, 0.0])
    x1 = rk4_step_simple(linear, 0.0, x0, 1.0)

    # x[0] should increase by 2.0, x[1] by 3.0
    assert pytest.approx(x1[0], rel=1e-10) == 2.0
    assert pytest.approx(x1[1], rel=1e-10) == 3.0


def test_rk4_integrate_shape():
    """Test rk4_integrate output shape."""
    def const_deriv(t, x, m=None):
        return np.ones(5)

    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x0 = np.zeros(5)
    n_steps = 100

    times, states = rk4_integrate(const_deriv, 0.0, x0, 0.01, n_steps, machine)

    # Check shapes
    assert times.shape == (n_steps + 1,)
    assert states.shape == (n_steps + 1, 5)

    # Check times
    assert pytest.approx(times[0], rel=1e-10) == 0.0
    assert pytest.approx(times[-1], rel=1e-10) == n_steps * 0.01


def test_rk4_integrate_linear_growth():
    """Test rk4_integrate with linear growth."""
    def linear(t, x, m=None):
        return np.array([10.0, 5.0, 0.0, 0.0, 0.0])

    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x0 = np.zeros(5)
    dt = 0.1
    n_steps = 10

    times, states = rk4_integrate(linear, 0.0, x0, dt, n_steps, machine)

    # After 10 steps of 0.1s with derivative 10 and 5
    # x[0] should be 10*0.1*10 = 10.0
    # x[1] should be 5*0.1*10 = 5.0
    assert pytest.approx(states[-1, 0], rel=1e-10) == 10.0
    assert pytest.approx(states[-1, 1], rel=1e-10) == 5.0


def test_rk4_with_machine():
    """Test RK4 step with actual machine dynamics."""
    params = MachineParams(npp=4, KE=0.095, Js=0.0006168)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    # Set initial state
    x0 = np.zeros(5)
    x0[2] = 0.095  # KA
    x0[4] = 5.0    # iQ = 5A

    # One RK4 step
    dt = 1e-4
    x1 = rk4_step(dynamics_machine, 0.0, x0, dt, machine)

    # Speed should increase (positive torque)
    assert x1[1] > 0.0
    # Position should increase
    assert x1[0] > 0.0


def test_rk4_integrate_motor_acceleration():
    """Test full motor acceleration simulation."""
    params = MachineParams(npp=4, KE=0.095, Js=0.0006168)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    # Initial state with current source
    x0 = np.zeros(5)
    x0[2] = 0.095  # KA
    x0[4] = 10.0   # iQ = 10A

    # Set voltage to maintain current (simplified)
    machine.state.udq[1] = 50.0

    dt = 1e-4
    n_steps = 1000

    times, states = rk4_integrate(dynamics_machine, 0.0, x0, dt, n_steps, machine)

    # Final speed should be significant
    final_speed = states[-1, 1]
    assert final_speed > 10.0  # At least some acceleration


def test_euler_step_comparison():
    """Compare Euler and RK4 accuracy."""
    # Exponential decay: dx = -x
    def exp_decay(t, x, m=None):
        return -x

    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x0 = np.array([1.0])
    dt = 0.1

    # True solution: x(t) = e^(-t)
    # After dt=0.1: x = e^(-0.1) ≈ 0.904837
    true_value = np.exp(-dt)

    # Euler result
    x_euler = euler_step(exp_decay, 0.0, x0, dt, machine)

    # RK4 result
    x_rk4 = rk4_step(exp_decay, 0.0, x0, dt, machine)

    # RK4 should be more accurate
    euler_error = abs(x_euler[0] - true_value)
    rk4_error = abs(x_rk4[0] - true_value)

    assert rk4_error < euler_error  # RK4 more accurate
    assert rk4_error < 1e-6  # RK4 very accurate


def test_rk4_callback():
    """Test callback functionality in rk4_integrate."""
    def const_deriv(t, x, m=None):
        return np.ones(3)

    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x0 = np.zeros(3)

    # Track callback calls
    callback_times = []
    def callback(t, x):
        callback_times.append(t)

    times, states = rk4_integrate(const_deriv, 0.0, x0, 0.1, 10, machine, callback)

    # Callback should be called for each step
    assert len(callback_times) == 10


def test_rk4_integrate_with_controller():
    """Test integration with controller."""
    params = MachineParams(npp=4, KE=0.095, Js=0.0006168)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x0 = np.zeros(5)
    x0[2] = 0.095

    # Simple controller: set voltage to produce torque
    def simple_controller(t, x, m):
        m.state.udq[0] = 0.0
        m.state.udq[1] = 100.0  # uQ voltage

    dt = 1e-4
    n_steps = 100
    downsample = 5  # Controller runs every 5 steps

    times, states = rk4_integrate_with_controller(
        dynamics_machine, simple_controller, 0.0, x0, dt, n_steps, machine, downsample
    )

    # Check output shape
    assert times.shape == (n_steps + 1,)
    assert states.shape == (n_steps + 1, 5)


def test_rk4_oscillation():
    """Test RK4 with harmonic oscillator."""
    # Simple harmonic oscillator: dx1/dt = x2, dx2/dt = -x1
    def oscillator(t, x, m=None):
        return np.array([x[1], -x[0]])

    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    # Initial: x1=1, x2=0 (starts at max displacement)
    x0 = np.array([1.0, 0.0])
    dt = 0.01

    # Integrate for one period: T = 2*pi
    n_steps = int(2 * np.pi / dt)

    x = x0.copy()
    for _ in range(n_steps):
        x = rk4_step(oscillator, 0.0, x, dt, machine)

    # After one period, should return to near initial state
    # RK4 maintains energy well, but not perfectly
    assert pytest.approx(x[0], rel=1e-3) == 1.0
    assert abs(x[1]) < 0.01  # Velocity near zero (less strict)


def test_rk4_fixture_params(motor_config):
    """Test RK4 with fixture parameters."""
    params = MachineParams(**motor_config)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x0 = np.zeros(5)
    x0[2] = params.KE

    # One step
    x1 = rk4_step(dynamics_machine, 0.0, x0, 1e-4, machine)

    # Should produce valid output
    assert x1.shape == (5,)
    assert np.all(np.isfinite(x1))