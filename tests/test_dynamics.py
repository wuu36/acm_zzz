"""tests for motor dynamics equations."""

import pytest
import numpy as np
from acmzzz.core.machine_params import MachineParams
from acmzzz.core.machine_state import MachineState
from acmzzz.core.machine import ACMachine
from acmzzz.core.dynamics import (
    dynamics_machine,
    dynamics_machine_current_source,
    dynamics_only_mechanical,
)

def test_dynamics_zero_state():
    """test dynamics with zero initial state."""
    params = MachineParams()
    state = MachineState()
    machine  = ACMachine(params=params, state=state)

    x = np.zeros(5)
    x[2] = params.KE    # initial flux

    fx = dynamics_machine(0.0, x, machine)
    
    # with zero currents and zero load, speed should be zero
    assert pytest.approx(fx[1], rel=1e-6) == 0.0
    # position derivation should be zero (no speed)
    assert pytest.approx(fx[0], rel=1e-6) == 0.0

def test_dynamics_with_voltage():
    """test dynamics with voltage input."""
    params = MachineParams(npp=4, R=1.1, Ld=5e-3, Lq=6e-3, KE=0.095, Js=0.0006168)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    # set inital state
    x = np.zeros(5)
    x[2] = 0.095    # KA
    x[3] = 2.0      # iD = 2A
    x[4] = 5.0      # iQ = 5A

    fx = dynamics_machine(0.0, x, machine)

    # check torque and acceleration
    Tem = 1.5 * params.npp * x[2] * x[4]
    expected_accel = Tem / params.Js
    assert pytest.approx(fx[1], rel=1e-6) == expected_accel

def test_dynamics_with_load_torque():
    """test dynamics with load torque."""
    params = MachineParams(npp=4, Js=0.0006168)
    state = MachineState()
    machine = ACMachine(params=params, state=state)
    
    x = np.zeros(5)
    x[2] = 0.095
    x[4] = 5.0

    machine.state.Tload = 1.5   # Load torque

    fx = dynamics_machine(0.0, x, machine)
    
    Tem = 1.5 * params.npp * x[2] * x[4]
    expected_accel = (Tem - 1.5) / params.Js
    assert pytest.approx(fx[1], rel=1e-6) == expected_accel

def test_dynamics_current_dynamics():
    """Test current dynamics with voltage input."""
    params = MachineParams(npp=4, R=1.1, Ld=5e-3, Lq=6e-3, KE=0.095)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x = np.zeros(5)
    x[2] = 0.095
    x[3] = 2.0  # iD
    x[4] = 5.0  # iQ

    machine.state.udq[0] = 50.0
    machine.state.udq[1] = 100.0

    fx = dynamics_machine(0.0, x, machine)

    # Check current derivatives
    # For PMSM: diD/dt = (uD - R*iD + omega*Lq*iQ) / Ld
    omega_syn = 0.0  # zero speed, zero slip
    expected_diD = (50.0 - 1.1 * 2.0 + omega_syn * 6e-3 * 5.0) / 5e-3
    assert pytest.approx(fx[3], rel=1e-6) == expected_diD

def test_dynamics_position_integral():
    """Test position integration."""
    params = MachineParams(npp=4)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x = np.zeros(5)
    x[1] = 100.0  # omega_r_mech = 100 rad/s
    x[2] = 0.095

    fx = dynamics_machine(0.0, x, machine)

    # Position derivative should equal speed (for PMSM with no slip)
    assert pytest.approx(fx[0], rel=1e-6) == 100.0


def test_dynamics_current_source():
    """Test dynamics with current source input."""
    params = MachineParams(npp=4, KE=0.095, Js=0.0006168)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x = np.zeros(5)
    x[1] = 50.0  # Initial speed

    # Current source: iQ = 10A
    fx = dynamics_machine_current_source(0.0, x, machine, iD_source=0.0, iQ_source=10.0)

    # Check torque
    KA = params.KE  # For PMSM with iD=0
    Tem = 1.5 * params.npp * KA * 10.0
    expected_accel = Tem / params.Js
    assert pytest.approx(fx[1], rel=1e-6) == expected_accel

    # Current derivatives should be zero (current source)
    assert pytest.approx(fx[3], rel=1e-6) == 0.0
    assert pytest.approx(fx[4], rel=1e-6) == 0.0


def test_dynamics_only_mechanical():
    """Test pure mechanical dynamics."""
    params = MachineParams(npp=4, Js=0.0006168)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x = np.zeros(5)
    x[1] = 100.0  # omega = 100 rad/s

    machine.state.Tload = 2.0

    # Torque source: Tem = 5 Nm
    fx = dynamics_only_mechanical(0.0, x, machine, Tem_source=5.0)

    # Check acceleration
    expected_accel = (5.0 - 2.0) / params.Js
    assert pytest.approx(fx[1], rel=1e-6) == expected_accel

    # Electromagnetic states unchanged
    assert pytest.approx(fx[2], rel=1e-6) == 0.0
    assert pytest.approx(fx[3], rel=1e-6) == 0.0
    assert pytest.approx(fx[4], rel=1e-6) == 0.0


def test_dynamics_pmsm_flux():
    """Test PMSM flux linkage dynamics."""
    params = MachineParams(npp=4, Ld=5e-3, Lq=6e-3, KE=0.095, Rreq=0.0)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x = np.zeros(5)
    x[3] = 2.0  # iD
    x[4] = 5.0  # iQ

    machine.state.udq[0] = 50.0
    machine.state.udq[1] = 100.0

    fx = dynamics_machine(0.0, x, machine)

    # For PMSM: KA = (Ld-Lq)*iD + KE, dKA/dt = (Ld-Lq)*diD/dt
    # Since Ld < Lq (IPM), KA decreases with positive iD
    # Check that dKA/dt is consistent with diD/dt
    expected_dKA = (params.Ld - params.Lq) * fx[3]
    assert pytest.approx(fx[2], rel=1e-6) == expected_dKA


def test_dynamics_output_shape():
    """Test dynamics output shape."""
    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x = np.zeros(5)
    fx = dynamics_machine(0.0, x, machine)

    assert fx.shape == (5,)
    assert fx.dtype == np.float64


def test_dynamics_fixture_params(motor_config):
    """Test dynamics with fixture parameters."""
    params = MachineParams(**motor_config)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    x = np.zeros(5)
    x[2] = params.KE
    x[4] = 10.0  # iQ = 10A

    fx = dynamics_machine(0.0, x, machine)

    # Verify torque calculation
    Tem = 1.5 * params.npp * params.KE * 10.0
    assert pytest.approx(fx[1] * params.Js, rel=1e-6) == Tem


def test_dynamics_speed_evolution():
    """Test that speed evolves correctly over multiple steps."""
    params = MachineParams(npp=4, KE=0.095, Js=0.0006168)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    # Initial state with torque-producing current
    x = np.zeros(5)
    x[2] = 0.095
    x[4] = 5.0  # iQ produces torque

    # Simple Euler integration for a few steps
    dt = 1e-4
    for _ in range(100):
        fx = dynamics_machine(0.0, x, machine)
        x[1] += fx[1] * dt  # Update speed

    # Speed should have increased (positive torque, no load)
    assert x[1] > 0.0