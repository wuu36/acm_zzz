"""tests for MachineState class."""

import pytest
import numpy as np
from acmzzz.core.machine_state import MachineState
from acmzzz.core.machine_params import MachineParams


def test_default_state():
    """test default MachineState initialization."""
    state = MachineState()
    assert state.NS == 5
    assert state.x.shape == (5,)
    assert np.all(state.x == 0.0)
    assert state.Tload == 0.0
    assert state.Tem == 0.0

def test_state_vector_dtype():
    """test state vector is float64."""
    state = MachineState()
    assert state.x.dtype == np.float64
    assert state.uab.dtype == np.float64
    assert state.udq.dtype == np.float64

def test_set_initial_flux():
    """test setting initial flux linkage."""
    state = MachineState()
    state.set_initial_flux(0.095)
    assert state.KA == 0.095
    assert state.x[2] == 0.095

def test_sync_from_state_vector():
    """test syncing derived outputs from state vector."""
    params = MachineParams(npp=4)
    state = MachineState()

    # set state vector values
    state.x[0] = 0.5    # theta_d_mech
    state.x[1] = 100.0  # omega_r_mech
    state.x[2] = 0.095  # KA
    state.x[3] = 2.0    # iD
    state.x[4] = 3.0    # iQ

    state.sync_from_state_vector(params)

    # check derived values
    assert state.theta_d_mech == 0.5
    assert state.omega_r_mech == 100.0
    assert state.KA == 0.095
    assert state.iD == 2.0
    assert state.iQ == 3.0

    # check electrical quantities
    assert state.theta_d == 0.5 * 4
    assert state.omega_r_elec == 100.0 * 4

def test_torque_calculation():
    """test electromagnetic torque calculation."""
    params = MachineParams(npp=4)
    state = MachineState()
    
    state.x[2] = 0.095      # KA
    state.x[4] = 10.0       # iQ = 10A

    state.sync_from_state_vector(params)

    # Tem = 1.5 * npp * KA * iQ
    expected_torque = 1.5 * 4 * 0.095 * 10.0
    assert pytest.approx(state.Tem, rel=1e-6) == expected_torque

def test_trigonometric_values():
    """test cos/sin calculation from theta_d."""
    params = MachineParams(npp=4)
    state = MachineState()

    # set theta_d_mech = pi/8, so theta_d = pi/2
    state.x[0] = np.pi / 8
    state.sync_from_state_vector(params)
    
    assert pytest.approx(state.theta_d, rel=1e-6) == np.pi / 2
    assert pytest.approx(state.cosT, rel=1e-6) == 0.0
    assert pytest.approx(state.sinT, rel=1e-6) == 1.0

def test_sync_to_state_vector():
    """test syncing state vector from derived outputs."""
    state = MachineState()

    # set derived values directly
    state.theta_d_mech = 1.0
    state.omega_r_mech = 50.0
    state.KA = 0.08
    state.iD = 1.5
    state.iQ = 2.5

    state.sync_to_state_vector()

    # check state vector
    assert state.x[0] == 1.0
    assert state.x[1] == 50.0
    assert state.x[2] == 0.08
    assert state.x[3] == 1.5
    assert state.x[4] == 2.5

def test_reset():
    """test reset functionality."""
    state = MachineState()
    state.x[0] = 1.0
    state.x[4] = 5.0
    state.Tload = 10.0
    state.Tem = 50.0

    state.reset()

    assert np.all(state.x == 0.0)
    assert state.Tload == 0.0
    assert state.Tem == 0.0
    assert state.cosT == 1.0
    assert state.sinT == 0.0

def test_input_arrays():
    """test input voltage arrays."""
    state = MachineState()
    
    # set input voltages
    state.udq[0] = 100.0    # uD
    state.udq[1] = 50.0     # uQ
    state.uab[0] = 80.0     # ualpha
    state.uab[1] = 40.0     # ubeta

    assert state.udq[0] == 100.0
    assert state.udq[1] == 50.0
    assert state.uab[0] == 80.0
    assert state.uab[1] == 40.0

def test_custom_state_init():
    """test custom state initialization."""
    initial_x = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
    state = MachineState(x=initial_x)
    
    assert state.x[0] == 1.0
    assert state.x[4] == 5.0