"""test for ACMachine class."""

import pytest
import numpy as np
from acmzzz.core.machine_params import MachineParams
from acmzzz.core.machine_state import MachineState
from acmzzz.core.machine import ACMachine

def test_machine_creation():
    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)
    
    assert machine.params.npp == 4
    assert machine.state.KA == 0.095

def test_machine_creation_custom_params():
    params = MachineParams(
        npp=21,
        R=0.1222,
        Ld=0.0011,
        Lq=0.00125,
        KE=0.127,
        Js=0.203,
    )
    state = MachineState()
    machine = ACMachine(params=params, state=state)
    
    assert machine.params.npp == 21
    assert machine.state.KA == 0.127

def test_calculation_torque():
    params = MachineParams(npp=4, KE=0.095)
    state = MachineState()
    machine = ACMachine(params=params, state=state)
    
    machine.state.KA = 0.095
    machine.state.iQ = 10.0

    torque = machine.calculate_torque()

    expected = 1.5 * 4 * 0.095 * 10.0
    assert pytest.approx(torque, rel=1e-6) == expected

def test_torque_zero_current():
    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    machine.state.iQ = 0.0
    torque = machine.calculate_torque()
    assert torque == 0.0

def test_set_voltage_dq():
    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    machine.set_voltage_dq(100.0, 50.0)

    assert machine.state.udq[0] == 100.0
    assert machine.state.udq[1] == 50.0

def test_set_voltage_ab():
    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    machine.set_voltage_ab(80.0, 60.0)
    
    assert machine.state.uab[0] == 80.0
    assert machine.state.uab[1] == 60.0

def test_set_load_torque():
    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    machine.set_load_torque(5.0)
    assert machine.state.Tload == 5.0

def test_get_electrical_angle():
    params = MachineParams(npp=4)
    state = MachineState()
    machine = ACMachine(params=params, state=state)
    
    machine.state.theta_d_mech = 0.5    # rad

    theta_elec = machine.get_electrical_angle()
    assert pytest.approx(theta_elec, rel=1e-6) == 0.5 * 4

def test_get_electrical_speed():
    params = MachineParams(npp=4)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    machine.state.omega_r_mech = 100.0

    omege_elec = machine.get_electrical_speed()
    assert pytest.approx(omege_elec, rel=1e-6) == 100.0 * 4     # 400 rad/s

def test_speed_rpm_conversion():
    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    machine.set_speed_rpm(1000.0)

    expected_omega = 1000.0 * 2.0 * np.pi / 60.0
    assert pytest.approx(machine.state.omega_r_mech, rel=1e-6) == expected_omega
    
    rpm = machine.get_speed_rpm()
    assert pytest.approx(rpm, rel=1e-6) == 1000.0

def test_reset():
    params = MachineParams(KE=0.095)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    machine.state.theta_d_mech = 1.0
    machine.state.omega_r_mech = 100.0
    machine.state.iQ = 5.0
    machine.state.Tload = 10.0

    machine.reset()

    assert machine.state.theta_d_mech == 0.0
    assert machine.state.omega_r_mech == 0.0
    assert machine.state.iQ == 0.0
    assert machine.state.Tload == 0.0
    assert machine.state.KA == 0.095

def test_copy_and_set_state():
    """Test state vector copy and set."""
    params = MachineParams()
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    # Copy state
    x_copy = machine.copy_state()
    assert x_copy.shape == (5,)

    # Modify copy
    x_copy[0] = 1.0
    x_copy[1] = 50.0
    x_copy[4] = 3.0  # iQ

    # Set back
    machine.set_state(x_copy)

    # Check values updated
    assert machine.state.x[0] == 1.0
    assert machine.state.x[1] == 50.0
    assert machine.state.x[4] == 3.0

    # Check outputs updated
    assert machine.state.theta_d_mech == 1.0
    assert machine.state.omega_r_mech == 50.0
    assert machine.state.iQ == 3.0


def test_update_outputs():
    """Test output update after state change."""
    params = MachineParams(npp=4, KE=0.095)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    # Modify state vector directly
    machine.state.x[0] = 0.25  # theta_mech
    machine.state.x[1] = 50.0  # omega_mech
    machine.state.x[2] = 0.095  # KA
    machine.state.x[4] = 5.0  # iQ

    # Update outputs
    machine.update_outputs()

    # Check derived outputs
    assert pytest.approx(machine.state.theta_d, rel=1e-6) == 0.25 * 4
    assert pytest.approx(machine.state.omega_r_elec, rel=1e-6) == 50.0 * 4
    assert pytest.approx(machine.state.Tem, rel=1e-6) == 1.5 * 4 * 0.095 * 5.0


def test_get_info():
    """Test machine info summary."""
    params = MachineParams(npp=4, R=1.1, KE=0.095)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    machine.state.iQ = 2.0
    machine.update_outputs()

    info = machine.get_info()

    assert info['motor_type'] == 'PMSM'
    assert info['pole_pairs'] == 4
    assert info['resistance'] == 1.1


def test_fixture_params(motor_config):
    """Test creating ACMachine from fixture."""
    params = MachineParams(**motor_config)
    state = MachineState()
    machine = ACMachine(params=params, state=state)

    assert machine.params.npp == motor_config['npp']
    assert machine.params.R == motor_config['R']
