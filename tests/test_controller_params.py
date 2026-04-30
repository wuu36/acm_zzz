"""Tests for controller parameters."""

import pytest
import numpy as np
from acmzzz.control.controller_params import (
    ControllerParams,
    FOCConfig,
    design_current_pi_gains,
    design_speed_pi_gains,
    auto_design_pi_gains,
)
from acmzzz.core.machine_params import MachineParams


def test_controller_params_creation():
    """Test ControllerParams creation."""
    params = ControllerParams()
    assert params.CL_TS == 1e-4
    assert params.VL_TS == 5e-4
    assert params.VL_ceiling == 5


def test_controller_params_custom():
    """Test ControllerParams with custom values."""
    params = ControllerParams(
        CL_TS=2e-4,
        VL_TS=1e-3,
        VL_ceiling=5,
        speed_Kp=0.05,
        speed_Ki=40.0,
        current_Kp=8.0,
        current_Ki=2000.0,
    )
    assert params.CL_TS == 2e-4
    assert params.speed_Kp == 0.05


def test_controller_params_timing_correction():
    """Test VL_TS auto-correction."""
    params = ControllerParams(CL_TS=1e-4, VL_TS=3e-4, VL_ceiling=5)
    # VL_TS should be corrected to CL_TS * VL_ceiling = 5e-4
    assert pytest.approx(params.VL_TS, rel=1e-6) == 5e-4


def test_controller_params_loop_rates():
    """Test loop rate calculations."""
    params = ControllerParams(CL_TS=1e-4, VL_ceiling=5)

    current_rate = params.get_current_loop_rate_hz()
    assert pytest.approx(current_rate, rel=1e-6) == 10000.0  # 10 kHz

    speed_rate = params.get_speed_loop_rate_hz()
    assert pytest.approx(speed_rate, rel=1e-6) == 2000.0  # 2 kHz


def test_controller_params_velocity_counter():
    """Test velocity loop counter."""
    params = ControllerParams(VL_ceiling=5)

    # Should return False for first 4 increments
    for i in range(4):
        execute_speed = params.increment_velocity_counter()
        assert execute_speed is False

    # 5th increment should trigger speed loop execution
    execute_speed = params.increment_velocity_counter()
    assert execute_speed is True

    # Counter should reset to 0
    assert params.velocity_loop_counter == 0


def test_controller_params_reset_counter():
    """Test velocity counter reset."""
    params = ControllerParams()
    params.velocity_loop_counter = 10
    params.reset_velocity_counter()
    assert params.velocity_loop_counter == 0


def test_foc_config_creation():
    """Test FOCConfig creation."""
    config = FOCConfig()
    assert config.control_mode == 'speed'
    assert config.id_command_strategy == 'mtpa'


def test_foc_config_mtpa():
    """Test FOCConfig MTPA strategy."""
    config = FOCConfig(id_command_strategy='mtpa')
    motor_params = MachineParams()

    id_cmd = config.get_id_command(motor_params, 100.0, 300.0)
    assert pytest.approx(id_cmd, rel=1e-6) == 0.0


def test_foc_config_fixed():
    """Test FOCConfig fixed strategy."""
    config = FOCConfig(id_command_strategy='fixed', id_command_value=-2.0)
    motor_params = MachineParams()

    id_cmd = config.get_id_command(motor_params, 100.0, 300.0)
    assert pytest.approx(id_cmd, rel=1e-6) == -2.0


def test_design_current_pi_gains():
    """Test current loop PI gain design."""
    R = 1.1  # Ohm
    L = 6e-3  # H
    bandwidth_hz = 1000.0

    Kp, Ki = design_current_pi_gains(R, L, bandwidth_hz)

    # Kp = bandwidth * 2*pi * L
    expected_Kp = bandwidth_hz * 2 * np.pi * L
    assert pytest.approx(Kp, rel=1e-6) == expected_Kp

    # Ki = R / L
    expected_Ki = R / L
    assert pytest.approx(Ki, rel=1e-6) == expected_Ki


def test_design_speed_pi_gains():
    """Test speed loop PI gain design."""
    Js = 0.0006168
    npp = 4
    KA = 0.095
    delta = 6.5
    current_bw_rad = 1000 * 2 * np.pi

    Kp, Ki = design_speed_pi_gains(Js, npp, KA, delta, current_bw_rad)

    # Ki = current_bw / delta^2
    expected_Ki = current_bw_rad / delta**2
    assert pytest.approx(Ki, rel=1e-6) == expected_Ki

    # Kp based on inertia
    assert Kp > 0.0


def test_auto_design_pi_gains():
    """Test auto PI gain design."""
    motor_params = MachineParams()
    controller_params = ControllerParams()

    current_Kp, current_Ki, speed_Kp, speed_Ki = auto_design_pi_gains(
        motor_params, controller_params, current_bw_hz=1000.0, speed_bw_hz=50.0
    )

    # Check gains are updated
    assert pytest.approx(controller_params.current_Kp, rel=1e-6) == current_Kp
    assert pytest.approx(controller_params.current_Ki, rel=1e-6) == current_Ki
    assert pytest.approx(controller_params.speed_Kp, rel=1e-6) == speed_Kp
    assert pytest.approx(controller_params.speed_Ki, rel=1e-6) == speed_Ki

    # Check gains are reasonable
    assert current_Kp > 0.0
    assert current_Ki > 0.0
    assert speed_Kp > 0.0
    assert speed_Ki > 0.0


def test_controller_params_fixture(controller_config):
    """Test ControllerParams from fixture."""
    params = ControllerParams(**controller_config)
    assert params.CL_TS == controller_config['CL_TS']
    assert params.VL_TS == controller_config['VL_TS']


def test_design_current_pi_gains_edge_cases():
    """Test current PI design with edge cases."""
    # Small bandwidth
    Kp, Ki = design_current_pi_gains(1.0, 1e-3, 100.0)
    assert Kp > 0.0

    # Large bandwidth
    Kp, Ki = design_current_pi_gains(1.0, 1e-3, 5000.0)
    assert Kp > 0.0


def test_foc_config_modes():
    """Test FOCConfig control modes."""
    # Speed mode
    config = FOCConfig(control_mode='speed')
    assert config.control_mode == 'speed'

    # Torque mode
    config = FOCConfig(control_mode='torque')
    assert config.control_mode == 'torque'

    # Position mode
    config = FOCConfig(control_mode='position')
    assert config.control_mode == 'position'