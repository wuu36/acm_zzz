"""test for MotorController class."""

import pytest
import numpy as np
from acmzzz.control.controller import MotorController
from acmzzz.control.controller_params import ControllerParams, FOCConfig
from acmzzz.control.regulator import PIRegulator
from acmzzz.core.machine_params import MachineParams

def test_controller_creation():
    """test MotorController creation with default params."""
    ctrl = MotorController()

    assert ctrl.motor_params is not None
    assert ctrl.controller_params is not None
    assert ctrl.foc_config is not None
    assert ctrl.speed_reg is not None
    assert ctrl.id_reg is not None
    assert ctrl.iq_reg is not None

def test_controller_with_custome_params():
    """test MotorController with custom parameters."""
    motor = MachineParams(npp=4, R=1.1, Ld=5e-3, Lq=6e-3, KE=0.095)
    ctrl_params = ControllerParams(
        CL_TS=1e-4,
        VL_TS=5e-4,
        speed_Kp=0.05,
        speed_Ki=40.0
    )
    foc_config = FOCConfig(control_mode='speed')

    ctrl = MotorController(
        motor_params=motor,
        controller_params=ctrl_params,
        foc_config=foc_config,
    )

    assert ctrl.motor_params.npp == 4
    assert ctrl.controller_params.CL_TS == 1e-4
    assert ctrl.speed_reg is not None
    assert ctrl.speed_reg.Kp == 0.05

def test_controller_speed_command():
    """Test speed command setting."""
    ctrl = MotorController()

    ctrl.set_speed_command(1000.0)
    assert ctrl.cmd_rpm == 1000.0

    omega_elec = ctrl.get_speed_command_elec()
    expected = 1000.0 / 60.0 * 2.0 * np.pi * ctrl.motor_params.npp
    assert pytest.approx(omega_elec, rel=1e-6) == expected


def test_controller_execute():
    """Test controller execute step."""
    ctrl = MotorController(controller_params=ControllerParams(VL_ceiling=1))

    ctrl.set_speed_command(100.0)
    ctrl.execute(omega_r_elec=0.0, id_=0.0, iq=0.0)

    # After execution, voltage commands should be set
    ud, uq = ctrl.get_voltage_commands_dq()
    assert uq > 0.0  # Positive iq for acceleration

def test_controller_time_update():
    """Test timebase update."""
    ctrl = MotorController(controller_params=ControllerParams(CL_TS=1e-4))

    initial_time = ctrl.timebase
    ctrl.update_time()

    assert pytest.approx(ctrl.timebase, rel=1e-6) == initial_time + 1e-4


def test_controller_feedback_update():
    """Test feedback signal update."""
    ctrl = MotorController()

    ctrl.update_feedback(theta_d=1.0, omega_r_elec=100.0, id_=5.0, iq=10.0)

    assert ctrl.theta_d == 1.0
    assert ctrl.omega_r_elec == 100.0
    assert ctrl.idq[0] == 5.0
    assert ctrl.idq[1] == 10.0

    # KA and Tem should be computed
    assert ctrl.KA > 0.0
    assert ctrl.Tem > 0.0


def test_controller_reset():
    """Test controller reset."""
    ctrl = MotorController(controller_params=ControllerParams(VL_ceiling=1))

    # Run some steps
    ctrl.set_speed_command(100.0)
    for _ in range(10):
        ctrl.execute(omega_r_elec=0.0, id_=0.0, iq=0.0)

    # Reset
    ctrl.reset()

    assert ctrl.timebase == 0.0
    assert ctrl.cmd_rpm == 0.0
    assert ctrl.speed_reg is not None
    assert ctrl.id_reg is not None
    assert ctrl.iq_reg is not None
    assert ctrl.speed_reg.Out == 0.0
    assert ctrl.id_reg.Out == 0.0
    assert ctrl.iq_reg.Out == 0.0


def test_controller_voltage_ab():
    """Test inverse Park transform for voltage commands."""
    ctrl = MotorController()

    # Set voltage commands directly
    ctrl.cmd_udq[0] = 100.0  # ud
    ctrl.cmd_udq[1] = 50.0   # uq
    ctrl.theta_d = 0.0       # Zero angle

    u_alpha, u_beta = ctrl.get_voltage_commands_ab()

    # At theta=0: u_alpha = ud, u_beta = uq
    assert pytest.approx(u_alpha, rel=1e-6) == 100.0
    assert pytest.approx(u_beta, rel=1e-6) == 50.0


def test_controller_voltage_ab_90deg():
    """Test inverse Park at 90 degrees."""
    ctrl = MotorController()

    ctrl.cmd_udq[0] = 100.0  # ud
    ctrl.cmd_udq[1] = 50.0   # uq
    ctrl.theta_d = np.pi / 2  # 90 degrees

    u_alpha, u_beta = ctrl.get_voltage_commands_ab()

    # At theta=90deg: u_alpha = -uq, u_beta = ud
    assert pytest.approx(u_alpha, rel=1e-6) == -50.0
    assert pytest.approx(u_beta, rel=1e-6) == 100.0


def test_controller_set_pi_gains():
    """Test PI gain update."""
    ctrl = MotorController()

    ctrl.set_pi_gains(speed_Kp=0.1, speed_Ki=50.0, current_Kp=10.0, current_Ki=1000.0)

    assert ctrl.speed_reg is not None
    assert ctrl.id_reg is not None
    assert ctrl.iq_reg is not None
    assert ctrl.speed_reg.Kp == 0.1
    assert ctrl.speed_reg.Ki == 50.0
    assert ctrl.id_reg.Kp == 10.0
    assert ctrl.iq_reg.Kp == 10.0


def test_controller_auto_design_gains():
    """Test auto PI gain design."""
    motor = MachineParams(npp=4, R=1.1, Lq=6e-3, KE=0.095, Js=0.0006168)
    ctrl = MotorController(motor_params=motor)

    ctrl.auto_design_gains(current_bw_hz=1000.0, speed_bw_hz=50.0)

    # Gains should be updated
    assert ctrl.speed_reg is not None
    assert ctrl.id_reg is not None
    assert ctrl.speed_reg.Kp > 0.0
    assert ctrl.speed_reg.Ki > 0.0
    assert ctrl.id_reg.Kp > 0.0
    assert ctrl.id_reg.Ki > 0.0


def test_controller_observer_gains_2nd_order():
    """Test 2nd-order observer gains."""
    motor = MachineParams(npp=4, Js=0.0006168)
    ctrl = MotorController(motor_params=motor, observer_order=2, observer_bw=100.0)

    # 2nd-order: ell1 = 2*omega, ell2 = omega^2 * Js/npp
    assert pytest.approx(ctrl.ell1, rel=1e-6) == 200.0
    assert ctrl.ell2 > 0.0


def test_controller_observer_gains_3rd_order():
    """Test 3rd-order observer gains."""
    motor = MachineParams(npp=4, Js=0.0006168)
    ctrl = MotorController(motor_params=motor, observer_order=3, observer_bw=100.0)

    # 3rd-order: ell1 = 3*omega, ell2 = 3*omega^2, ell3 = omega^3 * Js/npp
    assert pytest.approx(ctrl.ell1, rel=1e-6) == 300.0
    assert pytest.approx(ctrl.ell2, rel=1e-6) == 3 * 100**2
    assert ctrl.ell3 > 0.0


def test_controller_observer_gains_4th_order():
    """Test 4th-order observer gains."""
    motor = MachineParams(npp=4, Js=0.0006168)
    ctrl = MotorController(motor_params=motor, observer_order=4, observer_bw=100.0)

    # 4th-order gains
    assert pytest.approx(ctrl.ell1, rel=1e-6) == 400.0
    assert pytest.approx(ctrl.ell2, rel=1e-6) == 6 * 100**2
    assert ctrl.ell3 > 0.0
    assert pytest.approx(ctrl.ell4, rel=1e-6) == 100**4


def test_controller_get_info():
    """Test controller info dictionary."""
    ctrl = MotorController()

    ctrl.set_speed_command(500.0)
    ctrl.execute(omega_r_elec=0.0, id_=0.0, iq=0.0)

    info = ctrl.get_info()

    assert 'time' in info
    assert 'speed_cmd_rpm' in info
    assert info['speed_cmd_rpm'] == 500.0


def test_controller_torque_command():
    """Test torque command setting."""
    motor = MachineParams(npp=4, KE=0.095)
    ctrl = MotorController(motor_params=motor)

    ctrl.KA = motor.KE  # Initialize KA
    ctrl.set_torque_command(1.0)  # 1 N·m

    # iq should be computed from torque
    expected_iq = 1.0 / (1.5 * motor.npp * motor.KE)
    assert pytest.approx(ctrl.cmd_idq[1], rel=1e-6) == expected_iq


def test_controller_ka_calculation():
    """Test active flux calculation."""
    motor = MachineParams(npp=4, Ld=5e-3, Lq=6e-3, KE=0.095)
    ctrl = MotorController(motor_params=motor)

    # Update feedback with id current
    ctrl.update_feedback(theta_d=0.0, omega_r_elec=0.0, id_=2.0, iq=5.0)

    # KA = (Ld - Lq) * id + KE
    expected_KA = (motor.Ld - motor.Lq) * 2.0 + motor.KE
    assert pytest.approx(ctrl.KA, rel=1e-6) == expected_KA


def test_controller_tem_calculation():
    """Test torque calculation."""
    motor = MachineParams(npp=4, KE=0.095)
    ctrl = MotorController(motor_params=motor)

    ctrl.update_feedback(theta_d=0.0, omega_r_elec=0.0, id_=0.0, iq=10.0)

    # Tem = 1.5 * npp * iq * KA
    expected_Tem = 1.5 * motor.npp * 10.0 * motor.KE
    assert pytest.approx(ctrl.Tem, rel=1e-6) == expected_Tem


def test_controller_multiple_execute():
    """Test multiple execute steps."""
    ctrl = MotorController(
        controller_params=ControllerParams(VL_ceiling=1, CL_TS=1e-4)
    )

    ctrl.set_speed_command(100.0)

    times = []
    voltages = []

    for _ in range(10):
        ctrl.execute(omega_r_elec=0.0, id_=0.0, iq=0.0)
        times.append(ctrl.timebase)
        voltages.append(ctrl.cmd_udq[1])

    # Time should increase linearly
    assert times[-1] > times[0]
    assert pytest.approx(times[1] - times[0], rel=1e-6) == 1e-4


def test_controller_speed_tracking():
    """Test speed tracking over multiple steps."""
    ctrl = MotorController(
        motor_params=MachineParams(npp=4),
        controller_params=ControllerParams(
            speed_Kp=0.01,
            speed_Ki=5.0,
            speed_limit=100.0,
            VL_ceiling=1
        )
    )

    ctrl.set_speed_command(100.0)

    # Simulate speed approaching target
    for i in range(20):
        omega = (100.0 * i / 20) / 60.0 * 2.0 * np.pi * 4
        ctrl.execute(omega_r_elec=omega, id_=0.0, iq=0.0)

    # iq should be positive throughout
    iq_cmds = [ctrl.cmd_idq[1] for _ in range(20)]
    assert all(iq >= 0 for iq in iq_cmds)


def test_controller_with_regulator_instances():
    """Test controller with custom regulator instances."""
    speed_reg = PIRegulator(Kp=0.2, Ki=100.0, OutLimit=20.0)
    id_reg = PIRegulator(Kp=15.0, Ki=3000.0, OutLimit=400.0)
    iq_reg = PIRegulator(Kp=15.0, Ki=3000.0, OutLimit=400.0)

    ctrl = MotorController(
        speed_reg=speed_reg,
        id_reg=id_reg,
        iq_reg=iq_reg
    )

    assert ctrl.speed_reg is not None
    assert ctrl.id_reg is not None
    assert ctrl.speed_reg.Kp == 0.2
    assert ctrl.id_reg.Kp == 15.0


def test_controller_fixture(motor_config, controller_config):
    """Test MotorController with fixtures."""
    motor = MachineParams(**motor_config)
    ctrl_params = ControllerParams(**controller_config)

    ctrl = MotorController(motor_params=motor, controller_params=ctrl_params)

    assert ctrl.motor_params.npp == motor_config['npp']
    assert ctrl.controller_params.CL_TS == controller_config['CL_TS']


def test_controller_velocity_counter_integration():
    """Test velocity loop counter integration."""
    ctrl = MotorController(
        controller_params=ControllerParams(VL_ceiling=5)
    )

    ctrl.set_speed_command(100.0)

    # Execute 5 times
    for i in range(5):
        ctrl.execute(omega_r_elec=0.0, id_=0.0, iq=0.0)

    # Counter should be reset to 0
    assert ctrl.controller_params.velocity_loop_counter == 0