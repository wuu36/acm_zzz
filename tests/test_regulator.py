"""Tests for PI and PID regulators."""

import pytest
from acmzzz.control.regulator import PIRegulator, PIDRegulator, incremental_pi


def test_pi_creation():
    """Test PI regulator creation."""
    reg = PIRegulator(Kp=1.0, Ki=10.0, OutLimit=100.0)
    assert reg.Kp == 1.0
    assert reg.Ki == 10.0
    assert reg.OutLimit == 100.0
    assert reg.Out == 0.0

def test_pi_zero_error():
    """Test PI with zero error."""
    reg = PIRegulator(Kp=1.0, Ki=10.0, OutLimit=100.0)
    reg.Ref = 0.0
    reg.Fbk = 0.0

    reg.execute()

    # Zero error should produce zero output change
    assert pytest.approx(reg.Out, rel=1e-6) == 0.0


def test_pi_step_response():
    """Test PI integral accumulation."""
    reg = PIRegulator(Kp=0.0, Ki=5.0, OutLimit=100.0)
    reg.Ref = 10.0  # Setpoint = 10
    reg.Fbk = 0.0   # Feedback = 0 (constant error)

    # Execute multiple steps with constant error
    outputs = []
    for i in range(20):
        reg.execute()
        outputs.append(reg.Out)

    # Integral should accumulate: Ki * error * steps
    # After 20 steps: 5 * 10 * 20 = 1000, but limited to 100
    assert pytest.approx(outputs[-1], rel=1e-6) == 100.0


def test_pi_output_limit_upper():
    """Test PI output upper limit."""
    reg = PIRegulator(Kp=100.0, Ki=0.0, OutLimit=50.0)
    reg.Ref = 100.0
    reg.Fbk = 0.0

    reg.execute()

    # P gain 100 * error 100 = 10000, but limited to 50
    assert pytest.approx(reg.Out, rel=1e-6) == 50.0


def test_pi_output_limit_lower():
    """Test PI output lower limit."""
    reg = PIRegulator(Kp=100.0, Ki=0.0, OutLimit=50.0)
    reg.Ref = -100.0
    reg.Fbk = 0.0

    reg.execute()

    assert pytest.approx(reg.Out, rel=1e-6) == -50.0


def test_pi_integral_action():
    """Test PI integral action."""
    reg = PIRegulator(Kp=0.0, Ki=10.0, OutLimit=100.0)
    reg.Ref = 1.0
    reg.Fbk = 0.0

    # Execute several steps with constant error
    for _ in range(10):
        reg.execute()

    # After 10 steps with Ki=10 and error=1: Out = 10*1*10 = 100
    # But should be limited
    assert pytest.approx(reg.Out, rel=1e-6) == 100.0


def test_pi_incremental_form():
    """Test incremental PI form produces correct output."""
    reg = PIRegulator(Kp=1.0, Ki=2.0, OutLimit=1000.0)
    reg.Ref = 5.0
    reg.Fbk = 3.0

    # First step
    reg.execute()

    # Err = 2, OutPrev = 0, ErrPrev = 0
    # Out = 0 + 1*(2-0) + 2*2 = 2 + 4 = 6
    assert pytest.approx(reg.Out, rel=1e-6) == 6.0

    # Second step with new feedback
    reg.Fbk = 4.0
    reg.execute()

    # Err = 1, OutPrev = 6, ErrPrev = 2
    # Out = 6 + 1*(1-2) + 2*1 = 6 - 1 + 2 = 7
    assert pytest.approx(reg.Out, rel=1e-6) == 7.0


def test_pi_reset():
    """Test PI reset functionality."""
    reg = PIRegulator(Kp=1.0, Ki=10.0, OutLimit=100.0)
    reg.Ref = 10.0
    reg.Fbk = 0.0

    # Execute several steps
    for _ in range(10):
        reg.execute()

    # Reset
    reg.reset()

    assert reg.Err == 0.0
    assert reg.ErrPrev == 0.0
    assert reg.Out == 0.0
    assert reg.OutPrev == 0.0


def test_pi_setters():
    """Test PI setter methods."""
    reg = PIRegulator(Kp=1.0, Ki=1.0, OutLimit=100.0)

    reg.set_reference(50.0)
    assert reg.Ref == 50.0

    reg.set_feedback(45.0)
    assert reg.Fbk == 45.0

    reg.execute()
    output = reg.get_output()
    assert output == reg.Out

    error = reg.get_error()
    assert error == 5.0


def test_incremental_pi_function():
    """Test standalone incremental_pi function."""
    reg = PIRegulator(Kp=1.0, Ki=2.0, OutLimit=100.0)
    reg.Ref = 5.0
    reg.Fbk = 3.0

    # Use function instead of method
    incremental_pi(reg)

    # Same result as execute()
    assert pytest.approx(reg.Out, rel=1e-6) == 6.0


def test_pid_creation():
    """Test PID regulator creation."""
    reg = PIDRegulator(
        Kp=1.0, Ki=10.0, Kd=0.1, tau=0.01,
        OutLimit=100.0, IntLimit=50.0, T=0.001
    )
    assert reg.Kp == 1.0
    assert reg.Ki == 10.0
    assert reg.Kd == 0.1


def test_pid_zero_error():
    """Test PID with zero error."""
    reg = PIDRegulator(
        Kp=1.0, Ki=10.0, Kd=0.1, tau=0.01,
        OutLimit=100.0, IntLimit=50.0, T=0.001
    )
    reg.Ref = 0.0
    reg.Fbk = 0.0

    reg.execute()

    assert pytest.approx(reg.Out, rel=1e-6) == 0.0


def test_pid_proportional_only():
    """Test PID proportional term."""
    reg = PIDRegulator(
        Kp=10.0, Ki=0.0, Kd=0.0, tau=0.01,
        OutLimit=100.0, IntLimit=50.0, T=0.001
    )
    reg.Ref = 5.0
    reg.Fbk = 0.0

    reg.execute()

    # P = Kp * error = 10 * 5 = 50
    assert pytest.approx(reg.Out, rel=1e-6) == 50.0


def test_pid_integral_antiwindup():
    """Test PID integrator anti-windup."""
    reg = PIDRegulator(
        Kp=0.0, Ki=1000.0, Kd=0.0, tau=0.01,
        OutLimit=100.0, IntLimit=10.0, T=0.001
    )
    reg.Ref = 1.0
    reg.Fbk = 0.0

    # Execute many steps
    for _ in range(100):
        reg.execute()

    # Integrator should be limited
    assert abs(reg.integrator) <= reg.IntLimit


def test_pid_reset():
    """Test PID reset."""
    reg = PIDRegulator(
        Kp=1.0, Ki=10.0, Kd=0.1, tau=0.01,
        OutLimit=100.0, IntLimit=50.0, T=0.001
    )

    # Run some steps
    reg.Ref = 10.0
    for _ in range(10):
        reg.Fbk = 5.0
        reg.execute()

    # Reset
    reg.reset()

    assert reg.integrator == 0.0
    assert reg.differentiator == 0.0
    assert reg.Out == 0.0


def test_pi_negative_reference():
    """Test PI with negative reference."""
    reg = PIRegulator(Kp=10.0, Ki=5.0, OutLimit=100.0)
    reg.Ref = -10.0
    reg.Fbk = 0.0

    reg.execute()

    # Error = -10, P term = -100
    assert pytest.approx(reg.Out, rel=1e-6) == -100.0


def test_pi_tracking_error_history():
    """Test PI tracks error history correctly."""
    reg = PIRegulator(Kp=1.0, Ki=1.0, OutLimit=100.0)

    # First step
    reg.Ref = 10.0
    reg.Fbk = 5.0
    reg.execute()
    assert reg.Err == 5.0
    assert reg.ErrPrev == 5.0

    # Second step
    reg.Fbk = 7.0
    reg.execute()
    assert reg.Err == 3.0
    assert reg.ErrPrev == 3.0

# Additional PID tests for Phase 12

def test_pid_integral_accumulation():
    """Test PID integral accumulation."""
    reg = PIDRegulator(
        Kp=0.0, Ki=100.0, Kd=0.0, tau=0.01,
        OutLimit=1000.0, IntLimit=500.0, T=0.001
    )
    reg.Ref = 10.0
    reg.Fbk = 0.0

    # Execute 10 steps with constant error
    for _ in range(10):
        reg.execute()

    # Integral: Ki * error * T * steps = 100 * 10 * 0.001 * 10 = 10
    assert pytest.approx(reg.integrator, rel=1e-6) == 10.0


def test_pid_derivative_response():
    """Test PID derivative term response to changing measurement."""
    reg = PIDRegulator(
        Kp=0.0, Ki=0.0, Kd=10.0, tau=0.001,
        OutLimit=1000.0, IntLimit=500.0, T=0.001
    )
    reg.Ref = 0.0
    reg.Fbk = 0.0
    reg.prev_measurement = 10.0  # Previous measurement higher

    reg.execute()

    # Derivative: Kd * (prev_meas - meas) / T = 10 * (10 - 0) / 0.001 = 100000
    # But filtered through tau, so less
    assert reg.Out > 0.0  # Positive derivative output


def test_pid_output_limit():
    """Test PID output limiting."""
    reg = PIDRegulator(
        Kp=100.0, Ki=0.0, Kd=0.0, tau=0.01,
        OutLimit=50.0, IntLimit=100.0, T=0.001
    )
    reg.Ref = 10.0
    reg.Fbk = 0.0

    reg.execute()

    # P = 100 * 10 = 1000, limited to 50
    assert pytest.approx(reg.Out, rel=1e-6) == 50.0


def test_pid_negative_output():
    """Test PID with negative output."""
    reg = PIDRegulator(
        Kp=10.0, Ki=0.0, Kd=0.0, tau=0.01,
        OutLimit=100.0, IntLimit=50.0, T=0.001
    )
    reg.Ref = -10.0
    reg.Fbk = 0.0

    reg.execute()

    assert pytest.approx(reg.Out, rel=1e-6) == -100.0


def test_pid_combined_terms():
    """Test PID with all three terms active."""
    reg = PIDRegulator(
        Kp=1.0, Ki=10.0, Kd=0.1, tau=0.01,
        OutLimit=1000.0, IntLimit=500.0, T=0.001
    )
    reg.Ref = 10.0
    reg.Fbk = 5.0
    reg.prev_measurement = 5.0  # Set initial measurement to avoid derivative kick

    # First step
    reg.execute()

    # P term: Kp * Err = 1 * 5 = 5
    # I term: Ki * Err * T = 10 * 5 * 0.001 = 0.05 (accumulated)
    # D term: filtered derivative (no change in measurement)
    # Total should be approximately P + I = 5 + 0.05 = 5.05
    assert reg.Out > 0.0  # Positive output
    assert pytest.approx(reg.Out, rel=0.1) == 5.05


def test_pid_derivative_on_measurement():
    """Test PID derivative is on measurement (not reference)."""
    reg = PIDRegulator(
        Kp=0.0, Ki=0.0, Kd=100.0, tau=0.01,
        OutLimit=1000.0, IntLimit=500.0, T=0.001
    )

    # Reference step (should not cause derivative kick)
    reg.Ref = 10.0  # Step change in reference
    reg.Fbk = 0.0
    reg.prev_measurement = 0.0

    reg.execute()

    # Derivative on measurement: measurement unchanged, no derivative
    # But prev_measurement - Fbk = 0 - 0 = 0
    # So no derivative kick from reference step
    assert pytest.approx(reg.differentiator, rel=1e-6) == 0.0


def test_pid_step_sequence():
    """Test PID through multiple steps."""
    reg = PIDRegulator(
        Kp=5.0, Ki=20.0, Kd=0.5, tau=0.005,
        OutLimit=500.0, IntLimit=200.0, T=0.001
    )
    reg.Ref = 100.0

    outputs = []
    # Run simulation with slowly converging feedback
    for i in range(50):
        reg.Fbk = 50.0 + i  # Feedback increases slowly
        reg.execute()
        outputs.append(reg.Out)

    # Output should change over time
    assert outputs[-1] != outputs[0]
    # Integrator should accumulate
    assert reg.integrator != 0.0