"""Tests for PI parameter tuner."""

import pytest
import numpy as np
from acmzzz.utils.tuner import (
    get_current_pi,
    get_speed_pi,
    tune_for_desired_speed_bandwidth,
    PITuner,
    print_pi_parameters,
    HAS_CONTROL_LIB,
)


class TestGetCurrentPI:
    """Tests for current PI design."""

    def test_basic_design(self):
        """Test basic current PI design."""
        R = 1.1  # Ohm
        L = 6e-3  # H
        bandwidth_hz = 1000.0

        Kp, Ki = get_current_pi(R, L, bandwidth_hz)

        # Kp = bandwidth * 2π * L
        expected_Kp = 1000.0 * 2.0 * np.pi * 6e-3
        assert pytest.approx(Kp, rel=1e-6) == expected_Kp

        # Ki = R / L
        expected_Ki = 1.1 / 6e-3
        assert pytest.approx(Ki, rel=1e-6) == expected_Ki

    def test_different_bandwidth(self):
        """Test design with different bandwidth."""
        R = 1.0
        L = 1e-3

        Kp1, Ki1 = get_current_pi(R, L, 500.0)
        Kp2, Ki2 = get_current_pi(R, L, 1000.0)

        # Higher bandwidth gives higher Kp
        assert Kp2 > Kp1
        # Ki is same (depends only on R/L)
        assert pytest.approx(Ki1, rel=1e-6) == Ki2

    def test_zero_bandwidth(self):
        """Test zero bandwidth gives zero Kp."""
        R = 1.0
        L = 1e-3

        Kp, Ki = get_current_pi(R, L, 0.0)
        assert pytest.approx(Kp, rel=1e-6) == 0.0
        # Ki still valid
        assert Ki > 0.0


class TestGetSpeedPI:
    """Tests for speed PI design."""

    def test_basic_design(self):
        """Test basic speed PI design."""
        Js = 0.0006168  # kg.m^2
        npp = 4
        KA = 0.095  # Wb
        delta = 6.5
        current_bw_rad = 1000.0 * 2.0 * np.pi

        Kp, Ki = get_speed_pi(Js, npp, KA, delta, current_bw_rad)

        # Ki = current_bw / delta^2
        expected_Ki = current_bw_rad / delta**2
        assert pytest.approx(Ki, rel=1e-6) == expected_Ki

        # Kp > 0
        assert Kp > 0.0

    def test_symmetric_optimal_relation(self):
        """Test symmetric optimal bandwidth relation."""
        Js = 0.0006168
        npp = 4
        KA = 0.095
        delta = 6.5

        current_bw_rad1 = 500.0 * 2.0 * np.pi
        current_bw_rad2 = 1000.0 * 2.0 * np.pi

        Kp1, Ki1 = get_speed_pi(Js, npp, KA, delta, current_bw_rad1)
        Kp2, Ki2 = get_speed_pi(Js, npp, KA, delta, current_bw_rad2)

        # Ki proportional to current bandwidth
        assert pytest.approx(Ki2 / Ki1, rel=1e-3) == 2.0

    def test_different_delta(self):
        """Test different delta values."""
        Js = 0.0006168
        npp = 4
        KA = 0.095
        current_bw_rad = 1000.0 * 2.0 * np.pi

        Kp1, Ki1 = get_speed_pi(Js, npp, KA, 3.0, current_bw_rad)
        Kp2, Ki2 = get_speed_pi(Js, npp, KA, 10.0, current_bw_rad)

        # Larger delta gives lower Ki (wider separation)
        assert Ki2 < Ki1

    def test_torque_constant_relation(self):
        """Test Kp relation to torque constant."""
        Js = 0.0006168
        npp = 4
        KA = 0.095
        delta = 6.5
        current_bw_rad = 1000.0 * 2.0 * np.pi

        Kp, Ki = get_speed_pi(Js, npp, KA, delta, current_bw_rad)

        KT = 1.5 * npp * KA
        expected_Kp = Js / npp / KT * delta * Ki
        assert pytest.approx(Kp, rel=1e-6) == expected_Kp


class TestTuneForDesiredSpeedBandwidth:
    """Tests for iterative bandwidth tuning."""

    def test_convergence(self):
        """Test that iteration converges."""
        R = 1.1
        L = 6e-3
        Js = 0.0006168
        npp = 4
        KA = 0.095
        desired_vl_bw_hz = 50.0

        result = tune_for_desired_speed_bandwidth(
            R, L, Js, npp, KA, desired_vl_bw_hz
        )

        current_Kp, current_Ki, speed_Kp, speed_Ki, cl_bw, vl_bw = result

        # All parameters should be positive
        assert current_Kp > 0.0
        assert current_Ki > 0.0
        assert speed_Kp > 0.0
        assert speed_Ki > 0.0

        # Achieved bandwidth should be close to desired
        assert abs(vl_bw - desired_vl_bw_hz) < 20.0

    def test_higher_speed_bandwidth(self):
        """Test tuning for higher speed bandwidth."""
        result1 = tune_for_desired_speed_bandwidth(
            1.1, 6e-3, 0.0006168, 4, 0.095, 30.0
        )
        result2 = tune_for_desired_speed_bandwidth(
            1.1, 6e-3, 0.0006168, 4, 0.095, 60.0
        )

        # Higher speed bandwidth requires higher current bandwidth
        _, _, _, _, cl_bw1, vl_bw1 = result1
        _, _, _, _, cl_bw2, vl_bw2 = result2

        assert cl_bw2 >= cl_bw1


class TestPITuner:
    """Tests for PITuner class."""

    def test_tuner_creation(self):
        """Test PITuner creation."""
        tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)

        assert tuner.R == 1.1
        assert tuner.L == 6e-3
        assert tuner.Js == 0.0006168
        assert tuner.npp == 4
        assert tuner.KA == 0.095
        assert tuner.delta == 6.5

    def test_tune_current_loop(self):
        """Test current loop tuning."""
        tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)
        tuner.tune_current_loop(1000.0)

        assert tuner.current_Kp > 0.0
        assert tuner.current_Ki > 0.0
        assert pytest.approx(tuner.cl_bw_hz, rel=1e-6) == 1000.0

    def test_tune_speed_loop(self):
        """Test speed loop tuning."""
        tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)
        tuner.tune_current_loop(1000.0)
        tuner.tune_speed_loop()

        assert tuner.speed_Kp > 0.0
        assert tuner.speed_Ki > 0.0
        assert tuner.vl_bw_hz > 0.0

    def test_tune_for_speed_bandwidth(self):
        """Test iterative tuning."""
        tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)
        tuner.tune_for_speed_bandwidth(50.0)

        # Should achieve approximately 50 Hz speed bandwidth
        assert abs(tuner.vl_bw_hz - 50.0) < 20.0

    def test_get_parameters(self):
        """Test get_parameters method."""
        tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)
        tuner.tune_current_loop(1000.0)
        tuner.tune_speed_loop()

        params = tuner.get_parameters()

        assert "current_Kp" in params
        assert "current_Ki" in params
        assert "speed_Kp" in params
        assert "speed_Ki" in params
        assert "current_bw_hz" in params
        assert "speed_bw_hz" in params

    def test_custom_delta(self):
        """Test tuner with custom delta."""
        tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095, delta=10.0)

        tuner.tune_current_loop(1000.0)
        tuner.tune_speed_loop()

        # With delta=10, speed bandwidth ≈ 1000/100 = 10 Hz
        assert pytest.approx(tuner.vl_bw_hz, rel=0.1) == 10.0


class TestPrintPIParameters:
    """Tests for print function."""

    def test_print_basic(self):
        """Test basic print functionality."""
        # Should not raise exception
        print_pi_parameters(10.0, 100.0, 0.05, 30.0)

    def test_print_with_bandwidth(self):
        """Test print with bandwidth."""
        # Should not raise exception
        print_pi_parameters(10.0, 100.0, 0.05, 30.0, 1000.0, 50.0)


class TestHASCONTROLLIB:
    """Tests for control library availability."""

    def test_flag_defined(self):
        """Test that flag is defined."""
        assert isinstance(HAS_CONTROL_LIB, bool)


# Integration tests

def test_tuner_consistency():
    """Test that tuner produces consistent results."""
    R, L, Js, npp, KA = 1.1, 6e-3, 0.0006168, 4, 0.095

    # Using direct functions
    Kp1, Ki1 = get_current_pi(R, L, 1000.0)
    current_bw_rad = 1000.0 * 2.0 * np.pi
    Kp2, Ki2 = get_speed_pi(Js, npp, KA, 6.5, current_bw_rad)

    # Using PITuner
    tuner = PITuner(R, L, Js, npp, KA)
    tuner.tune_current_loop(1000.0)
    tuner.tune_speed_loop()

    assert pytest.approx(tuner.current_Kp, rel=1e-6) == Kp1
    assert pytest.approx(tuner.current_Ki, rel=1e-6) == Ki1
    assert pytest.approx(tuner.speed_Kp, rel=1e-6) == Kp2
    assert pytest.approx(tuner.speed_Ki, rel=1e-6) == Ki2


def test_tuner_fixture(motor_config):
    """Test tuner with motor fixture."""
    tuner = PITuner(
        R=motor_config['R'],
        L=motor_config['Lq'],
        Js=motor_config['Js'],
        npp=motor_config['npp'],
        KA=motor_config['KE']
    )

    tuner.tune_current_loop(1000.0)
    tuner.tune_speed_loop()

    # All parameters should be reasonable
    params = tuner.get_parameters()
    assert all(v > 0 for v in [params['current_Kp'], params['current_Ki'],
                                params['speed_Kp'], params['speed_Ki']])


def test_bandwidth_proportionality():
    """Test bandwidth proportionality relationships."""
    tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)

    # Current bandwidth proportional to Kp
    tuner.tune_current_loop(500.0)
    Kp1 = tuner.current_Kp
    tuner.tune_current_loop(1000.0)
    Kp2 = tuner.current_Kp

    assert pytest.approx(Kp2 / Kp1, rel=1e-3) == 2.0


# Tests for transfer function analysis (only if control lib available)

@pytest.mark.skipif(not HAS_CONTROL_LIB, reason="python-control not installed")
def test_analyze_current_loop():
    """Test current loop transfer function analysis."""
    tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)
    tuner.tune_current_loop(1000.0)

    analysis = tuner.analyze()

    if "error" not in analysis.get("current_loop", {}):
        assert "bandwidth_hz" in analysis["current_loop"]


@pytest.mark.skipif(not HAS_CONTROL_LIB, reason="python-control not installed")
def test_analyze_speed_loop():
    """Test speed loop transfer function analysis."""
    tuner = PITuner(R=1.1, L=6e-3, Js=0.0006168, npp=4, KA=0.095)
    tuner.tune_current_loop(1000.0)
    tuner.tune_speed_loop()

    analysis = tuner.analyze()

    if "error" not in analysis.get("speed_loop", {}):
        assert "bandwidth_hz" in analysis["speed_loop"]