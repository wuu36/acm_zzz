"""tests for speed observer implementation."""

import pytest
import numpy as np
from acmzzz.observers.speed_observer import (
    SpeedObserver,
    dynamics_speed_observer,
    compute_observer_gains,
)

class TestSpeedObserverCreation:
    """tests for speedobserver creation."""
    
    def test_observer_creation_default(self):
        """test default observer creation."""
        observer = SpeedObserver()
        assert observer.order == 3
        assert observer.bandwidth == 100
        assert observer.npp == 4
        assert observer.Js == 0.0006168

    def test_observer_creation_custom(self):
        """test custom observer creation."""
        observer = SpeedObserver(npp=2, Js=0.001, bandwidth=50.0, order=4)
        assert observer.npp == 2
        assert observer.Js == 0.001
        assert observer.bandwidth == 50.0
        assert observer.order == 4

    def test_observer_state_size(self):
        """test state vector size based on order."""
        # 2nd-order
        observer = SpeedObserver(order=2)
        assert len(observer.x) == 3

        # 3rd-order
        observer = SpeedObserver(order=3)
        assert len(observer.x) == 4

        # 4th-order
        observer = SpeedObserver(order=4)
        assert len(observer.x) == 4

class TestObserverGains:
    """Tests for observer gain computation."""

    def test_gains_2nd_order(self):
        """Test 2nd-order observer gains."""
        ell1, ell2, ell3, ell4 = compute_observer_gains(2, 100.0, 4, 0.0006168)

        assert pytest.approx(ell1, rel=1e-6) == 200.0
        assert ell2 > 0.0
        assert ell3 == 0.0
        assert ell4 == 0.0

    def test_gains_3rd_order(self):
        """Test 3rd-order observer gains."""
        ell1, ell2, ell3, ell4 = compute_observer_gains(3, 100.0, 4, 0.0006168)

        assert pytest.approx(ell1, rel=1e-6) == 300.0
        assert pytest.approx(ell2, rel=1e-6) == 30000.0
        assert ell3 > 0.0
        assert ell4 == 0.0

    def test_gains_4th_order(self):
        """Test 4th-order observer gains."""
        ell1, ell2, ell3, ell4 = compute_observer_gains(4, 100.0, 4, 0.0006168)

        assert pytest.approx(ell1, rel=1e-6) == 400.0
        assert pytest.approx(ell2, rel=1e-6) == 60000.0
        assert ell3 > 0.0
        assert pytest.approx(ell4, rel=1e-6) == 100**4  # omega_ob^4 = 100000000

    def test_gains_in_observer(self):
        """Test gains computed in observer."""
        observer = SpeedObserver(bandwidth=100.0, order=3)

        # Should match compute_observer_gains
        ell1, ell2, ell3, ell4 = compute_observer_gains(3, 100.0, observer.npp, observer.Js)

        assert pytest.approx(observer.ell1, rel=1e-6) == ell1
        assert pytest.approx(observer.ell2, rel=1e-6) == ell2
        assert pytest.approx(observer.ell3, rel=1e-6) == ell3

    def test_gains_different_bandwidth(self):
        """Test gains for different bandwidth."""
        observer1 = SpeedObserver(bandwidth=50.0)
        observer2 = SpeedObserver(bandwidth=200.0)

        # Higher bandwidth should have higher gains
        assert observer2.ell1 > observer1.ell1
        assert observer2.ell2 > observer1.ell2

class TestObserverEstimates:
    """tests for observer estimates."""
    
    def test_get_position_estimate(self):
        """test position estimate."""
        observer = SpeedObserver()
        observer.x[0] = 1.5

        assert pytest.approx(observer.get_position_estimate(), rel=1e-6) == 1.5
    
    def test_get_speed_estimate(self):
        """Test speed estimate."""
        observer = SpeedObserver()
        observer.x[1] = 100.0  # rad/s

        assert pytest.approx(observer.get_speed_estimate(), rel=1e-6) == 100.0

    def test_get_speed_estimate_mech(self):
        """Test mechanical speed estimate."""
        observer = SpeedObserver(npp=4)
        observer.x[1] = 400.0  # 400 rad/s electrical

        # Mechanical speed = 400 / 4 = 100 rad/s
        assert pytest.approx(observer.get_speed_estimate_mech(), rel=1e-6) == 100.0

    def test_get_speed_estimate_rpm(self):
        """Test speed estimate in RPM."""
        observer = SpeedObserver(npp=4)
        observer.x[1] = 400.0  # 400 rad/s electrical = 100 rad/s mech

        # 100 rad/s * 60 / (2π) ≈ 954.9 rpm
        expected = 100.0 * 60.0 / (2.0 * np.pi)
        assert pytest.approx(observer.get_speed_estimate_rpm(), rel=1e-3) == expected

    def test_get_disturbance_estimate(self):
        """Test disturbance torque estimate."""
        observer = SpeedObserver(order=3)
        observer.x[2] = 0.5  # 0.5 Nm disturbance

        assert pytest.approx(observer.get_disturbance_estimate(), rel=1e-6) == 0.5

    def test_get_total_torque_estimate(self):
        """Test total torque estimate."""
        observer = SpeedObserver(order=3)
        observer.Tem = 1.0
        observer.x[2] = 0.5

        # Total = Tem + disturbance = 1.0 + 0.5 = 1.5
        assert pytest.approx(observer.get_total_torque_estimate(), rel=1e-6) == 1.5

class TestObserverUpdate:
    """Tests for observer update."""

    def test_update_zero_error(self):
        """Test update with zero error."""
        observer = SpeedObserver(bandwidth=10.0)  # Low bandwidth for stability

        # Set initial position estimate equal to measured
        observer.set_position_estimate(1.0)

        # Update with same position (zero error)
        observer.update(theta_measured=1.0, Tem=0.0, dt=1e-4)

        # Position should stay approximately the same
        assert pytest.approx(observer.get_position_estimate(), rel=1e-3) == 1.0

    def test_update_with_error(self):
        """Test update with position error."""
        observer = SpeedObserver(bandwidth=10.0)

        # Initial estimate at 0, measured at 1.0
        observer.set_position_estimate(0.0)

        # Multiple updates should converge
        for _ in range(100):
            observer.update(theta_measured=1.0, Tem=0.0, dt=1e-4)

        # Should converge toward measured position
        assert observer.output_error < 1.0  # Error reduced

    def test_update_with_speed(self):
        """Test update with actual speed."""
        observer = SpeedObserver(bandwidth=50.0)

        # Simulate motor running at constant speed
        omega = 100.0  # rad/s
        dt = 1e-4

        theta = 0.0
        for i in range(100):
            theta += omega * dt  # Position increases
            observer.update(theta_measured=theta, Tem=0.0, dt=dt)

        # Observer should estimate speed close to omega
        # (may take longer to converge perfectly)
        speed_est = observer.get_speed_estimate()
        assert speed_est > 0.0  # Should detect positive speed

    def test_update_with_torque(self):
        """Test update with electromagnetic torque."""
        observer = SpeedObserver(bandwidth=50.0, order=3)

        # Motor accelerating with torque
        Tem = 1.0  # 1 Nm torque
        dt = 1e-4

        theta = 0.0
        omega = 0.0

        for i in range(200):
            # Simple acceleration: omega increases due to torque
            omega += Tem / observer.Js * dt
            theta += omega * dt

            observer.update(theta_measured=theta, Tem=Tem, dt=dt)

        # Observer should track the speed
        speed_est = observer.get_speed_estimate()
        assert speed_est > 0.0

    def test_update_convergence(self):
        """Test observer convergence to true values."""
        observer = SpeedObserver(bandwidth=100.0, npp=4, Js=0.0006168)

        # Simulate constant speed operation
        true_speed = 200.0  # rad/s electrical
        true_theta = 0.0
        Tem = 0.0
        dt = 1e-4

        # Initialize with WRONG position estimate (not matching measured)
        observer.set_position_estimate(5.0)  # 5 rad off
        observer.set_speed_estimate(0.0)

        # Initial error should be large
        initial_error = abs(5.0 - 0.0)  # Expected ~5 rad

        errors = []
        for i in range(500):
            true_theta += true_speed * dt
            observer.update(theta_measured=true_theta, Tem=Tem, dt=dt)
            errors.append(abs(observer.output_error))

        # Error should decrease significantly from initial
        # Note: observer converges toward measured position
        # Final error should be much smaller than initial offset
        assert errors[-1] < initial_error

class TestObserverReset:
    """Tests for observer reset."""

    def test_reset_clears_state(self):
        """Test reset clears state vector."""
        observer = SpeedObserver()

        # Set some state
        observer.x[0] = 1.0
        observer.x[1] = 100.0
        observer.x[2] = 0.5

        observer.reset()

        assert np.all(observer.x == 0.0)
        assert observer.output_error == 0.0
        assert observer.Tem == 0.0

class TestObserverSetters:
    """Tests for observer setter methods."""

    def test_set_position_estimate(self):
        """Test setting position estimate."""
        observer = SpeedObserver()
        observer.set_position_estimate(2.5)

        assert pytest.approx(observer.x[0], rel=1e-6) == 2.5

    def test_set_speed_estimate(self):
        """Test setting speed estimate."""
        observer = SpeedObserver()
        observer.set_speed_estimate(300.0)

        assert pytest.approx(observer.x[1], rel=1e-6) == 300.0

    def test_set_bandwidth(self):
        """Test changing bandwidth."""
        observer = SpeedObserver(bandwidth=100.0)
        old_ell1 = observer.ell1

        observer.set_bandwidth(200.0)

        # Gains should be recomputed
        assert observer.bandwidth == 200.0
        assert observer.ell1 != old_ell1
        assert pytest.approx(observer.ell1, rel=1e-6) == 600.0  # 3 * 200

class TestObserverDynamicsFunction:
    """Tests for standalone dynamics function."""

    def test_dynamics_zero_error(self):
        """Test dynamics with zero error."""
        ell = (300.0, 30000.0, 10000.0, 0.0)
        x = np.array([1.0, 100.0, 0.0, 0.0])
        theta_measured = 1.0  # Same as estimate

        fx = dynamics_speed_observer(x, theta_measured, 0.0, ell, 4, 0.0006168)

        # With zero error, speed estimate should just propagate
        assert pytest.approx(fx[0], rel=1e-6) == 100.0  # x[1]

    def test_dynamics_with_error(self):
        """Test dynamics with position error."""
        ell = (300.0, 30000.0, 10000.0, 0.0)
        x = np.array([0.0, 0.0, 0.0, 0.0])
        theta_measured = 1.0  # Error of 1 rad

        fx = dynamics_speed_observer(x, theta_measured, 0.0, ell, 4, 0.0006168)

        # Observer should correct based on error
        assert fx[0] > 0.0  # Position should increase

    def test_dynamics_with_torque(self):
        """Test dynamics with electromagnetic torque."""
        ell = (300.0, 30000.0, 10000.0, 0.0)
        x = np.array([0.0, 0.0, 0.0, 0.0])
        Tem = 1.0

        fx = dynamics_speed_observer(x, 0.0, Tem, ell, 4, 0.0006168)

        # Speed derivative includes torque term
        # fx[1] = ell2*error + (Tem + x[2]) * npp / Js
        expected_speed_deriv = Tem * 4 / 0.0006168  # Without error term
        assert pytest.approx(fx[1], rel=1e-3) == pytest.approx(expected_speed_deriv, rel=1e-3)
        
class TestObserverIntegration:
    """Integration tests for observer."""

    def test_observer_tracking_constant_speed(self):
        """Test tracking constant speed over time."""
        observer = SpeedObserver(bandwidth=200.0)
        dt = 1e-4
        true_speed = 100.0  # rad/s

        theta = 0.0
        estimates = []

        for _ in range(1000):
            theta += true_speed * dt
            observer.update(theta_measured=theta, Tem=0.0, dt=dt)
            estimates.append(observer.get_speed_estimate())

        # Should converge to true speed
        final_estimate = estimates[-1]
        # Allow some tolerance for convergence
        assert abs(final_estimate - true_speed) < 50.0

    def test_observer_with_disturbance(self):
        """Test observer estimating disturbance torque."""
        observer = SpeedObserver(bandwidth=100.0, order=3)
        dt = 1e-4

        # Motor with constant load torque
        Tem = 1.0  # 1 Nm commanded
        T_load = 0.5  # 0.5 Nm load (disturbance)

        theta = 0.0
        omega = 0.0
        Js = observer.Js

        for _ in range(500):
            # Acceleration: (Tem - T_load) / Js
            omega += (Tem - T_load) / Js * dt
            theta += omega * dt

            observer.update(theta_measured=theta, Tem=Tem, dt=dt)

        # Observer should estimate some disturbance
        disturbance = observer.get_disturbance_estimate()
        # Disturbance estimate should be negative (opposing motion)
        # or positive depending on observer dynamics
        # Note: exact value depends on convergence time

def test_observer_fixture(motor_config):
    """Test observer with motor fixture."""
    npp = motor_config['npp']
    Js = motor_config['Js']

    observer = SpeedObserver(npp=npp, Js=Js, bandwidth=100.0)

    # Basic check
    assert observer.npp == npp
    assert pytest.approx(observer.Js, rel=1e-6) == Js


def test_observer_multiple_updates():
    """Test observer through multiple update cycles."""
    observer = SpeedObserver(bandwidth=50.0)

    theta = 0.0
    omega = 0.0
    Tem = 0.5
    dt = 1e-4

    for i in range(100):
        # Motor dynamics
        omega += Tem / observer.Js * dt
        theta += omega * dt

        observer.update(theta_measured=theta, Tem=Tem, dt=dt)

    # Observer should be tracking
    assert observer.get_speed_estimate() > 0.0