"""tests for angle utility functions."""

import pytest
import numpy as np
from acmzzz.utils.angle import(
    angle_normalize,
    angle_diff,
    angle_wrap,
    angle_increment,
    angle_to_degrees,
    angle_to_radians,
    speed_to_rpm,
    rpm_to_speed,
    electrical_angle,
    mechanical_angle,
)

class TestAngleNormalize:
    """tests for angle_normalize function."""
    
    def test_normalize_positive_angle(self):
        """test normalization of positive angle."""
        # 7 rad > 2pi, should wrap to 7 - 2pi
        result = angle_normalize(7.0)
        excepted = 7.0 - 2.0 * np.pi
        assert pytest.approx(result, rel=1e-6) == excepted

    def test_normalize_negative_angle(self):
        """Test normalization of negative angle."""
        # -1 rad, should wrap to 2π - 1
        result = angle_normalize(-1.0)
        expected = 2.0 * np.pi - 1.0
        assert pytest.approx(result, rel=1e-6) == expected

    def test_normalize_already_in_range(self):
        """Test angle already in range."""
        result = angle_normalize(1.0)
        assert pytest.approx(result, rel=1e-6) == 1.0

    def test_normalize_zero(self):
        """Test zero angle."""
        result = angle_normalize(0.0)
        assert pytest.approx(result, rel=1e-6) == 0.0

    def test_normalize_2pi(self):
        """Test exactly 2π."""
        result = angle_normalize(2.0 * np.pi)
        assert pytest.approx(result, rel=1e-6) == 0.0
    
    def test_normalize_multiple_wraps(self):
        """Test angle with multiple wraps."""
        # 10π wraps 5 times
        result = angle_normalize(10.0 * np.pi)
        assert pytest.approx(result, rel=1e-6) == 0.0

    def test_normalize_pi_range(self):
        """Test normalization to [-π, π] range."""
        # 4 rad > π, should wrap to 4 - 2π (negative)
        result = angle_normalize(4.0, range_type='-pi_to_pi')
        expected = 4.0 - 2.0 * np.pi
        assert pytest.approx(result, rel=1e-6) == expected

    def test_normalize_negative_to_pi_range(self):
        """Test negative angle in [-π, π] range."""
        result = angle_normalize(-0.5, range_type='-pi_to_pi')
        assert pytest.approx(result, rel=1e-6) == -0.5

    def test_normalize_large_negative_pi_range(self):
        """Test large negative angle in [-π, π] range."""
        # -4 rad < -π, should wrap to 2π - 4 (positive)
        result = angle_normalize(-4.0, range_type='-pi_to_pi')
        expected = 2.0 * np.pi - 4.0
        assert pytest.approx(result, rel=1e-6) == expected

class TestAngleDiff:
    """test for angle_diff function."""
    
    def test_diff_small_positive(self):
        """test small positive difference."""
        result = angle_diff(0.5, 0.0)
        assert pytest.approx(result, rel=1e-6) == 0.5

    def test_diff_small_negative(self):
        """Test small negative difference."""
        result = angle_diff(0.0, 0.5)
        assert pytest.approx(result, rel=1e-6) == -0.5

    def test_diff_wraparound(self):
        """Test difference across 2π boundary."""
        # 0.1 rad vs 6.2 rad (near 2π)
        result = angle_diff(0.1, 6.2)
        # Should wrap to small positive value
        expected = 0.1 - (6.2 - 2.0 * np.pi)
        assert pytest.approx(result, rel=1e-6) == expected

    def test_diff_opposite_wraparound(self):
        """Test opposite wraparound case."""
        # 6.2 rad vs 0.1 rad
        result = angle_diff(6.2, 0.1)
        # Should wrap to small negative value
        expected = (6.2 - 2.0 * np.pi) - 0.1
        assert pytest.approx(result, rel=1e-6) == expected

    def test_diff_quarter_pi(self):
        """Test π/4 difference."""
        result = angle_diff(np.pi/2, np.pi/4)
        assert pytest.approx(result, rel=1e-6) == np.pi/4

    def test_diff_three_quarter_pi(self):
        """Test 3π/4 difference."""
        result = angle_diff(0.0, 3*np.pi/4)
        # Direct difference is -3π/4, wrapped difference would be +π/4
        # Function returns the smaller magnitude, but both are equal here
        # Returns -3π/4 (direct difference d1)
        expected = -3*np.pi/4
        assert pytest.approx(result, rel=1e-6) == expected

    def test_diff_pi(self):
        """Test π difference."""
        result = angle_diff(0.0, np.pi)
        # Both +π and -π are same distance
        assert abs(result) <= np.pi + 1e-6

    def test_diff_same_angles(self):
        """Test difference of same angles."""
        result = angle_diff(1.0, 1.0)
        assert pytest.approx(result, rel=1e-6) == 0.0

    def test_diff_large_angles(self):
        """Test difference of large angles."""
        # Both angles > 2π
        result = angle_diff(7.0, 6.5)
        expected = 0.5  # After normalization
        assert pytest.approx(result, rel=1e-6) == expected

    def test_diff_from_original_test_cases(self):
        """Test cases from original code comments."""
        # angle_diff(7, -7) ≈ 0.28 rad
        result = angle_diff(7.0, -7.0)
        # 7 normalized = 0.7168, -7 normalized = 5.2832
        # diff should be ~ -0.43 or wrapped to ~0.28
        assert abs(result) < np.pi

        # angle_diff(7, -6)
        result = angle_diff(7.0, -6.0)
        assert abs(result) < np.pi

class TestAngleWrap:
    """tests for angle_wrap function."""
    
    def test_wrap_around_zero(self):
        """test wrap around zero center."""
        result = angle_wrap(7.0, center=0.0)
        expected = 7.0 - 2.0 * np.pi
        assert pytest.approx(result, rel=1e-6) == expected

    def test_wrap_around_pi(self):
        """Test wrap around π center."""
        result = angle_wrap(4.0, center=np.pi)
        # 4 - π = 0.7168, no wrap needed
        expected = 4.0
        assert pytest.approx(result, rel=1e-6) == expected

    def test_wrap_no_wrap_needed(self):
        """Test when no wrap is needed."""
        result = angle_wrap(1.0, center=0.5)
        assert pytest.approx(result, rel=1e-6) == 1.0

class TestAngleIncrement:
    """tests for angle_increment function."""
    
    def test_increment_positive(self):
        """tests positive increment."""
        result = angle_increment(1.0, 0.5)
        assert pytest.approx(result, rel=1e-6) == 1.5

    def test_increment_cross_boundary(self):
        """Test increment crossing 2π."""
        result = angle_increment(6.0, 0.5)
        assert pytest.approx(result, rel=1e-6) == 6.5  # No wrap

    def test_increment_negative(self):
        """Test negative increment."""
        result = angle_increment(1.0, -0.5)
        assert pytest.approx(result, rel=1e-6) == 0.5

class TestAngleConversions:
    """test for angle conversion functions."""
    
    def test_to_degress(self):
        """test radian to degree conversion."""
        result = angle_to_degrees(np.pi)
        assert pytest.approx(result, rel=1e-6) == 180.0
    
    def test_to_radians(self):
        """test degree to radian conversion."""
        result = angle_to_radians(180.0)
        assert pytest.approx(result, rel=1e-6) == np.pi

    def test_to_degrees_zero(self):
        """Test zero angle."""
        result = angle_to_degrees(0.0)
        assert pytest.approx(result, rel=1e-6) == 0.0

    def test_roundtrip(self):
        """Test roundtrip conversion."""
        original = 1.5
        result = angle_to_radians(angle_to_degrees(original))
        assert pytest.approx(result, rel=1e-6) == original

class TestSpeedConversions:
    """test for speed conversion functions."""
    
    def test_speed_to_rpm(self):
        """test rad/s to RPM conversion."""
        # 100 rad/s ≈ 954.9 rpm
        result = speed_to_rpm(100.0)
        excepted = 100.0 * 60.0 / (2.0 * np.pi)
        assert pytest.approx(result, rel=1e-6) == excepted

    def test_rpm_to_speed(self):
        """Test RPM to rad/s conversion."""
        # 1000 rpm ≈ 104.7 rad/s
        result = rpm_to_speed(1000.0)
        expected = 1000.0 * 2.0 * np.pi / 60.0
        assert pytest.approx(result, rel=1e-6) == expected

    def test_speed_roundtrip(self):
        """Test roundtrip conversion."""
        original = 50.0
        result = rpm_to_speed(speed_to_rpm(original))
        assert pytest.approx(result, rel=1e-6) == original

class TestElectricalMechanical:
    """Tests for electrical/mechanical angle conversions."""

    def test_electrical_angle(self):
        """Test mechanical to electrical angle."""
        theta_mech = 1.0  # rad
        npp = 4
        result = electrical_angle(theta_mech, npp)
        assert pytest.approx(result, rel=1e-6) == 4.0

    def test_mechanical_angle(self):
        """Test electrical to mechanical angle."""
        theta_elec = 4.0  # rad
        npp = 4
        result = mechanical_angle(theta_elec, npp)
        assert pytest.approx(result, rel=1e-6) == 1.0

    def test_angle_roundtrip(self):
        """Test roundtrip conversion."""
        theta_mech = 2.5
        npp = 4
        theta_elec = electrical_angle(theta_mech, npp)
        theta_back = mechanical_angle(theta_elec, npp)
        assert pytest.approx(theta_back, rel=1e-6) == theta_mech

    def test_electrical_angle_zero(self):
        """Test zero angle."""
        result = electrical_angle(0.0, 4)
        assert pytest.approx(result, rel=1e-6) == 0.0

# additional edge case tests

def test_angle_normalize_fixture():
    """test with fixture-like values."""
    # typical motor angles
    for angle in [0.0, np.pi/2, np.pi, 3*np.pi/2]:
        result = angle_normalize(angle)
        assert 0.0 <= result < 2.0 * np.pi

def test_angle_diff_symmetry():
    """Test that angle_diff(a,b) ≈ -angle_diff(b,a)."""
    a = 1.5
    b = 0.5
    diff_ab = angle_diff(a, b)
    diff_ba = angle_diff(b, a)
    assert pytest.approx(diff_ab, rel=1e-6) == -diff_ba


def test_angle_diff_additivity():
    """Test angle_diff(a,c) ≈ angle_diff(a,b) + angle_diff(b,c)."""
    a = 0.5
    b = 1.5
    c = 2.5
    diff_ab = angle_diff(a, b)
    diff_bc = angle_diff(b, c)
    diff_ac = angle_diff(a, c)
    # Should be approximately additive (within one wrap)
    assert pytest.approx(diff_ab + diff_bc, rel=1e-6) == diff_ac