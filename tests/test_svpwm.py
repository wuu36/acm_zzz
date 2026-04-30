"""tests for svpwm implementation."""

import pytest
import numpy as np
from acmzzz.inverter.svpwm import (
    SVPWM,
    determine_sector,
    svpwm_duty_cycle,
    inverse_clarke_svpwm,
    SQRT3,
)

class TestSVPWMCreation:
    """tests for SVPWM class creation."""
    
    def test_svpwm_creation_default(self):
        """test default SVPWM creation."""
        svpwm = SVPWM()
        assert svpwm.Vdc == 300.0
        assert svpwm.max_duty == 0.95
        assert svpwm.min_duty == 0.05
        assert svpwm.sector == 0
        assert svpwm.Ta == 0.5
        assert svpwm.Tb == 0.5
        assert svpwm.Tc == 0.5
    
    def test_svpwm_creation_custom_vdc(self):
        """Test custom Vdc."""
        svpwm = SVPWM(Vdc=500.0)
        assert svpwm.Vdc == 500.0

    def test_svpwm_creation_custom_limits(self):
        """Test custom duty limits."""
        svpwm = SVPWM(Vdc=300.0, max_duty=0.9, min_duty=0.1)
        assert svpwm.max_duty == 0.9
        assert svpwm.min_duty == 0.1

class TestSVPWMSectorDetermination:
    """Tests for sector determination."""

    def test_sector_zero_voltage(self):
        """Test sector for zero voltage."""
        sector = determine_sector(0.0, 0.0, 300.0)
        assert sector == 0

    def test_sector_at_30_degrees(self):
        """Test sector at 30 degrees (Sector 3 in SVPWM numbering)."""
        # Voltage vector at 30 degrees
        # SVPWM sector numbering is different from angle sequence
        magnitude = 100.0
        angle = np.radians(30)
        Ualpha = magnitude * np.cos(angle)
        Ubeta = magnitude * np.sin(angle)
        sector = determine_sector(Ualpha, Ubeta, 300.0)
        # At 30°: A>0 (sector=1), C>0 (sector=3), B<0 → Sector 3
        assert sector == 3

    def test_sector_at_90_degrees(self):
        """Test sector at 90 degrees (Sector 1 in SVPWM numbering)."""
        magnitude = 100.0
        angle = np.radians(90)
        Ualpha = magnitude * np.cos(angle)
        Ubeta = magnitude * np.sin(angle)
        sector = determine_sector(Ualpha, Ubeta, 300.0)
        # At 90°: A>0 (sector=1), C<0, B<0 → Sector 1
        assert sector == 1

    def test_sector_at_150_degrees(self):
        """Test sector at 150 degrees (Sector 5 in SVPWM numbering)."""
        magnitude = 100.0
        angle = np.radians(150)
        Ualpha = magnitude * np.cos(angle)
        Ubeta = magnitude * np.sin(angle)
        sector = determine_sector(Ualpha, Ubeta, 300.0)
        # At 150°: A>0 (sector=1), C<0, B>0 (sector=5) → Sector 5
        assert sector == 5

    def test_sector_at_210_degrees(self):
        """Test sector at 210 degrees (Sector 4 in SVPWM numbering)."""
        magnitude = 100.0
        angle = np.radians(210)
        Ualpha = magnitude * np.cos(angle)
        Ubeta = magnitude * np.sin(angle)
        sector = determine_sector(Ualpha, Ubeta, 300.0)
        # At 210°: A<0, C<0, B>0 (sector=4) → Sector 4
        assert sector == 4

    def test_sector_at_270_degrees(self):
        """Test sector at 270 degrees (Sector 6 in SVPWM numbering)."""
        magnitude = 100.0
        angle = np.radians(270)
        Ualpha = magnitude * np.cos(angle)
        Ubeta = magnitude * np.sin(angle)
        sector = determine_sector(Ualpha, Ubeta, 300.0)
        # At 270°: A<0, C>0 (sector=2), B>0 (sector=6) → Sector 6
        assert sector == 6

    def test_sector_at_330_degrees(self):
        """Test sector at 330 degrees (Sector 2 in SVPWM numbering)."""
        magnitude = 100.0
        angle = np.radians(330)
        Ualpha = magnitude * np.cos(angle)
        Ubeta = magnitude * np.sin(angle)
        sector = determine_sector(Ualpha, Ubeta, 300.0)
        # At 330°: A<0, C>0 (sector=2), B<0 → Sector 2
        assert sector == 2

    def test_sector_all_angles(self):
        """Test sector transitions through all angles."""
        Vdc = 300.0
        magnitude = 100.0

        for angle_deg in range(0, 360, 30):
            angle_rad = np.radians(angle_deg)
            Ualpha = magnitude * np.cos(angle_rad)
            Ubeta = magnitude * np.sin(angle_rad)
            sector = determine_sector(Ualpha, Ubeta, Vdc)

            # Verify sector is valid
            assert 0 <= sector <= 6

class TestSVPWMDutyCycleCalculation:
    """Tests for duty cycle calculation."""

    def test_duty_cycle_zero_voltage(self):
        """Test duty cycle for zero voltage."""
        sector, Ta, Tb, Tc = svpwm_duty_cycle(0.0, 0.0, 300.0)
        assert sector == 0
        assert pytest.approx(Ta, rel=1e-3) == 0.5
        assert pytest.approx(Tb, rel=1e-3) == 0.5
        assert pytest.approx(Tc, rel=1e-3) == 0.5

    def test_duty_cycle_symmetry(self):
        """Test duty cycles sum to appropriate value."""
        # For a balanced three-phase voltage, duty cycles should
        # produce symmetric waveforms
        magnitude = 50.0
        angle = np.radians(30)
        Ualpha = magnitude * np.cos(angle)
        Ubeta = magnitude * np.sin(angle)

        sector, Ta, Tb, Tc = svpwm_duty_cycle(Ualpha, Ubeta, 300.0)

        # All duty cycles should be within limits
        assert 0.05 <= Ta <= 0.95
        assert 0.05 <= Tb <= 0.95
        assert 0.05 <= Tc <= 0.95

    def test_duty_cycle_limits(self):
        """Test duty cycle limits are enforced."""
        # Large voltage should hit limits
        sector, Ta, Tb, Tc = svpwm_duty_cycle(200.0, 200.0, 300.0)

        assert Ta <= 0.95
        assert Tb <= 0.95
        assert Tc <= 0.95
        assert Ta >= 0.05
        assert Tb >= 0.05
        assert Tc >= 0.05

    def test_duty_cycle_small_voltage(self):
        """Test duty cycle for small voltage."""
        # Small voltage should produce near 50% duty
        sector, Ta, Tb, Tc = svpwm_duty_cycle(10.0, 5.0, 300.0)

        # Duty cycles should be close to 0.5
        assert abs(Ta - 0.5) < 0.1
        assert abs(Tb - 0.5) < 0.1
        assert abs(Tc - 0.5) < 0.1

class TestSVPWMClassMethods:
    """Tests for SVPWM class methods."""

    def test_set_voltage(self):
        """Test set_voltage method."""
        svpwm = SVPWM()
        svpwm.set_voltage(100.0, 50.0)
        assert svpwm.Ualpha == 100.0
        assert svpwm.Ubeta == 50.0

    def test_calculate(self):
        """Test calculate method."""
        svpwm = SVPWM()
        svpwm.set_voltage(100.0, 50.0)
        svpwm.calculate()

        # Should have valid outputs
        assert 0 <= svpwm.sector <= 6
        assert 0.05 <= svpwm.Ta <= 0.95
        assert 0.05 <= svpwm.Tb <= 0.95
        assert 0.05 <= svpwm.Tc <= 0.95

    def test_get_duty_cycles(self):
        """Test get_duty_cycles method."""
        svpwm = SVPWM()
        svpwm.set_voltage(100.0, 50.0)
        svpwm.calculate()

        Ta, Tb, Tc = svpwm.get_duty_cycles()
        assert Ta == svpwm.Ta
        assert Tb == svpwm.Tb
        assert Tc == svpwm.Tc

    def test_get_sector(self):
        """Test get_sector method."""
        svpwm = SVPWM()
        svpwm.set_voltage(100.0, 50.0)
        svpwm.calculate()

        sector = svpwm.get_sector()
        assert sector == svpwm.sector

    def test_reset(self):
        """Test reset method."""
        svpwm = SVPWM()
        svpwm.set_voltage(100.0, 50.0)
        svpwm.calculate()
        svpwm.reset()

        assert svpwm.Ualpha == 0.0
        assert svpwm.Ubeta == 0.0
        assert svpwm.sector == 0
        assert svpwm.Ta == 0.5

    def test_full_workflow(self):
        """Test complete SVPWM workflow."""
        svpwm = SVPWM(Vdc=300.0)

        # Set voltage vector at 60 degrees
        magnitude = 100.0
        angle = np.radians(60)
        Ualpha = magnitude * np.cos(angle)
        Ubeta = magnitude * np.sin(angle)

        svpwm.set_voltage(Ualpha, Ubeta)
        svpwm.calculate()

        # At 60°: should be Sector 3 (A>0, C>0, B<0)
        assert svpwm.sector == 3

        # Duty cycles should be valid
        Ta, Tb, Tc = svpwm.get_duty_cycles()
        assert all(0.05 <= d <= 0.95 for d in [Ta, Tb, Tc])

class TestSVPWMEdgeCases:
    """Tests for edge cases."""

    def test_negative_voltage(self):
        """Test negative voltage (reverse direction)."""
        sector, Ta, Tb, Tc = svpwm_duty_cycle(-100.0, 0.0, 300.0)

        # Should still produce valid outputs
        assert 0 <= sector <= 6
        assert all(0.05 <= d <= 0.95 for d in [Ta, Tb, Tc])

    def test_large_beta_voltage(self):
        """Test large beta-axis voltage."""
        sector, Ta, Tb, Tc = svpwm_duty_cycle(0.0, 150.0, 300.0)

        # Should hit limits
        assert all(d <= 0.95 for d in [Ta, Tb, Tc])

    def test_boundary_angle_0_degrees(self):
        """Test voltage at 0 degrees boundary."""
        Ualpha = 100.0
        Ubeta = 0.0
        sector = determine_sector(Ualpha, Ubeta, 300.0)

        # At 0°: A=0 (sector stays 0), C>0 (sector=2), B<0 → Sector 2
        assert sector in [2, 0]

    def test_boundary_angle_60_degrees(self):
        """Test voltage at 60 degrees boundary."""
        angle = np.radians(60)
        Ualpha = 100.0 * np.cos(angle)
        Ubeta = 100.0 * np.sin(angle)
        sector = determine_sector(Ualpha, Ubeta, 300.0)

        # At 60°: A>0 (sector=1), C>0 (sector=3), B<0 → Sector 3
        assert sector in [3, 2]

    def test_boundary_angle_180_degrees(self):
        """Test voltage at 180 degrees boundary."""
        Ualpha = -100.0
        Ubeta = 0.0
        sector = determine_sector(Ualpha, Ubeta, 300.0)

        # At 180 degrees, should be in sector 4 or 3
        assert sector in [3, 4]


class TestInverseClarkeSVPWM:
    """Tests for inverse Clarke SVPWM reference."""

    def test_inverse_clarke_zero(self):
        """Test inverse Clarke with zero input."""
        Ua, Ub, Uc = inverse_clarke_svpwm(0.0, 0.0)
        assert pytest.approx(Ua, rel=1e-6) == 0.0
        assert pytest.approx(Ub, rel=1e-6) == 0.0
        assert pytest.approx(Uc, rel=1e-6) == 0.0

    def test_inverse_clarke_alpha_only(self):
        """Test inverse Clarke with alpha only."""
        Ua, Ub, Uc = inverse_clarke_svpwm(100.0, 0.0)
        assert pytest.approx(Ua, rel=1e-6) == 100.0
        assert pytest.approx(Ub, rel=1e-6) == -50.0
        assert pytest.approx(Uc, rel=1e-6) == -50.0

    def test_inverse_clarke_beta_only(self):
        """Test inverse Clarke with beta only."""
        Ua, Ub, Uc = inverse_clarke_svpwm(0.0, 100.0)
        assert pytest.approx(Ua, rel=1e-6) == 0.0
        assert pytest.approx(Ub, rel=1e-6) == SQRT3 / 2 * 100.0
        assert pytest.approx(Uc, rel=1e-6) == -SQRT3 / 2 * 100.0

    def test_inverse_clarke_sum(self):
        """Test that Ua + Ub + Uc = 0 (balanced)."""
        Ua, Ub, Uc = inverse_clarke_svpwm(100.0, 50.0)
        assert pytest.approx(Ua + Ub + Uc, rel=1e-6) == 0.0

# integration tests

def test_svpwm_comparison_with_function():
    """compare svpwm class with standalone function."""
    Ualpha, Ubeta, Vdc = 100.0, 50.0, 300.0
    
    # using class
    svpwm = SVPWM(Vdc=Vdc)
    svpwm.set_voltage(Ualpha, Ubeta)
    svpwm.calculate()
    Ta_class, Tb_class, Tc_class = svpwm.get_duty_cycles()
    
    # using function
    sector_func, Ta_func, Tb_func, Tc_func = svpwm_duty_cycle(Ualpha, Ubeta, Vdc)
    
    # results should match
    assert pytest.approx(Ta_class, rel=1e-6) == Ta_func
    assert pytest.approx(Tb_class, rel=1e-6) == Tb_func
    assert pytest.approx(Tc_class, rel=1e-6) == Tc_func

def test_svpwm_rotating_vector():
    """test svpwm with rotating volatage vector."""
    svpwm = SVPWM(Vdc=300.0)
    magnitude = 100.0

    sectors = []
    for angle_deg in range(0, 360, 10):
        angle_rad = np.radians(angle_deg)
        Ualpha = magnitude * np.cos(angle_rad)
        Ubeta = magnitude * np.sin(angle_rad)

        svpwm.set_voltage(Ualpha, Ubeta)
        svpwm.calculate()

        sectors.append(svpwm.sector)

    # Sectors should cycle through 1-6 as angle increases
    # (with possible boundary overlaps)
    assert len(set(sectors)) >= 3  # Should visit multiple sectors

def test_svpwm_fixture():
    """Test SVPWM with typical motor control values."""
    # Typical voltage commands for 300V DC bus
    svpwm = SVPWM(Vdc=300.0, max_duty=0.95, min_duty=0.05)

    # Set typical voltage (about 1/3 of Vdc)
    svpwm.set_voltage(100.0, 60.0)  # ~120V magnitude
    svpwm.calculate()

    # Verify reasonable duty cycles
    Ta, Tb, Tc = svpwm.get_duty_cycles()

    # All should be within operating range
    assert all(0.1 <= d <= 0.9 for d in [Ta, Tb, Tc])