import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from core.steady_state import (
    calculate_critical_inductance,
    determine_mode,
    solve_ccm_ideal,
    solve_ccm_with_loss,
    solve_dcm_gain,
    solve_steady_state,
    solve_closed_loop_duty,
    D_MAX,
)

def test_ccm_ideal():
    Vout = solve_ccm_ideal(12.0, 0.5)
    expected = 24.0
    assert abs(Vout - expected) < 0.01, f"CCM ideal: expected {expected}, got {Vout}"

def test_ccm_with_loss():
    Vout, gain = solve_ccm_with_loss(12.0, 0.5, 10.0, 0.05, 0.5)
    ideal = solve_ccm_ideal(12.0, 0.5)
    assert Vout < ideal, f"Lossy Vout should be less than ideal: {Vout} vs {ideal}"
    assert Vout > 0, f"Vout should be positive: {Vout}"

def test_dcm_gain():
    L = 5e-6
    R = 10.0
    f = 100e3
    D = 0.3
    Vout, gain = solve_dcm_gain(12.0, D, R, L, f)
    Lc = calculate_critical_inductance(D, R, f)
    assert L < Lc, f"Should be DCM: L={L*1e6}μH < Lc={Lc*1e6}μH"
    assert Vout > 12.0, f"DCM Vout should be > Vin: {Vout}"

def test_critical_inductance():
    Lc = calculate_critical_inductance(0.5, 10.0, 100e3)
    expected = (1 - 0.5)**2 * 0.5 * 10.0 / (2 * 100e3)
    assert abs(Lc - expected) < 1e-10

def test_mode_determination():
    Lc = calculate_critical_inductance(0.5, 10.0, 100e3)
    assert determine_mode(Lc * 2, Lc) == "CCM"
    assert determine_mode(Lc * 0.5, Lc) == "DCM"

def test_d_max_limit():
    Vout, gain = solve_ccm_with_loss(12.0, 0.95, 10.0, 0.05)
    D_used = min(0.95, D_MAX)
    Vout_expected, _ = solve_ccm_with_loss(12.0, D_used, 10.0, 0.05)
    assert abs(Vout - Vout_expected) < 0.01, "D should be clamped to D_MAX"

def test_closed_loop_duty():
    D = solve_closed_loop_duty(12.0, 24.0, 10.0, 0.05, 0.5)
    assert D is not None
    assert 0.01 <= D <= D_MAX
    Vout, _ = solve_ccm_with_loss(12.0, D, 10.0, 0.05, 0.5)
    assert abs(Vout - 24.0) < 1.0, f"Closed loop Vout={Vout}, target=24.0"

def test_steady_state_ccm():
    r = solve_steady_state(12.0, 0.5, 10.0, 100e-6, 100e-6, 100e3, 0.05, 0.5)
    assert r["mode"] == "CCM"
    assert r["Vout"] > 12.0
    assert r["Vout"] < solve_ccm_ideal(12.0, 0.5)
    assert r["delta_IL"] > 0
    assert r["delta_Vout"] > 0
    assert 0 < r["efficiency"] <= 1

def test_steady_state_dcm():
    L_small = 10e-6
    r = solve_steady_state(12.0, 0.3, 10.0, L_small, 100e-6, 100e3, 0.05, 0.5)
    Lc = calculate_critical_inductance(0.3, 10.0, 100e3)
    if L_small < Lc:
        assert r["mode"] == "DCM"
    assert r["Vout"] > 12.0

def test_efficiency_loss():
    r_no_loss = solve_steady_state(12.0, 0.5, 10.0, 100e-6, 100e-6, 100e3, 0.0, 0.0)
    r_with_loss = solve_steady_state(12.0, 0.5, 10.0, 100e-6, 100e-6, 100e3, 1.0, 0.7)
    assert r_with_loss["efficiency"] < r_no_loss["efficiency"]

if __name__ == "__main__":
    test_ccm_ideal()
    test_ccm_with_loss()
    test_dcm_gain()
    test_critical_inductance()
    test_mode_determination()
    test_d_max_limit()
    test_closed_loop_duty()
    test_steady_state_ccm()
    test_steady_state_dcm()
    test_efficiency_loss()
    print("All 10 tests passed!")